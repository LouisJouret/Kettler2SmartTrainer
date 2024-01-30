# Copyright (c) 2024 Louis Jouret
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from server import FTMS_server
from kettler import KettlerRacer9
import numpy as np


def bytes_to_sint16(b1, b2):
    return int.from_bytes([b1, b2], byteorder='little', signed=True)


def byte_to_uint8(b):
    return int(b)


def decode_power_request(data):
    return bytes_to_sint16(data[1], data[2])


def decode_physical_param(data):
    # Decode the byte array according to the provided structure
    wind_speed_raw = bytes_to_sint16(data[1], data[2])
    grade_raw = bytes_to_sint16(data[3], data[4])
    crr_raw = byte_to_uint8(data[5])
    cw_raw = byte_to_uint8(data[6])

    # Apply resolution to each metric
    wind_speed = wind_speed_raw * 0.001  # Wind speed in meters per second
    grade = grade_raw * 0.01  # Grade in percentage
    crr = crr_raw * 0.0001  # Coefficient of Rolling Resistance (unitless)
    cw = cw_raw * 0.01  # Wind Resistance Coefficient in Kg/m

    return wind_speed, grade, crr, cw


def calculate_gear(mass, slope_percentage, rolling_resistance_coefficient, drag_coefficient, wind_speed, bike_speed):

    alpha = 1/55  # manual tuning parameter
    g = 9.81  # acceleration due to gravity (m/s²)
    air_density = 1.225  # air density at sea level (kg/m³)
    frontal_area = 0.5  # estimated frontal area in m²

    slope_radians = np.arctan(slope_percentage / 100)
    G = mass * g * np.sin(slope_radians)
    R = rolling_resistance_coefficient * mass * g * np.cos(slope_radians)

    total_wind_speed = bike_speed + wind_speed
    W = 0.5 * air_density * frontal_area * drag_coefficient * total_wind_speed**2

    total_resistance = G + R + W
    torque = alpha * total_resistance
    gear = -2.26 + 2.94*torque + 3.76*torque**2 - 1.2*torque**3 + 0.101*torque**4
    gear = int(gear)
    if gear > 20:
        gear = 20
    elif gear < 1:
        gear = 1
    return gear


def handle_request_00(server: FTMS_server, request, indoor_bike: KettlerRacer9):
    request.value = bytearray([0x80, 0x00, 0x01])
    server.update_value(server.fitness_machine_service_uuid,
                        server.control_point_char_uuid)


def handle_request_01(server: FTMS_server, request, indoor_bike: KettlerRacer9):
    request.value = bytearray([0x80, 0x01, 0x01])
    server.update_value(server.fitness_machine_service_uuid,
                        server.control_point_char_uuid)
    indoor_bike.erg_training = False


def handle_request_05(server: FTMS_server, request, indoor_bike: KettlerRacer9):
    if not indoor_bike.erg_mode:
        indoor_bike.put_in_erg_mode()
    target_power = decode_power_request(request.value)
    indoor_bike.set_power(target_power)
    request.value = bytearray([0x80, 0x05, 0x01])
    server.update_value(server.fitness_machine_service_uuid,
                        server.control_point_char_uuid)


def handle_request_07(server: FTMS_server, request, indoor_bike: KettlerRacer9):
    request.value = bytearray([0x80, 0x07, 0x01])
    indoor_bike.put_in_sim_mode()
    server.update_value(server.fitness_machine_service_uuid,
                        server.control_point_char_uuid)


def handle_request_11(server: FTMS_server, request, indoor_bike: KettlerRacer9):
    if not indoor_bike.sim_mode:
        indoor_bike.put_in_sim_mode()
    metrics = decode_physical_param(request.value)
    request.value = bytearray([0x80, 0x11, 0x01])
    server.update_value(server.fitness_machine_service_uuid,
                        server.control_point_char_uuid)
    if indoor_bike.erg_training:
        gear = 8
    else:
        gear = calculate_gear(
            mass=80,
            slope_percentage=metrics[1],
            rolling_resistance_coefficient=metrics[2],
            drag_coefficient=metrics[3],
            wind_speed=metrics[0],
            bike_speed=indoor_bike.speed
        )
    indoor_bike.set_gear(gear)


def default(*args, **kwargs):
    print("Unsupported request.")


case = {
    0x00: handle_request_00,
    0x01: handle_request_01,
    0x05: handle_request_05,
    0x07: handle_request_07,
    0x11: handle_request_11,
}

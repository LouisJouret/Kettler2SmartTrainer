import serial
import logging
import asyncio
import numpy as np
import time
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)
from typing import Any

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(name=__name__)


class KettlerUSB:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = serial.Serial(
            port=port,
            baudrate=57600,
            timeout=3
        )
        self.power = 0
        self.cadence = 0
        self.speed = 0
        self.wanted_power = 0
        self.writePower = False
        self.gear = 0
        self.writeGear = False
        self.erg_mode = False
        self.sim_mode = False
        self.erg_training = False
        self.reset_port()
        self.direct_write("RS")
        _ = self.port.readline().decode("utf-8").strip()
        time.sleep(8)
        self.direct_write("CM")
        _ = self.port.readline().decode("utf-8").strip()

    def open(self):
        if not self.port.is_open:
            self.port.open()

    def close(self):
        if self.port.is_open:
            self.port.close()

    def direct_write(self, data):
        if self.port.is_open:
            self.port.write(f"{data}\r\n".encode("utf-8"))

    def put_in_erg_mode(self):
        self.erg_mode = True
        self.erg_training = True
        self.sim_mode = False
        self.wanted_power = 25
        self.writePower = True
        self.writeGear = False
        print("Entering Erg Mode...")

    def put_in_sim_mode(self):
        self.sim_mode = True
        self.erg_mode = False
        self.gear = 5
        self.writeGear = True
        self.writePower = False
        print("Entering Sim Mode...")

    def request_state(self):
        if self.writePower:
            self.direct_write("CM")
            self.direct_write("PW" + str(self.wanted_power))
            self.writePower = False
        elif self.writeGear:
            self.direct_write("CM")
            self.direct_write("BL" + str(100 + self.gear))
            self.writeGear = False
        else:
            self.direct_write("ST")

    def set_power(self, power_to_write):
        self.wanted_power = max(0, int(power_to_write))
        self.writePower = True

    def set_gear(self, gear_to_write):
        self.gear = gear_to_write
        self.writeGear = True

    def process_response(self):
        if self.port.is_open:
            data = self.port.readline().decode("utf-8").strip()
            if data:
                metrics = data.split()
                if len(metrics) > 7:
                    self.power = int(metrics[-1])
                    self.cadence = 2*int(metrics[1])
                    self.speed = 0.1*int(metrics[2])

    def read(self):
        if self.port.is_open:
            self.request_state()
            self.process_response()

    def reset_port(self):
        if self.port.is_open:
            self.port.close()
        self.port.open()


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    logger.debug(f"Reading {characteristic.value}")
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    characteristic.value = value
    logger.debug(f"Char value set to {characteristic.value}")


def bytes_to_sint16(b1, b2):
    return int.from_bytes([b1, b2], byteorder='little', signed=True)


def byte_to_uint8(b):
    return int(b)


def decode_sim(data):
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


def decode_erg(data):
    return bytes_to_sint16(data[1], data[2])


def calculate_gear(mass, slope_percentage, rolling_resistance_coefficient, frontal_area, drag_coefficient, wind_speed, bike_speed):

    # Constants
    alpha = 1/55
    g = 9.81  # acceleration due to gravity (m/s²)
    air_density = 1.225  # air density at sea level (kg/m³)
    mass = 75  # rider + bike in kg
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


async def run():
    kettler = KettlerUSB()

    indoor_bike_char_uuid = "00002AD2-0000-1000-8000-00805F9B34FB"
    control_point_char_uuid = "00002AD9-0000-1000-8000-00805F9B34FB"
    fitness_machine_service_uuid = "00001826-0000-1000-8000-00805F9B34FB"
    gatt = {
        fitness_machine_service_uuid: {
            indoor_bike_char_uuid: {
                "Properties": (GATTCharacteristicProperties.read |
                               GATTCharacteristicProperties.notify),
                "Permissions": GATTAttributePermissions.readable,
                "Value": bytearray(
                    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            },
            control_point_char_uuid: {
                "Properties": (GATTCharacteristicProperties.write |
                               GATTCharacteristicProperties.indicate),
                "Permissions": GATTAttributePermissions.writeable,
                "Value": bytearray([0x00])
            }
        }

    }

    my_service_name = "KettlerRacer9"
    server = BlessServer(name=my_service_name)
    server.read_request_func = read_request
    server.write_request_func = write_request

    await server.add_gatt(gatt)
    await server.start()
    logger.debug("Server started, updating power value every second.")
    try:
        while True:
            now = time.time()
            request = server.get_characteristic(
                control_point_char_uuid)
            if request.value[0] == 0x00:
                request.value = bytearray([0x80, 0x00, 0x01])
                server.update_value(
                    fitness_machine_service_uuid, control_point_char_uuid)
            elif request.value[0] == 0x01:
                request.value = bytearray([0x80, 0x01, 0x01])
                server.update_value(
                    fitness_machine_service_uuid, control_point_char_uuid)
                kettler.erg_training = False
            elif request.value[0] == 0x05:
                if not kettler.erg_mode:
                    kettler.put_in_erg_mode()
                target_power = decode_erg(request.value)
                kettler.set_power(target_power)
                request.value = bytearray([0x80, 0x05, 0x01])
                server.update_value(
                    fitness_machine_service_uuid, control_point_char_uuid)
            elif request.value[0] == 0x07:
                request.value = bytearray([0x80, 0x07, 0x01])
                kettler.put_in_sim_mode()
                server.update_value(
                    fitness_machine_service_uuid, control_point_char_uuid)
            elif request.value[0] == 0x11:
                if not kettler.sim_mode:
                    kettler.put_in_sim_mode()
                metrics = decode_sim(request.value)
                request.value = bytearray([0x80, 0x11, 0x01])
                server.update_value(
                    fitness_machine_service_uuid, control_point_char_uuid)
                if kettler.erg_training:
                    gear = 8
                else:
                    gear = calculate_gear(
                        mass=75,
                        slope_percentage=metrics[1],
                        rolling_resistance_coefficient=metrics[2],
                        frontal_area=0.5,
                        drag_coefficient=metrics[3],
                        wind_speed=metrics[0],
                        bike_speed=kettler.speed
                    )
                print(gear)
                kettler.set_gear(gear)
            else:
                if not request.value[0] == 0x80:
                    print("Request not recognized: ", request.value)

            kettler.read()
            cadence_low_byte = (kettler.cadence) & 0xFF
            cadence_high_byte = (kettler.cadence >> 8) & 0xFF

            power_low_byte = kettler.power & 0xFF
            power_high_byte = (kettler.power >> 8) & 0xFF
            char_power = server.get_characteristic(indoor_bike_char_uuid)
            char_power.value = bytearray(
                [0x44, 0x02, 0x00, 0x03, cadence_low_byte, cadence_high_byte, power_low_byte, power_high_byte, 0x00, 0x00])
            server.update_value(
                fitness_machine_service_uuid, indoor_bike_char_uuid)
            print(time.time()-now)
            await asyncio.sleep(2)

    except asyncio.CancelledError:
        logger.debug("Stopping the server")
        kettler.direct_write("RS")
        _ = kettler.port.readline().decode("utf-8").strip()
        await server.stop()


asyncio.run(run())

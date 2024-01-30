# Copyright (c) 2024 Louis Jouret
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import logging
from typing import Any
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(name=__name__)


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    logger.debug(f"Reading {characteristic.value}")
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    characteristic.value = value
    logger.debug(f"Char value set to {characteristic.value}")


class FTMS_server(BlessServer(name="KettlerRacer9")):
    def __init__(self) -> None:
        super().__init__()
        self.indoor_bike_char_uuid = "00002AD2-0000-1000-8000-00805F9B34FB"
        self.control_point_char_uuid = "00002AD9-0000-1000-8000-00805F9B34FB"
        self.fitness_machine_service_uuid = "00001826-0000-1000-8000-00805F9B34FB"
        self.gatt = {
            self.fitness_machine_service_uuid: {
                self.indoor_bike_char_uuid: {
                    "Properties": (GATTCharacteristicProperties.read |
                                   GATTCharacteristicProperties.notify),
                    "Permissions": GATTAttributePermissions.readable,
                    "Value": bytearray(
                        [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
                },
                self.control_point_char_uuid: {
                    "Properties": (GATTCharacteristicProperties.write |
                                   GATTCharacteristicProperties.indicate),
                    "Permissions": GATTAttributePermissions.writeable,
                    "Value": bytearray([0x00])
                }
            }

        }

        self.read_request_func = read_request
        self.write_request_func = write_request

    def start(self) -> None:
        self.start()

    def stop(self) -> None:
        self.stop()

    def get_request(self) -> str:
        return self.get_characteristic(self.control_point_char_uuid)

    def update_value(self, service_uuid: str, char_uuid: str) -> None:
        self.update_value(service_uuid, char_uuid)

    def add_gatt(self, gatt: dict) -> None:
        self.add_gatt(gatt)

    def push_bike_metrics(self, cadence, power) -> None:
        cadence_low_byte = (cadence) & 0xFF
        cadence_high_byte = (cadence >> 8) & 0xFF
        power_low_byte = power & 0xFF
        power_high_byte = (power >> 8) & 0xFF
        char_power = self.get_characteristic(self.indoor_bike_char_uuid)
        char_power.value = bytearray(
            [0x44, 0x02, 0x00, 0x03, cadence_low_byte, cadence_high_byte, power_low_byte, power_high_byte, 0x00, 0x00])
        self.update_value(
            self.fitness_machine_service_uuid, self.indoor_bike_char_uuid)

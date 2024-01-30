# Copyright (c) 2024 Louis Jouret
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import serial


class KettlerRacer9:
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

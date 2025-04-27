import os
import platform
import numpy as np
from datetime import datetime

from Common.mod_manager import ModManager
from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames


class Battery:
    EMPTY = 0
    ERROR = 1
    UNKNOWN = 2
    DISCHARGE = 3
    CHARGE = 4

    # BMS switches off at 12000 mV (4 * 3.00 V)
    # That is very low. If stored in that state for a longer time, it will die
    # Set SW limits to 3.25 * 4 = 13000 mV
    STATE_NAMES = ["Empty", "Error", "Unknown", "Discharging", "Charging"]

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print(f"Adding {self.full_bodypart_name}")

        self.batterystate = Battery.ERROR
        self.oldstate = Battery.ERROR
        self.voltage = 0
        self.current = 0

    def print_state(self):
        txt = f"Battery : Voltage {self.voltage} mV, Current {self.current} mA, State = "
        txt += Battery.STATE_NAMES[self.batterystate]
        print(txt)

    def check_not_empty(self):
        battery_full_enough = self.batterystate >= Battery.DISCHARGE
        battery_full_enough = battery_full_enough and (self.voltage >= 13000)
        return battery_full_enough

    def new_data(self, data):
        try:
            new_byte_array_uint16 = data[3:3 + 4 * 2]
            new_byte_array_int16 = data[3 + (4 * 2):-2]

            uint16_array = np.frombuffer(new_byte_array_uint16, dtype=">u2")
            int16_array = np.frombuffer(new_byte_array_int16, dtype=">i2")

            battery_state = uint16_array[3]

            self.temperature = int16_array[0]  # First 16-bit integer
            self.current = int16_array[1]  # Third 16-bit integer
            self.voltage = int16_array[2]  # Second 16-bit integer

            # First set to unknown because not all bits are implemented.
            self.batterystate = Battery.UNKNOWN

            if (battery_state & 0x0F00) == 0x0100:
                self.batterystate = Battery.DISCHARGE

            if (battery_state & 0x0F00) == 0x0500:
                self.batterystate = Battery.CHARGE

            if (battery_state & 0x0F00) == 0x0C00:
                self.batterystate = Battery.EMPTY

            if self.batterystate != self.oldstate:
                self.print_state()

            self.oldstate = self.batterystate

            if not self.check_not_empty():
                print("WARNING: Battery is almost empty, recharge first.")

        except Exception as e:
            print(f"Battery data processing error: {e}")

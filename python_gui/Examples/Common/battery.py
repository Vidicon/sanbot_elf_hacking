import os
import platform
import numpy as np
from datetime import datetime

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
    state_names = ["Empty", "Error", "Unknown", "Discharging", "Charging"]

    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.batterystate = Battery.ERROR
        self.oldstate = Battery.ERROR
        self.Voltage = 0
        self.Current = 0

    def print_state(self):
        txt = "Battery    : Voltage {} mV, Current {} mA, State = ".format(self.Voltage, self.Current)
        txt += Battery.state_names[self.batterystate]
        print(txt)

    def check_not_empty(self):
        batteryFullEnough = self.batterystate >= Battery.DISCHARGE
        batteryFullEnough = batteryFullEnough and (self.Voltage >= 13000)
        return batteryFullEnough

    def new_data(self, data):
        try:
            new_byte_array_uint16 = data[3 : 3 + 4 * 2]
            new_byte_array_int16 = data[3 + (4 * 2) : -2]

            # hex_values = " ".join([format(x, "02X") for x in new_byte_array_uint16])
            # print("< " + hex_values)

            # hex_values = " ".join([format(x, "02X") for x in new_byte_array_int16])
            # print("< " + hex_values)

            unt16_array = np.frombuffer(new_byte_array_uint16, dtype=">u2")

            # DeviceType = unt16_array[0]
            # FW_Version = unt16_array[1]
            # HW_Version = unt16_array[2]
            BatteryState = unt16_array[3]

            # print(format(BatteryState, "02X"))

            int16_array = np.frombuffer(new_byte_array_int16, dtype=">i2")

            self.Temperature = int16_array[0]  # First 16-bit integer
            self.Current = int16_array[1]  # Third 16-bit integer
            self.Voltage = int16_array[2]  # Second 16-bit integer

            # First set to unknown because not all bits are implemented.
            self.batterystate = Battery.UNKNOWN

            if (BatteryState & 0x0F00) == 0x0100:
                self.batterystate = Battery.DISCHARGE

            if (BatteryState & 0x0F00) == 0x0500:
                self.batterystate = Battery.CHARGE

            if (BatteryState & 0x0F00) == 0x0C00:
                self.batterystate = Battery.EMPTY

            if self.batterystate != self.oldstate:
                self.print_state()

            self.oldstate = self.batterystate

            if not self.check_not_empty():
                print("WARNING : Battery is almost empty, recharge first.")

        except:
            print("Battery data processing error")

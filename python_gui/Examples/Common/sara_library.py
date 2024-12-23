import os
import platform
import numpy as np
from Common.mod_manager import ModManager
from Common.dist_sensors import DistanceSensors
from Common.compass import Compass
from Common.sara_common import *


class SaraRobot:
    # LEFTARM = 0
    # RIGHTARM = 1
    # BASE = 2
    # HEAD = 3
    # BODY = 4
    # BATTERY = 5
    # BODYDISTANCESENSORS = 6

    def __init__(self, com_windows1, com_windows2, com_linux1, com_linux2):
        print("-" * 80)
        self.com_windows1 = com_windows1
        self.com_windows2 = com_windows2
        self.com_linux1 = com_linux1
        self.com_linux2 = com_linux2

        self.start()

        self.left_arm = RobotArm(self.mod_manager, SaraRobotPartNames.LEFTARM)
        self.right_arm = RobotArm(self.mod_manager, SaraRobotPartNames.RIGHTARM)
        self.base = RobotArm(self.mod_manager, SaraRobotPartNames.BASE)
        self.battery = Battery(self.mod_manager, SaraRobotPartNames.BATTERY)
        self.body = Body(self.mod_manager, SaraRobotPartNames.BODY)

        print("-" * 80)

    def start(self):

        print("Starting robot communication")

        if "LINUX" in platform.system().upper():
            print("Linux detected")

            self.mod_manager = ModManager(port1=self.com_linux1, port2=self.com_linux2, baudrate=115200)
        else:
            print("Windows detected")

            self.mod_manager = ModManager(port1=self.windows1, port2=self.windows2, baudrate=115200)

        self.mod_manager.set_receive_callback(self.my_receive_callback)
        self.mod_manager.open_port()

        return

    def stop(self):
        self.mod_manager.close_port()

    def getversion(self):
        self.mod_manager.cmd_Generic(SaraRobotCommands.CMD_VERSION, 0, 0)
        return

    def my_receive_callback(self, data):
        # hex_values = " ".join([format(x, "02X") for x in data])
        # print("< " + hex_values)

        response = data[1]

        if response == (SaraRobotCommands.CMD_VERSION | SaraRobotCommands.RESP_BIT):
            try:
                string_from_bytearray = data[3:-2].decode("utf-8")
                print("Software version : " + string_from_bytearray)
            except:
                print("Version bytes error")

            print("-" * 80)

        if response == (SaraRobotCommands.CMD_GET_BATTERY | SaraRobotCommands.RESP_BIT):
            self.battery.new_data(data)

        if response == (SaraRobotCommands.CMD_GET_DISTANCESENSORS | SaraRobotCommands.RESP_BIT):
            self.body.distancesensors.new_data(data)

        if response == (SaraRobotCommands.CMD_GET_COMPASS | SaraRobotCommands.RESP_BIT):
            self.body.compass.new_data(data)

        if response == (SaraRobotCommands.CMD_COMP_MOVE | SaraRobotCommands.RESP_BIT):
            self.body.compass.rotate_absolute_ready(data)


class Body:
    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)
        self.distancesensors = DistanceSensors(self.mod_manager, bodypart)

        self.compass = Compass(self.mod_manager, bodypart)


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

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.batterystate = Battery.ERROR
        self.oldstate = Battery.ERROR
        self.Voltage = 0
        self.Current = 0

    def print_state(self):
        txt = "Battery : Voltage {} mV, Current {} mA, State = ".format(self.Voltage, self.Current)
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


class RobotArm:
    UP = 500
    FORWARD = 350
    DOWN = 100

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        self.led = ColorLed(self.mod_manager, self.bodypart)
        self.motor = Motor(self.mod_manager, self.bodypart)
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)


class Motor:

    CMD_LA_MOVE = 0x30
    CMD_RA_MOVE = 0x31
    CMD_BASE_MOVE = 0x32

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".motor"
        print("Adding " + self.full_bodypart_name)

    def move(self, position):

        if position < 0:
            print(self.full_bodypart_name + " : Error --> Position < 0")
            return

        if position > 500:
            print(self.full_bodypart_name + " : Error --> Position > 500")
            return

        if self.bodypart == SaraRobot.LEFTARM:
            position *= -1
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF
            self.mod_manager.cmd_Generic(Motor.CMD_LA_MOVE, 2, np.array([high, low]))

        if self.bodypart == SaraRobot.RIGHTARM:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.mod_manager.cmd_Generic(Motor.CMD_RA_MOVE, 2, np.array([high, low]))


class ColorLed:
    NOCOLOR = 0
    RED = 1
    GREEN = 2
    BLUE = 3
    WHITE = 4
    REDGREEN = 5

    LED_NONE = 0
    LED_OFF = 1
    LED_ON = 2
    LED_BLINK_OFF = 3
    LED_BLINK_SLOW = 4
    LED_BLINK_FAST = 5
    LED_BLINK_VERYFAST = 6

    CMD_LA_COLOR = 0x10
    CMD_RA_COLOR = 0x11
    CMD_BASE_COLOR = 0x12

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        print("Adding " + bodypart_to_string(bodypart) + ".led")

    def setcolor(self, color=0, blink=0):

        if self.bodypart == SaraRobot.LEFTARM:
            Parameters = np.array([ColorLed.CMD_LA_COLOR, color, blink])

        if self.bodypart == SaraRobot.RIGHTARM:
            Parameters = np.array([ColorLed.CMD_RA_COLOR, color, blink])

        if self.bodypart == SaraRobot.BASE:
            Parameters = np.array([ColorLed.CMD_BASE_COLOR, color, blink])

        self.mod_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

        return

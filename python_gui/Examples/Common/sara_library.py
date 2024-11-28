import os
import platform
import numpy as np
from Common.mod_manager import ModManager

body_parts_names = ["left_arm", "right_arm", "base", "head", "body"]

RESP_BIT = 0x80
CMD_VERSION = 0x01


def bodypart_to_string(bodypart):
    return body_parts_names[bodypart]


class SaraRobot:
    LEFTARM = 0
    RIGHTARM = 1
    BASE = 2
    HEAD = 3
    BODY = 4

    def __init__(self, com_windows1, com_windows2, com_linux1, com_linux2):
        print("-" * 80)
        self.com_windows1 = com_windows1
        self.com_windows2 = com_windows2
        self.com_linux1 = com_linux1
        self.com_linux2 = com_linux2

        self.start()

        self.left_arm = RobotArm(self.mod_manager, SaraRobot.LEFTARM)
        self.right_arm = RobotArm(self.mod_manager, SaraRobot.RIGHTARM)
        self.base = RobotArm(self.mod_manager, SaraRobot.BASE)
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
        self.mod_manager.cmd_Generic(CMD_VERSION, 0, 0)
        return

    def my_receive_callback(self, data):
        # hex_values = " ".join([format(x, "02X") for x in data])
        # print("< " + hex_values)

        # Decode
        response = data[1]

        if response == (CMD_VERSION | RESP_BIT):
            try:
                string_from_bytearray = data[3:-2].decode("utf-8")
                print("Software version : " + string_from_bytearray)
            except:
                print("Version bytes error")

            print("-" * 80)


class RobotArm:

    UP = 500
    FORWARD = 350
    DOWN = 100

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        self.led = ColorLed(self.mod_manager, self.bodypart)
        self.motor = Motor(self.mod_manager, self.bodypart)
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".arm"
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
    RED = 1
    GREEN = 2
    BLUE = 3
    WHITE = 4
    ALL = 5

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

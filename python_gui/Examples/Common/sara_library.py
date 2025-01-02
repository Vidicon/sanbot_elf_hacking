import platform
import numpy as np
from Common.mod_manager import ModManager
from Common.distance_sensors import DistanceSensors
from Common.compass import Compass
from Common.battery import Battery
from Common.colorled import ColorLed
from Common.robot_base import RobotBase

# from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands

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
        self.base = RobotBase(self.mod_manager, SaraRobotPartNames.BASE)
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

            self.mod_manager = ModManager(port1=self.com_windows1, port2=self.com_windows2, baudrate=115200)

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


class RobotArm:
    UP = 500
    FORWARD = 350
    DOWN = 100

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.led = ColorLed(self.mod_manager, self.bodypart)
        self.motor = RobotArmMotor(self.mod_manager, self.bodypart)


class RobotArmMotor:
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
            self.mod_manager.cmd_Generic(RobotArmMotor.CMD_LA_MOVE, 2, np.array([high, low]))

        if self.bodypart == SaraRobot.RIGHTARM:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.mod_manager.cmd_Generic(RobotArmMotor.CMD_RA_MOVE, 2, np.array([high, low]))

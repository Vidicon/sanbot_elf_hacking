import platform
import numpy as np

from Common.distance_sensors import DistanceSensors
from Common.compass import Compass
from Common.battery import Battery
from Common.colorled import ColorLed
from Common.robot_base import RobotBase
from Common.encoders import Encoders
from Common.motionsensors import MotionSensors

# from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.sara_common import RobotArmPositions

from Common.bridge_manager import BridgeManager
from Common.sara_ports import SaraRobotPorts


class SaraRobot:
    def __init__(
        self,
        logging=True,
    ):
        print("-" * 80)
        self.com_windows1 = SaraRobotPorts.COM_HEAD_WINDOWS
        self.com_windows2 = SaraRobotPorts.COM_BODY_WINDOWS
        self.com_linux1 = SaraRobotPorts.COM_HEAD_LINUX
        self.com_linux2 = SaraRobotPorts.COM_BODY_LINUX
        self.logging = logging

        self.start()

        self.left_arm = RobotArm(self.bridge_manager, SaraRobotPartNames.LEFTARM)
        self.right_arm = RobotArm(self.bridge_manager, SaraRobotPartNames.RIGHTARM)
        self.base = RobotBase(self.bridge_manager, SaraRobotPartNames.BASE)
        self.battery = Battery(self.bridge_manager, SaraRobotPartNames.BATTERY)
        self.body = Body(self.bridge_manager, SaraRobotPartNames.BODY)
        self.head = Head(self.bridge_manager, SaraRobotPartNames.HEAD)

        print("-" * 80)

    def start(self):
        print("Starting robot communication")

        if "LINUX" in platform.system().upper():
            print("Linux detected")
            self.bridge_manager = BridgeManager(
                port1=self.com_linux1, port2=self.com_linux2, baudrate=115200
            )
        else:
            print("Windows detected")
            self.bridge_manager = BridgeManager(
                port1=self.com_windows1, port2=self.com_windows2, baudrate=115200
            )

        self.bridge_manager.set_receive_callback(self.my_receive_callback)
        self.bridge_manager.connect()

        print("-" * 80)

        return

    def stop(self):
        self.base.brake(ApplyBrake=False)
        self.bridge_manager.disconnect()

    def my_receive_callback(self, data):
        response = data[1]

        # if (data[2] + 5) != len(data):
        #     print("Error: data length mismatch")
        #     print("Expected length: " + str(data[2] + 5))
        #     print("Received length: " + str(len(data)))
        #     print("Data: " + data.hex())

        if response == (SaraRobotCommands.CMD_VERSION | SaraRobotCommands.RESP_BIT):
            self.new_version_data(data)
            return

        if response == (SaraRobotCommands.CMD_GET_BATTERY | SaraRobotCommands.RESP_BIT):
            self.battery.new_data(data)
            if self.logging:
                self.battery.print_state()
            return

        if response == (
            SaraRobotCommands.CMD_GET_DISTANCESENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.body.distancesensors.new_data(data)
            if self.logging:
                self.body.distancesensors.print_values()
            return

        if response == (SaraRobotCommands.CMD_GET_COMPASS | SaraRobotCommands.RESP_BIT):
            self.body.compass.new_data(data)
            if self.logging:
                self.body.compass.print_values()
            return

        if response == (SaraRobotCommands.CMD_COMP_MOVE | SaraRobotCommands.RESP_BIT):
            self.body.compass.rotate_absolute_ready(data)
            return

        if response == (
            SaraRobotCommands.CMD_GET_ENCODERS | SaraRobotCommands.RESP_BIT
        ):
            self.body.encoders.new_data(data)
            if self.logging:
                self.body.encoders.print_values()
            return

        if response == (
            SaraRobotCommands.CMD_GET_MOTIONSENSORS | SaraRobotCommands.RESP_BIT
        ):
            self.body.motionsensors.new_data(data)
            if self.logging:
                self.body.motionsensors.print_values()
            return

        # If not decoded, print the data
        hex_values = " ".join([format(x, "02X") for x in data])
        print("< " + hex_values)

    def new_version_data(self, data):
        try:
            string_from_bytearray = data[3:-2].decode("utf-8")
            print("-" * 80)
            print("Software version : " + string_from_bytearray)
        except:
            print("Version bytes error")

        print("-" * 80)
        return


class Body:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.distancesensors = DistanceSensors(self.bridge_manager, bodypart)
        self.compass = Compass(self.bridge_manager, bodypart)
        self.encoders = Encoders(self.bridge_manager, bodypart)
        self.motionsensors = MotionSensors(
            self.bridge_manager, bodypart)

    def getversion(self):
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_VERSION, 0, 0, SaraRobotPartNames.BODY
        )


class Head:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

    def getversion(self):
        self.bridge_manager.cmd_Generic(
            SaraRobotCommands.CMD_VERSION, 0, 0, SaraRobotPartNames.HEAD
        )


class RobotArm:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.bodypart = bodypart
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.led = ColorLed(self.bridge_manager, self.bodypart)
        self.motor = RobotArmMotor(self.bridge_manager, self.bodypart)


class RobotArmMotor:
    CMD_LA_MOVE = 0x30
    CMD_RA_MOVE = 0x31
    CMD_BASE_MOVE = 0x32

    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
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

        if self.bodypart == SaraRobotPartNames.LEFTARM:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF
            self.bridge_manager.cmd_Generic(
                RobotArmMotor.CMD_LA_MOVE, 2, np.array([high, low])
            )

        if self.bodypart == SaraRobotPartNames.RIGHTARM:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.bridge_manager.cmd_Generic(
                RobotArmMotor.CMD_RA_MOVE, 2, np.array([high, low])
            )

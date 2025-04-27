import platform
import numpy as np
from Common.distance_sensors import DistanceSensors
from Common.compass import Compass
from Common.battery import Battery
from Common.colorled import ColorLed
from Common.robot_base import RobotBase

from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.sara_common import RobotArmPositions

from Common.bridge_manager import BridgeManager
import time

class SaraRobot:
    # LEFTARM = 0
    # RIGHTARM = 1
    # BASE = 2
    # HEAD = 3
    # BODY = 4
    # BATTERY = 5
    # BODYDISTANCESENSORS = 6

    def __init__(self, remote_host):
        print("-" * 80)
        self.remote_host = remote_host

        self.start()

        self.left_arm = RobotArm(self.bridge_manager, SaraRobotPartNames.LEFTARM)
        self.right_arm = RobotArm(self.bridge_manager, SaraRobotPartNames.RIGHTARM)
        self.base = RobotBase(self.bridge_manager, SaraRobotPartNames.BASE)
        self.battery = Battery(self.bridge_manager, SaraRobotPartNames.BATTERY)
        self.body = Body(self.bridge_manager, SaraRobotPartNames.BODY)
        self.head = Head(self.bridge_manager, SaraRobotPartNames.HEAD)  

        time.sleep(2)

        print("-" * 80)

    def start(self):
        print("Starting robot communication")

        self.bridge_manager = BridgeManager(self.remote_host)
        self.bridge_manager.set_receive_callback(self.my_receive_callback)
        self.bridge_manager.connect()
        return

    def stop(self):
        self.base.brake(ApplyBrake=False)
        self.bridge_manager.disconnect()

    def my_receive_callback(self, data, debug=False):
        if debug:
            hex_values = " ".join([format(x, "02X") for x in data])
            print("< " + hex_values)

        datalength_cmd = data[2]
        datalength_data = len(data)   

        self.process_response(data)

        # # Check if we have a second command
        # while (datalength_cmd + 5) < datalength_data:
        #     data = data[datalength_cmd + 5:]
        #     print("Copied remaining data to a new_buffer")

        #     hex_values = " ".join([format(x, "02X") for x in data])
        #     print("< " + hex_values)

        #     datalength_cmd = data[2]
        #     datalength_data = len(data)  

        #     # Process first command
        #     print(f"Command byte (data[1]): {data[1]:02X}")


        #     # print(f"Byte 2+3+2 : {datalength_cmd+3+2}, {len(data)}")

        #     self.process_response(data)
        return

    def process_response(self, new_buffer):
        if (new_buffer[0] != 0xD5):
            return

        response = new_buffer[1]

        # print(f"Response byte (new_buffer[1]): {response:02X}")

        if response == (SaraRobotCommands.CMD_VERSION | SaraRobotCommands.RESP_BIT):
            try:
                string_from_bytearray = new_buffer[3:-2].decode("utf-8")
                print("Software version : " + string_from_bytearray)
            except:
                print("Version bytes error")

            print("-" * 80)

        if response == (SaraRobotCommands.CMD_GET_BATTERY | SaraRobotCommands.RESP_BIT):
            self.battery.new_data(new_buffer)

        if response == (SaraRobotCommands.CMD_GET_DISTANCESENSORS | SaraRobotCommands.RESP_BIT):
            self.body.distancesensors.new_data(new_buffer)

        if response == (SaraRobotCommands.CMD_GET_COMPASS | SaraRobotCommands.RESP_BIT):
            self.body.compass.new_data(new_buffer)

        if response == (SaraRobotCommands.CMD_COMP_MOVE | SaraRobotCommands.RESP_BIT):
            self.body.compass.rotate_absolute_ready(new_buffer)

class Body:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.distancesensors = DistanceSensors(self.bridge_manager, bodypart)
        self.compass = Compass(self.bridge_manager, bodypart)

    def getversion(self):
        self.bridge_manager.cmd_Generic(SaraRobotCommands.CMD_VERSION, 0, 0, SaraRobotPartNames.BODY)


class Head:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

    def getversion(self):
        self.bridge_manager.cmd_Generic(SaraRobotCommands.CMD_VERSION, 0, 0, SaraRobotPartNames.HEAD)


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
            self.bridge_manager.cmd_Generic(RobotArmMotor.CMD_LA_MOVE, 2, np.array([high, low]))

        if self.bodypart == SaraRobotPartNames.RIGHTARM:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.bridge_manager.cmd_Generic(RobotArmMotor.CMD_RA_MOVE, 2, np.array([high, low]))

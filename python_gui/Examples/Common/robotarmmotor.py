import platform
import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.sara_common import RobotArmPositions

from Common.bridge_manager import BridgeManager
from Common.sara_ports import SaraRobotPorts

class RobotArmMotor:
    def __init__(self, bridge_manager, parent_name, instance_ENUM, maximum_position=None, minimum_position=None):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        if (maximum_position is not None) and (minimum_position is not None):
            self.maximum_position = maximum_position
            self.minimum_position = minimum_position
        else:
            self.maximum_position = 500
            self.minimum_position = 0

    def move(self, position):
        if position < self.minimum_position:
            print(bodypart_to_string(self.instance_ENUM) + " : Error --> Position < 0")
            return

        if position > self.maximum_position:
            print(bodypart_to_string(self.instance_ENUM) + " : Error --> Position > 500")
            return

        if self.instance_ENUM == SaraRobotPartNames.LEFT_ARM_MOTOR:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF
            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_LA_MOVE, 2, np.array([high, low])
            )

        if self.instance_ENUM == SaraRobotPartNames.RIGHT_ARM_MOTOR:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_RA_MOVE, 2, np.array([high, low])
            )

        if self.instance_ENUM == SaraRobotPartNames.PAN_MOTOR:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_HEAD_PAN_MOVE, 2, np.array([high, low])
            )

        if self.instance_ENUM == SaraRobotPartNames.TILT_MOTOR:
            high = (int(position) >> 8) & 0xFF
            low = int(position) & 0xFF

            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_HEAD_TILT_MOVE, 2, np.array([high, low])
            )

    def home(self):
        if self.instance_ENUM == SaraRobotPartNames.LEFT_ARM_MOTOR:
            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_LA_HOME, 0, np.array([0, 0])
            )

        if self.instance_ENUM == SaraRobotPartNames.RIGHT_ARM_MOTOR:
            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_RA_HOME, 0, np.array([0, 0])
            )

        if self.instance_ENUM == SaraRobotPartNames.PAN_MOTOR:
            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_HEAD_PAN_HOME, 0, np.array([0, 0])
            )

        if self.instance_ENUM == SaraRobotPartNames.TILT_MOTOR:
            self.bridge_manager.cmd_Generic(
                SaraRobotCommands.CMD_HEAD_TILT_HOME, 0, np.array([0, 0])
            )

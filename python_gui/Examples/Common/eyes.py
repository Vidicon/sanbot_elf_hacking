import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class HeadEyes:
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

    def set_eyes(self, left_eye=None, right_eye=0):
        parameters = np.array([SaraRobotCommands.CMD_HEAD_EYES, left_eye, right_eye])
        self.bridge_manager.cmd_Generic(parameters[0], 2, np.array(parameters[1:]))
        return

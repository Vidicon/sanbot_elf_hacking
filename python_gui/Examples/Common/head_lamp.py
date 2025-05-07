import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class HeadLamp:
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

    def set_lamp(self, lamp_on=False):
        parameters = np.array([SaraRobotCommands.CMD_HEAD_LAMP, lamp_on])
        self.bridge_manager.cmd_Generic(parameters[0], 1, np.array(parameters[1:]))
        return

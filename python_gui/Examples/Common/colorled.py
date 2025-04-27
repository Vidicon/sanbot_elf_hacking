import os
import platform
import numpy as np
from datetime import datetime

# from Common.mod_manager import ModManager
from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames

from Common.bridge_manager import BridgeManager

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

    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.bodypart = bodypart
        print("Adding " + bodypart_to_string(bodypart) + ".led")

    def setcolor(self, color=0, blink=0):

        if self.bodypart == SaraRobotPartNames.LEFTARM:
            Parameters = np.array([ColorLed.CMD_LA_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.RIGHTARM:
            Parameters = np.array([ColorLed.CMD_RA_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.BASE:
            Parameters = np.array([ColorLed.CMD_BASE_COLOR, color, blink])

        self.bridge_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

        return

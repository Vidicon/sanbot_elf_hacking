import os
import platform
import numpy as np
from datetime import datetime

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands

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

    def __init__(self, bridge_manager, bodypart, option=""):
        self.bridge_manager = bridge_manager
        self.bodypart = bodypart
        self.option = option

        if (len(self.option) == 0):
            self.bodypart = bodypart
            print("Adding " + "robot." + bodypart_to_string(bodypart) + ".led")
        else:   
            if (self.option == "left_led"):
                self.bodypart = SaraRobotPartNames.LEFT_LED
                print("Adding " + "robot." + bodypart_to_string(bodypart) + ".left_led")
            elif (self.option == "right_led"):
                self.bodypart = SaraRobotPartNames.RIGHT_LED
                print("Adding " + "robot." + bodypart_to_string(bodypart) + ".right_led")
            else:
                print("Invalid option for ColorLed: " + self.option)
                return  

    def setcolor(self, color=0, blink=0):

        if self.bodypart == SaraRobotPartNames.LEFTARM:
            Parameters = np.array([SaraRobotCommands.CMD_LA_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.RIGHTARM:
            Parameters = np.array([SaraRobotCommands.CMD_RA_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.BASE:
            Parameters = np.array([SaraRobotCommands.CMD_BASE_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.LEFT_LED:
            Parameters = np.array([SaraRobotCommands.CMD_HEAD_LEFT_COLOR, color, blink])

        if self.bodypart == SaraRobotPartNames.RIGHT_LED:
            Parameters = np.array([SaraRobotCommands.CMD_HEAD_RIGHT_COLOR, color, blink])

        self.bridge_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

        return

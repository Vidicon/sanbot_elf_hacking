import sys
import os
import time

from Common.sara_library_bridged import *

# NOCOLOR = 0
# RED = 1
# GREEN = 2
# BLUE = 3
# WHITE = 4

# LED_NONE = 0
# LED_OFF = 1
# LED_ON = 2
# LED_BLINK_OFF = 3
# LED_BLINK_SLOW = 4
# LED_BLINK_FAST = 5
# LED_BLINK_VERYFAST = 6


def main():
    robot = SaraRobot('saradev.local')

    robot.getversion()

    robot.left_arm.led.setcolor(color=ColorLed.RED, blink=ColorLed.LED_BLINK_VERYFAST)
    robot.right_arm.led.setcolor(color=ColorLed.GREEN, blink=ColorLed.LED_BLINK_SLOW)
    robot.base.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_FAST)

    # Add HEAD LEFT
    # Add HEAD RIGHT

    time.sleep(5)

    robot.left_arm.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_ON)
    robot.right_arm.led.setcolor(color=ColorLed.WHITE, blink=ColorLed.LED_ON)
    robot.base.led.setcolor(color=ColorLed.WHITE, blink=ColorLed.LED_ON)

    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

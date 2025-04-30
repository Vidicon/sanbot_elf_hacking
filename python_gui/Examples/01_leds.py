import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()

    robot.left_arm.led.setcolor(color=ColorLed.RED, blink=ColorLed.LED_BLINK_VERYFAST)
    robot.right_arm.led.setcolor(color=ColorLed.GREEN, blink=ColorLed.LED_BLINK_SLOW)
    robot.base.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_FAST)
    
    robot.head.left_led.setcolor(
        color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_FAST
    )  
    robot.head.right_led.setcolor(
        color=ColorLed.GREEN, blink=ColorLed.LED_BLINK_SLOW
    )  

    time.sleep(3)

    robot.left_arm.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_ON)
    robot.right_arm.led.setcolor(color=ColorLed.WHITE, blink=ColorLed.LED_ON)
    robot.base.led.setcolor(color=ColorLed.WHITE, blink=ColorLed.LED_ON)

    robot.head.left_led.setcolor(
        color=ColorLed.WHITE, blink=ColorLed.LED_ON
    )  
    robot.head.right_led.setcolor(
        color=ColorLed.RED, blink=ColorLed.LED_ON
    )  

    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

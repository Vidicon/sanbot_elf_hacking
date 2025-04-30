import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=True)

    robot.head.getversion()
    robot.body.getversion()

    robot.left_arm.motor.home()
    robot.right_arm.motor.home()
    
    robot.head.pan_motor.home()
    robot.head.tilt_motor.home()
    
    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

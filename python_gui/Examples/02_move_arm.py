import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM2", "/dev/ttyACM1")

    robot.left_arm.motor.move(position=RobotArm.UP)
    robot.right_arm.motor.move(position=RobotArm.UP)

    time.sleep(5)

    robot.left_arm.motor.move(position=RobotArm.FORWARD)
    robot.right_arm.motor.move(position=RobotArm.DOWN)

    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

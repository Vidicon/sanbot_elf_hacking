import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    robot.left_arm.motor.move(position=RobotArm.UP)
    robot.right_arm.motor.move(position=RobotArm.UP)

    time.sleep(5)

    robot.left_arm.motor.move(position=RobotArm.FORWARD)
    robot.right_arm.motor.move(position=RobotArm.DOWN)

    time.sleep(5)

    robot.left_arm.motor.move(position=220)
    robot.right_arm.motor.move(position=300)

    robot.stop()

    return


if __name__ == "__main__":
    main()

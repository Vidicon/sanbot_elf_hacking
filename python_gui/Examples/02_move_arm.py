import sys
import os
import time

from Common.sara_library import SaraRobot
from Common.sara_library import SaraRobotPartNames
from Common.sara_common import RobotArmPositions


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    robot.left_arm.motor.move(position=RobotArmPositions.UP)
    robot.right_arm.motor.move(position=RobotArmPositions.UP)

    time.sleep(3)

    robot.left_arm.motor.move(position=RobotArmPositions.FORWARD)
    robot.right_arm.motor.move(position=RobotArmPositions.FORWARD)

    time.sleep(3)

    robot.left_arm.motor.move(position=RobotArmPositions.DOWN)
    robot.right_arm.motor.move(position=RobotArmPositions.DOWN)

    time.sleep(3)

    # robot.left_arm.motor.move(position=220)
    # robot.right_arm.motor.move(position=300)

    robot.stop()

    return


if __name__ == "__main__":
    main()

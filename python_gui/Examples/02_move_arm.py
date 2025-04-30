import sys
import os
import time

from Common.sara_library import SaraRobot
from Common.sara_library import SaraRobotPartNames
from Common.sara_common import RobotArmPositions


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()


    robot.left_arm.motor.move(position=RobotArmPositions.UP)
    robot.right_arm.motor.move(position=RobotArmPositions.UP)

    time.sleep(3)

    robot.left_arm.motor.move(position=RobotArmPositions.FORWARD)
    robot.right_arm.motor.move(position=RobotArmPositions.FORWARD)

    time.sleep(3)

    robot.left_arm.motor.move(position=RobotArmPositions.DOWN)
    robot.right_arm.motor.move(position=RobotArmPositions.DOWN)

    time.sleep(2)
    robot.stop()

    return


if __name__ == "__main__":
    main()

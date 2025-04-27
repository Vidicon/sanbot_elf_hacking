import sys
import os
import time

from Common.sara_common import RobotArmPositions
from Common.sara_library_bridged import *

def main():
    robot = SaraRobot('saradev.local')

    robot.head.getversion()
    robot.body.getversion()

    robot.left_arm.motor.move(position=RobotArmPositions.UP)
    robot.right_arm.motor.move(position=RobotArmPositions.UP)

    time.sleep(5)

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

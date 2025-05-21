import sys
import os
import time

from Common.sara_library import SaraRobot
from Common.sara_library import SaraRobotPartNames
from Common.sara_common import RobotArmPositions
from Common.sara_common import RobotHeadPositions


def main():
    robot = SaraRobot(logging=False)

    time.sleep(1)

    robot.head.getversion()
    robot.body.getversion()

    robot.head.pan_motor.move(position=RobotHeadPositions.PAN_LEFT)
    robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_UP)
    
    time.sleep(5)

    robot.head.pan_motor.move(position=RobotHeadPositions.PAN_RIGHT)
    robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_DOWN)

    time.sleep(5)

    robot.head.pan_motor.move(position=RobotHeadPositions.PAN_MID)
    robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_MID)

    time.sleep(2)
    robot.stop()

    return


if __name__ == "__main__":
    main()

import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()

    robot.head.eyes.set_eyes(left_eye=1, right_eye=2)

    time.sleep(5)

    robot.head.eyes.set_eyes(left_eye=2, right_eye=3)

    time.sleep(5)

    robot.head.eyes.set_eyes(left_eye=3, right_eye=3)

    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

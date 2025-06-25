import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()

    robot.head.lamp.set_lamp(lamp_on=True)

    time.sleep(5)

    robot.head.lamp.set_lamp(lamp_on=False)

    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

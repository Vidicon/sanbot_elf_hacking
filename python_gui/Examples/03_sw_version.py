import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM1", "/dev/ttyACM2")

    robot.getversion()

    time.sleep(3)

    robot.stop()

    return


if __name__ == "__main__":
    main()

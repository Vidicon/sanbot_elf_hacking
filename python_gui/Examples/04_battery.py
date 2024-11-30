import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()
    time.sleep(3)

    while 1:
        batterij = robot.battery.getstate()
        time.sleep(1)

    robot.stop()

    return


if __name__ == "__main__":
    main()

import sys
import os
import time

from Common.sara_library import *

def main():
    robot = SaraRobot("COM10", "COM11", "/dev/ttyACM1", "/dev/ttyACM0", logging=False)

    time.sleep(1)

    robot.body.getversion()
    robot.head.getversion()

    time.sleep(10)
    robot.stop()

    return


if __name__ == "__main__":
    main()

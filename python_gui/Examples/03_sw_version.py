import sys
import os
import time

from Common.sara_library_bridged import *


def main():
    robot = SaraRobot('saradev.local')

    robot.getversion()
    # robot.head.getversion()

    time.sleep(10)

    robot.stop()

    return


if __name__ == "__main__":
    main()

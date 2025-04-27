import sys
import os
import time

from Common.sara_library_bridged import *


def main():
    robot = SaraRobot('saradev.local')

    robot.head.getversion()
    robot.body.getversion()

    time.sleep(5)

    robot.stop()

    return


if __name__ == "__main__":
    main()

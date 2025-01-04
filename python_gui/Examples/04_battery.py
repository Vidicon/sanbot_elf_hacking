import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    # Wait 2 seconds for the robot to connect and receive the first data
    time.sleep(2)

    while 1:
        time.sleep(1)
        # Print some battery information to the console
        robot.battery.printstate()

        # Check if the battery is full enough to use.
        batteryNotEmpty = robot.battery.checkNotEmpty()
        print(batteryNotEmpty)

    robot.stop()

    return


if __name__ == "__main__":
    main()

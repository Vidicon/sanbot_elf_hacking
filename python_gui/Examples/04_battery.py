import sys
import os
import time

from Common.sara_library_bridged import *

def main():
    robot = SaraRobot('saradev.local')

    robot.head.getversion()
    robot.body.getversion()

    # Wait 2 seconds for the robot to connect and receive the first data
    time.sleep(2)

    while 1:
        time.sleep(1)
        # Print some battery information to the console
        robot.battery.print_state()

        # Check if the battery is full enough to use.
        batteryNotEmpty = robot.battery.check_not_empty()
        print(batteryNotEmpty)

    robot.stop()

    return


if __name__ == "__main__":
    main()

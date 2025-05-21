import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()


    while True:
        touchsensors = robot.head.touch_sensors.get_all_values()
        print("Touch sensors:", [int(value) for value in touchsensors])

        if (touchsensors[4]):
            print("Touch front right detected!")
        if (touchsensors[0]):
            print("Touch front left detected!")
        if (touchsensors[1]):   
            print("Touch rear left detected!")
        if (touchsensors[5]):   
            print("Touch rear right detected!")      
          
        time.sleep(0.5)
    
    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

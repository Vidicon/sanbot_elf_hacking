import sys
import os
import time

from Common.sara_library import *


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()


    while True:
        touchsensors_head = robot.head.touch_sensors.get_all_values()
        # print("Touch sensors head:", [int(value) for value in touchsensors_head])

        touchsensors_body = robot.body.touch_sensors.get_all_values()
        # print("Touch sensors body:", [int(value) for value in touchsensors_body])   

        if (touchsensors_head[4]):
            print("Touch front right detected!")
        if (touchsensors_head[0]):
            print("Touch front left detected!")
        if (touchsensors_head[1]):   
            print("Touch rear left detected!")
        if (touchsensors_head[5]):   
            print("Touch rear right detected!")      
          
        if (touchsensors_body[0]):   
            print("Touch arm left detected!")      
        if (touchsensors_body[1]):   
            print("Touch arm right detected!")      

        time.sleep(0.5)
    
    time.sleep(1)
    robot.stop()

    return


if __name__ == "__main__":
    main()

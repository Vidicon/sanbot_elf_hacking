import time
from Common.sara_library_bridged import *

def main():
    robot = SaraRobot('saradev.local')

    robot.head.getversion()
    robot.body.getversion()

    print("Press CTRL + C to stop.")

    try:
        while True:
            distance_warning = robot.body.distancesensors.sensor_warning()
            distance_collision = robot.body.distancesensors.sensor_collision()
            cliff_warning = robot.body.distancesensors.sensor_cliffwarning()

            if distance_collision:
                print("Distance sensors: Collision ahead!")
                robot.body.distancesensors.print_values()
            else:
                if distance_warning:
                    print("Distance sensors: Getting close!")
                    robot.body.distancesensors.print_values()
                else:
                    print("Distance sensors: All clear!")
                    # robot.body.distancesensors.print_values()

            #------------------------------------------------------------------------
            # Safety measure. If cliff_warning == True, the programm will stop.
            #------------------------------------------------------------------------
            assert cliff_warning == False, "Cliff sensor triggered!"

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping")
        robot.stop()

    return


if __name__ == "__main__":
    main()

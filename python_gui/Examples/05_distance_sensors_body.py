import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot("COM10", "COM11", "/dev/ttyACM1", "/dev/ttyACM0", logging=False)

    time.sleep(1)

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

            #------------------------------------------------------------------------
            # Safety measure. If cliff_warning == True, the programm will stop.
            #------------------------------------------------------------------------
            assert cliff_warning == False, "Cliff sensor triggered!"

            # print(".")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping")
        robot.stop()

    return


if __name__ == "__main__":
    main()

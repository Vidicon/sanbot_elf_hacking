import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    print("Press CTRL + C to stop.")

    try:
        while True:
            distance_warning = robot.body.distancesensors.sensor_warning()
            distance_collision = robot.body.distancesensors.sensor_collision()

            if distance_collision:
                print("Distance sensors: Collision ahead!")
                robot.body.distancesensors.print_values()
            else:
                if distance_warning:
                    print("Distance sensors: Getting close!")
                    robot.body.distancesensors.print_values()

            print(".")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping")
        robot.stop()

    return


if __name__ == "__main__":
    main()

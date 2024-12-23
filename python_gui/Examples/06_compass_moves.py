import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    # Wait for first valid data
    time.sleep(1.0)

    # Read the angle
    angle = robot.body.compass.read_abs_angle()
    print(f"{angle:.1f} Deg")

    # Rotate to an absolute angle of 45 degree
    robot.body.compass.rotate_absolute(45, wait_for_finish=True)

    # Rotate a relative angle
    angle = robot.body.compass.read_abs_angle()
    print(f"{angle:.1f} Deg")
    robot.body.compass.rotate_absolute(angle + 90, wait_for_finish=True)

    robot.body.compass.rotate_absolute(-160, wait_for_finish=True)

    # time.sleep(10)

    robot.stop()
    return


if __name__ == "__main__":
    main()

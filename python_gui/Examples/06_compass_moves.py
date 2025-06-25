import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot(logging=False)

    robot.head.getversion()
    robot.body.getversion()

    # Wait for first valid data
    time.sleep(1.0)

    # Read the angle
    angle = robot.body.compass.read_abs_angle()
    robot.body.compass.print_values()

    # Rotate to an absolute angle of 45 degree
    robot.body.compass.rotate_absolute(45, wait_for_finish=True)

    # Rotate a relative angle
    angle = robot.body.compass.read_abs_angle()
    robot.body.compass.print_values()

    robot.body.compass.rotate_absolute(angle + 90, wait_for_finish=True)

    angle = robot.body.compass.read_abs_angle()
    robot.body.compass.print_values()

    robot.body.compass.rotate_absolute(200, wait_for_finish=True, rotation_tmo_threshold=20)

    angle = robot.body.compass.read_abs_angle()
    robot.body.compass.print_values()

    robot.stop()
    return


if __name__ == "__main__":
    main()

import time
from Common.sara_library import SaraRobot
from Common.colorled import ColorLed


def drive_normal(robot):
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=40, Rotation_Velocity=0)
    return


def drive_slow(robot):
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=20, Rotation_Velocity=0)
    return


def rotate_on_collision(robot, FL, FR, F, L, R):
    # Check where the collision is first

    angle = robot.body.compass.read_abs_angle()

    if F:
        if FL and not FR:
            print("Front + FL")
            robot.body.compass.rotate_absolute((angle + 60) % 360, wait_for_finish=True)

        if not FL and FR:
            print("Front + FR")
            robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)

        if FL and FR:
            print("Front + FL + FR")
            robot.body.compass.rotate_absolute((angle - 120) % 360, wait_for_finish=True)

        if not FL and not FR:
            print("Front")
            robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)

        drive_normal(robot)

    elif L:
        if FL:
            robot.body.compass.rotate_absolute((angle + 60) % 360, wait_for_finish=True)
            print("Left + FL")
        else:
            print("Left")
            robot.body.compass.rotate_absolute((angle + 45) % 360, wait_for_finish=True)

        drive_normal(robot)

    elif R:
        if FR:
            print("Right + FR")
            robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)
        else:
            print("Right")
            robot.body.compass.rotate_absolute((angle - 45) % 360, wait_for_finish=True)

        drive_normal(robot)

    elif FL:
        print("FL")
        robot.body.compass.rotate_absolute((angle + 45) % 360, wait_for_finish=True)
        drive_normal(robot)

    elif FR:
        print("FR")
        robot.body.compass.rotate_absolute((angle - 45) % 360, wait_for_finish=True)
        drive_normal(robot)

    else:
        print("????")
        time.sleep(10)

    # time.sleep(1)
    return


def update_leds(robot, FL, FR, F, L, R):

    robot.base.led.setcolor(color=ColorLed.Red, blink=ColorLed.LED_BLINK_FAST)

    return


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    print("Press CTRL + C to stop.")

    # Start drive forward until a warning is detected
    drive_normal(robot)

    try:
        while True:

            # distance_warning = robot.body.distancesensors.sensor_warning()
            # distance_collision = robot.body.distancesensors.sensor_collision()

            FL = robot.body.distancesensors.is_collision_frontleft()
            FR = robot.body.distancesensors.is_collision_frontright()
            F = robot.body.distancesensors.is_collision_front()
            L = robot.body.distancesensors.is_collision_left(threshold=15000)
            R = robot.body.distancesensors.is_collision_right(threshold=15000)

            if FL or FR or F or L or R:
                print("Distance sensors: Collision!")

                robot.base.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_FAST)

                robot.body.distancesensors.print_values()
                robot.base.move_stop()
                rotate_on_collision(robot, FL, FR, F, L, R)

            else:
                # all fine
                robot.base.led.setcolor(color=ColorLed.GREEN, blink=ColorLed.LED_ON)
                pass

            # print(".")
            time.sleep(0.25)

    except KeyboardInterrupt:
        print("Stopping")
        robot.base.move_stop()
        time.sleep(1)
        robot.stop()

    return


if __name__ == "__main__":
    main()

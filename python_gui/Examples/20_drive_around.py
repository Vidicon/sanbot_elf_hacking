import time
from Common.sara_library import SaraRobot
from Common.colorled import ColorLed
from Common.sara_common import RobotArmPositions

last_front_move = 0


def drive_normal(robot):
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=40, Rotation_Velocity=0)
    return


def drive_slow(robot):
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=20, Rotation_Velocity=0)
    return


def plot_sensors(robot, FL, FR, F, L, R):
    # Initialize a dynamic array of characters
    myarray = list("--------")

    if L:
        myarray[0] = "X"
        myarray[1] = "X"

    if FL:
        myarray[2] = "X"

    if F:
        myarray[3] = "X"
        myarray[4] = "X"

    if FR:
        myarray[5] = "X"

    if R:
        myarray[6] = "X"
        myarray[7] = "X"

    print(" ".join(myarray) + " ==> ", end="")
    return


def rotate_on_collision(robot, L, FL, F, FR, R):
    # Check where the collision is first
    global last_front_move

    angle = robot.body.compass.read_abs_angle()

    if F:
        if FL and not FR:
            print("F + FL")
            robot.body.compass.rotate_absolute((angle + 60) % 360, wait_for_finish=True)

        if not FL and FR:
            print("F + FR")
            robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)

        if FL and FR:
            print("F + FL + FR")

            if last_front_move == 0:
                robot.body.compass.rotate_absolute((angle - 120) % 360, wait_for_finish=True)
            else:
                robot.body.compass.rotate_absolute((angle + 120) % 360, wait_for_finish=True)

            last_front_move = (last_front_move + 1) % 2

        if not FL and not FR:
            print("F")

            if last_front_move == 0:
                robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)
            else:
                robot.body.compass.rotate_absolute((angle + 60) % 360, wait_for_finish=True)

            last_front_move = (last_front_move + 1) % 2

        drive_normal(robot)

    elif L:
        if FL:
            robot.body.compass.rotate_absolute((angle + 60) % 360, wait_for_finish=True)
            print("L + FL")
        else:
            print("L")
            robot.body.compass.rotate_absolute((angle + 45) % 360, wait_for_finish=True)

        drive_normal(robot)

    elif R:
        if FR:
            print("R + FR")
            robot.body.compass.rotate_absolute((angle - 60) % 360, wait_for_finish=True)
        else:
            print("R")
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
        print("Something very wrong")
        time.sleep(10)

    return


def update_leds(robot, L, FL, F, FR, R):
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
            L = robot.body.distancesensors.is_collision_left()
            FL = robot.body.distancesensors.is_collision_frontleft()
            F = robot.body.distancesensors.is_collision_front()
            FR = robot.body.distancesensors.is_collision_frontright()
            R = robot.body.distancesensors.is_collision_right()

            if L or FL or F or FR or R:
                # Distance sensors: Collision!
                robot.base.move_stop()
                robot.base.led.setcolor(color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_FAST)

                plot_sensors(robot, L, FL, F, FR, R)

                robot.left_arm.motor.move(position=RobotArmPositions.DOWN)
                robot.right_arm.motor.move(position=RobotArmPositions.DOWN)

                # robot.body.distancesensors.print_values()
                rotate_on_collision(robot, L, FL, F, FR, R)
            else:
                # all fine
                robot.base.led.setcolor(color=ColorLed.GREEN, blink=ColorLed.LED_ON)
                pass

            # Detect motion with the IR sensor

            # Play some music

            # Move some arms

            # Some colors

            # Move the head

            time.sleep(0.25)

    except KeyboardInterrupt:
        print("Stopping")
        robot.base.move_stop()
        time.sleep(1)
        robot.stop()

    return


if __name__ == "__main__":
    main()

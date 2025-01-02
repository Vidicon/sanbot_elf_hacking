import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()

    # Wait for first valid data
    time.sleep(1.0)

    # Some move examples.
    # +100 = max positive sepeed
    # -100 = max negative speed

    # Start with an absolute rotation angle
    robot.body.compass.rotate_absolute(abs_rotation_angle=160, wait_for_finish=True)

    # Do some XYR moves
    robot.base.move(Sideways_Velocity=50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(2.0)
    robot.base.move_stop()
    robot.base.brake(ApplyBrake=True)
    time.sleep(2.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=50, Rotation_Velocity=0)
    time.sleep(2.0)
    robot.base.move_stop()
    robot.base.brake(ApplyBrake=True)
    time.sleep(1.0)

    robot.base.move(Sideways_Velocity=-50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(2.0)
    robot.base.move_stop()
    robot.base.brake(ApplyBrake=True)
    time.sleep(2.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=-50, Rotation_Velocity=0)
    time.sleep(2.0)
    robot.base.move_stop()
    robot.base.brake(ApplyBrake=True)
    time.sleep(1.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=60)
    time.sleep(2.0)
    robot.base.move_stop()
    robot.base.brake(ApplyBrake=True)
    time.sleep(1.0)

    # Turn back to an absolute angle
    robot.body.compass.rotate_absolute(abs_rotation_angle=160, wait_for_finish=True)

    robot.base.brake(ApplyBrake=False)
    robot.stop()
    return


if __name__ == "__main__":
    main()

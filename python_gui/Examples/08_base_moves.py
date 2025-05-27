import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot(logging=False)

    # Wait for first valid data
    time.sleep(1.0)

    # Some move examples.
    # +100 = max positive sepeed
    # -100 = max negative speed

    # Start with an absolute rotation angle
    robot.body.compass.rotate_absolute(abs_rotation_angle=160, wait_for_finish=True)

    # ==================================================================================================
    # Moves have a safety timeout of 2 seconds.
    # To move for a longer time, keep sending the command.
    # In this case, send every 1 second.
    # ==================================================================================================
    # Do some XYR moves
    robot.base.move(Sideways_Velocity=50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move(Sideways_Velocity=50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move_stop()
    time.sleep(2.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=50, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=50, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move_stop()
    time.sleep(1.0)

    robot.base.move(Sideways_Velocity=-50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move(Sideways_Velocity=-50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move_stop()
    time.sleep(2.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=-50, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=-50, Rotation_Velocity=0)
    time.sleep(1.0)
    robot.base.move_stop()
    time.sleep(1.0)

    robot.base.move(Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=60)
    time.sleep(1.0)
    robot.base.move(Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=60)
    time.sleep(1.0)
    robot.base.move_stop()
    time.sleep(1.0)

    # Turn back to an absolute angle
    robot.body.compass.rotate_absolute(abs_rotation_angle=160, wait_for_finish=True)
    time.sleep(1.0)

    robot.base.brake(ApplyBrake=False)
    robot.stop()
    return


if __name__ == "__main__":
    main()

import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot(logging=False)

    # Wait for first valid data
    time.sleep(1.0)

    # ==================================================================================================
    # Moves have a safety timeout of 2 seconds.
    # To move for a longer time, keep sending the command.
    # In this case, send every 1 second.
    # ==================================================================================================
    # Do some XYR moves
    robot.base.move(Sideways_Velocity=50, Forward_Velocity=0, Rotation_Velocity=0)
    time.sleep(10.0)

    robot.base.brake(ApplyBrake=False)
    robot.stop()
    return


if __name__ == "__main__":
    main()

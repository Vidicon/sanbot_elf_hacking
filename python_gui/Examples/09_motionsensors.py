import time
from Common.sara_library import SaraRobot


def main():
    robot = SaraRobot(logging=False)

    time.sleep(1)

    print("Press CTRL + C to stop.")

    try:
        while True:
            motionsensors = robot.body.motionsensors.sensors
            print("Motion sensors:", [int(value) for value in motionsensors])
            
            if (motionsensors[0]):
                print("Motion front detected!")

            if (motionsensors[1]):
                print("Motion back detected!")
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping")
        robot.stop()

    return


if __name__ == "__main__":
    main()

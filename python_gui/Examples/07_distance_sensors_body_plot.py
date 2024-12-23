import matplotlib.pyplot as plt
import numpy as np
import time
import math
from Common.sara_library import SaraRobot
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arc
import addcopyfighandler

# Define as global otherwise the animation does not work.
ani = []


# Update function for the animation
def update_sensor_plot(frame, ax, robot):
    raw_values = robot.body.distancesensors.get_all_values() / 65536
    sensor_angles = robot.body.distancesensors.sensor_angles

    # Clear the axis and re-plot
    ax.clear()
    ax.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax.set_theta_direction(-1)  # Clockwise direction

    print(raw_values)
    print(sensor_angles)

    for i in range(8):
        start_angle = sensor_angles[i, 0] / 180 * math.pi
        stop_angle = sensor_angles[i, 1] / 180 * math.pi

        my_color = "lightgrey"
        if raw_values[i] < (25000.0 / 65536.0):
            my_color = "yellow"

        if raw_values[i] < (15000.0 / 65536.0):
            my_color = "red"

        # ax.fill_between([start_angle, stop_angle], [raw_values[i], raw_values[i]], color=my_color)

        ax.fill_between(
            x=[start_angle, stop_angle],
            y1=raw_values[i],
            y2=1,
            color=my_color,
        )

    print("+")


def main():
    global ani

    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(8, 6))
    ax.set_rlim(0, 1)
    ax.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax.set_theta_direction(-1)  # Clockwise direction

    robot = SaraRobot("COM2", "COM3", "/dev/ttyACM0", "/dev/ttyACM1")

    robot.getversion()
    time.sleep(1)

    # Set up the animation
    ani = FuncAnimation(
        fig, update_sensor_plot, fargs=(ax, robot), save_count=50, cache_frame_data=False, interval=500
    )  # Pass ax as an argument

    # Show the plot
    plt.show()


if __name__ == "__main__":
    main()

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
def update_sensor_plot(frame, ax1, ax2, robot):
    raw_values = robot.body.distance_sensors.get_all_values() / 65536
    sensor_angles_bottom = robot.body.distance_sensors.sensor_angles_bottom
    sensor_angles_mid = robot.body.distance_sensors.sensor_angles_mid

    # Clear the axis and re-plot
    ax1.clear()
    ax1.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax1.set_theta_direction(-1)  # Clockwise direction

    ax2.clear()
    ax2.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax2.set_theta_direction(-1)  # Clockwise direction

    # print(raw_values)

    for i in range(8):
        start_angle = sensor_angles_bottom[i, 0] / 180 * math.pi
        stop_angle = sensor_angles_bottom[i, 1] / 180 * math.pi

        my_color = "lightgrey"
        if raw_values[i] < (30000.0 / 65536.0):
            my_color = "yellow"

        if raw_values[i] < (20000.0 / 65536.0):
            my_color = "red"

        # ax.fill_between([start_angle, stop_angle], [raw_values[i], raw_values[i]], color=my_color)

        ax1.fill_between(
            x=[start_angle, stop_angle],
            y1=raw_values[i],
            y2=1,
            color=my_color,
        )

    for i in range(3):
        start_angle = sensor_angles_mid[i, 0] / 180 * math.pi
        stop_angle = sensor_angles_mid[i, 1] / 180 * math.pi

        my_color = "lightgrey"
        if raw_values[i + 8] < (30000.0 / 65536.0):
            my_color = "yellow"

        if raw_values[i + 8] < (20000.0 / 65536.0):
            my_color = "red"

        # ax.fill_between([start_angle, stop_angle], [raw_values[i], raw_values[i]], color=my_color)

        ax2.fill_between(
            x=[start_angle, stop_angle],
            y1=raw_values[i + 8],
            y2=1,
            color=my_color,
        )

    print("+")


def main():
    global ani

    # fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(8, 6))
    fig, (ax1, ax2) = plt.subplots(1, 2, subplot_kw={"projection": "polar"}, figsize=(12, 6))

    # Display the plot non-blockingly
    plt.ion()  # Enable interactive mode
    plt.show()

    ax1.set_rlim(0, 1)
    ax1.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax1.set_theta_direction(-1)  # Clockwise direction

    ax2.set_rlim(0, 1)
    ax2.set_theta_offset(np.pi / 2)  # Rotate plot so 0 is at the top
    ax2.set_theta_direction(-1)  # Clockwise direction

    robot = SaraRobot(logging=False)

    time.sleep(1)

    # Set up the animation
    # ani = FuncAnimation(
    #     fig,
    #     update_sensor_plot,
    #     fargs=(ax1, ax2, robot),
    #     save_count=50,
    #     cache_frame_data=True,
    #     interval=250,
    # )

    old_counter = robot.body.distance_sensors.get_rx_counter()

    try:
        while True:
            update_sensor_plot(fig, ax1, ax2, robot)

            fig.canvas.draw_idle()  # Redraw the canvas
            fig.canvas.flush_events()  # Flush GUI events to update the figure

            # Wait for new distance sensor data
            try:
                while old_counter == robot.body.distance_sensors.get_rx_counter():
                    time.sleep(0.1)

                old_counter = robot.body.distance_sensors.get_rx_counter()
            except KeyboardInterrupt:
                return

    except KeyboardInterrupt:
        print("Stopping")
        robot.stop()


if __name__ == "__main__":
    main()

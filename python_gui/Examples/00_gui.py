import tkinter as tk
from tkinter import scrolledtext
from tkinter import font
import time

from Common.sara_library import *
from Common.sara_common import *

DEBUG = True


class SaraGUI:
    def __init__(self, robot):
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("Sara Development User Interface - BODY")

        # Set up the window close protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Create the main frame
        self.frame = tk.Frame(self.root)
        self.frame.pack(padx=20, pady=20)

        # --------------------------------------------------------------------------------------
        # General settings
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 0
        button_width = 15

        self.button_version_head = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Version Head",
            command=lambda: self.robot.head.getversion(),
        )
        self.button_version_head.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_version_body = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Version Body",
            command=lambda: self.robot.body.getversion(),
        )
        self.button_version_body.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1
        self.button_stop = tk.Button(
            self.frame,
            height=4,  # Adjust the height to span multiple rows
            width=button_width,
            text="STOP",
            # bg="lightcoral",  # Set background color to light red
        )
        self.button_stop.grid(
            row=row_count, column=col_count, rowspan=4, sticky="w"
        )  # Use rowspan to span multiple rows

        # --------------------------------------------------------------------------------------
        # Left arm
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 1

        self.button_LA_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left White",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.WHITE, blink=ColorLed.LED_ON
            ),
        )
        self.button_LA_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Red",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.RED, blink=ColorLed.LED_ON
            ),
        )
        self.button_LA_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Green",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.GREEN, blink=ColorLed.LED_ON
            ),
        )
        self.button_LA_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Blue",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.BLUE, blink=ColorLed.LED_ON
            ),
        )
        self.button_LA_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Blue_Blink = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Blue Blink",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.BLUE, blink=ColorLed.LED_BLINK_VERYFAST
            ),
        )
        self.button_LA_Blue_Blink.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LA_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Off",
            command=lambda: self.robot.left_arm.led.setcolor(
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF
            ),
        )
        self.button_LA_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Right arm
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count = 2

        self.button_RA_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right White",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.WHITE, blink=ColorLed.LED_ON
            ),
        )
        self.button_RA_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Red",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.RED, blink=ColorLed.LED_ON
            ),
        )
        self.button_RA_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Green",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.GREEN, blink=ColorLed.LED_ON
            ),
        )
        self.button_RA_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Blue",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.BLUE, blink=ColorLed.LED_ON
            ),
        )
        self.button_RA_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Green_Blink = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Green Blink",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.GREEN, blink=ColorLed.LED_BLINK_FAST
            ),
        )
        self.button_RA_Green_Blink.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RA_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Off",
            command=lambda: self.robot.right_arm.led.setcolor(
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF
            ),
        )
        self.button_RA_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Base
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count += 1

        self.button_BASE_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base White",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.WHITE, blink=ColorLed.LED_ON
            ),
        )
        self.button_BASE_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Red",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.RED, blink=ColorLed.LED_ON
            ),
        )
        self.button_BASE_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Green",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.GREEN, blink=ColorLed.LED_ON
            ),
        )
        self.button_BASE_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Blue",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.BLUE, blink=ColorLed.LED_ON
            ),
        )
        self.button_BASE_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Blink_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Blink Red",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.RED, blink=ColorLed.LED_BLINK_FAST
            ),
        )
        self.button_BASE_Blink_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_BASE_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Base Off",
            command=lambda: self.robot.base.led.setcolor(
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF
            ),
        )
        self.button_BASE_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Head
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count += 1

        self.button_HEAD_Red = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Left Red",
            command=lambda: self.robot.head.left_led.setcolor(
                color=ColorLed.RED, blink=ColorLed.LED_ON
            ),
        )
        self.button_HEAD_Red.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_White = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Left White",
            command=lambda: self.robot.head.left_led.setcolor(
                color=ColorLed.WHITE, blink=ColorLed.LED_ON
            ),
        )
        self.button_HEAD_White.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Right_Green = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Green",
            command=lambda: self.robot.head.right_led.setcolor(
                color=ColorLed.GREEN, blink=ColorLed.LED_ON
            ),
        )
        self.button_HEAD_Right_Green.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Right_Blue = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Blue",
            command=lambda: self.robot.head.right_led.setcolor(
                color=ColorLed.BLUE, blink=ColorLed.LED_ON
            ),
        )
        self.button_HEAD_Right_Blue.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Left_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Left Off",
            command=lambda: self.robot.head.left_led.setcolor(
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF
            ),
        )
        self.button_HEAD_Left_Off.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_HEAD_Right_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right Off",
            command=lambda: self.robot.head.right_led.setcolor(
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF
            ),
        )
        self.button_HEAD_Right_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Arms
        # --------------------------------------------------------------------------------------
        row_count = 0
        col_count += 1

        self.button_LeftArm_Up = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Arm Up",
            command=lambda: robot.left_arm.motor.move(position=RobotArmPositions.UP),
        )
        self.button_LeftArm_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LeftArm_Forward = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Arm Forward",
            command=lambda: robot.left_arm.motor.move(
                position=RobotArmPositions.FORWARD
            ),
        )
        self.button_LeftArm_Forward.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LeftArm_Down = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Arm Down",
            command=lambda: robot.left_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_LeftArm_Down.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1
        row_count += 1

        self.button_RightArm_Up = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Arm Up",
            command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.UP),
        )
        self.button_RightArm_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RightArm_Forward = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Arm Forward",
            command=lambda: robot.right_arm.motor.move(
                position=RobotArmPositions.FORWARD
            ),
        )
        self.button_RightArm_Forward.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RightArm_Down = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Arm Down",
            command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_RightArm_Down.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Head buttons sensors
        # --------------------------------------------------------------------------------------
        row_count += 1
        row_count += 1

        self.button_Head_Up = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Up",
            # command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_Head_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Down = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Down",
            # command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_Head_Down.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Left = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Left",
            # command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_Head_Left.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Right = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Right",
            # command=lambda: robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_Head_Right.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Distance sensors
        # --------------------------------------------------------------------------------------
        col_count += 1
        row_count = 0

        self.distance_sensor_labels = []
        for i in range(12):
            label = tk.Button(
                self.frame, height=1, width=button_width, text=f"Distance {i+1}"
            )

            label.grid(row=row_count, column=col_count, sticky="w")
            self.distance_sensor_labels.append(label)
            row_count += 1

        # --------------------------------------------------------------------------------------
        # Distance sensors
        # --------------------------------------------------------------------------------------
        col_count += 1
        row_count = 0

        self.motion_sensor_labels = []
        for i in range(2):
            label = tk.Button(
                self.frame, height=1, width=button_width, text=f"Motion {i+1}"
            )

            label.grid(row=row_count, column=col_count, sticky="w")
            self.motion_sensor_labels.append(label)
            row_count += 1

        # --------------------------------------------------------------------------------------
        # Battery
        # --------------------------------------------------------------------------------------
        row_count += 1

        self.battery_labels = []
        for i in range(3):
            label = tk.Button(
                self.frame, height=1, width=button_width, text=f"Battery {i+1}"
            )

            label.grid(row=row_count, column=col_count, sticky="w")
            self.battery_labels.append(label)
            row_count += 1
        # --------------------------------------------------------------------------------------
        # Compass
        # --------------------------------------------------------------------------------------
        row_count += 1

        self.compass_labels = []
        for i in range(1):
            label = tk.Button(
                self.frame, height=1, width=button_width, text=f"Compass {i+1}"
            )

            label.grid(row=row_count, column=col_count, sticky="w")
            self.compass_labels.append(label)
            row_count += 1

        # --------------------------------------------------------------------------------------
        # streaming data area
        # --------------------------------------------------------------------------------------
        self.stream_area = scrolledtext.ScrolledText(
            self.frame, width=150, height=10, wrap="word"
        )
        self.stream_area.grid(row=20, column=0, columnspan=10, sticky="w")

    def on_closing(self):
        time.sleep(1)
        self.root.destroy()  # This will close the window

    def run(self):
        self.root.mainloop()


def main():
    robot = SaraRobot(logging=True)
    gui = SaraGUI(robot)
    gui.run()


if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import scrolledtext
from tkinter import font
import time
import math

from Common.sara_library import *
from Common.sara_common import *

DEBUG = True


class SaraGUI:
    def __init__(self, robot):
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("Sara User Interface")

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
            command=lambda: self.robot.noodstop()
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
        # Eyes
        # --------------------------------------------------------------------------------------
        row_count += 1
        row_count += 1

        self.button_Eyes0 = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Eyes 0",
            command=lambda: robot.head.eyes.set_eyes(left_eye=0, right_eye=0
            ),
        )
        self.button_Eyes0.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Eyes1 = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Eyes 1",
            command=lambda: robot.head.eyes.set_eyes(left_eye=1, right_eye=1
            ),
        )
        self.button_Eyes1.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Eyes2 = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Eyes 2",
            command=lambda: robot.head.eyes.set_eyes(left_eye=2, right_eye=2
            ),
        )
        self.button_Eyes2.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Eyes3 = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Eyes 3",
            command=lambda: robot.head.eyes.set_eyes(left_eye=3, right_eye=3
            ),
        )
        self.button_Eyes3.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Eyes4 = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Eyes 4",
            command=lambda: robot.head.eyes.set_eyes(left_eye=4, right_eye=4
            ),
        )
        self.button_Eyes4.grid(row=row_count, column=col_count, sticky="w")

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
                color=ColorLed.NOCOLOR, blink=ColorLed.LED_OFF)
        )
        self.button_RA_Off.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Head lamp
        # --------------------------------------------------------------------------------------
        row_count += 1
        row_count += 1

        self.button_Lamp_On = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Lamp On",
            command=lambda: robot.head.lamp.set_lamp(lamp_on=True)
        )
        self.button_Lamp_On.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Lamp_Off = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Lamp Off",
            command=lambda: robot.head.lamp.set_lamp(lamp_on=False)
        )
        self.button_Lamp_Off.grid(row=row_count, column=col_count, sticky="w")

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
        # Touch sensors
        # --------------------------------------------------------------------------------------
        row_count += 1
        row_count += 1

        self.button_TOUCH_LFront = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Left Front",
        )
        self.button_TOUCH_LFront.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_TOUCH_LRear = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Left Rear",
        )
        self.button_TOUCH_LRear.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_TOUCH_RFront = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Right Front",
        )
        self.button_TOUCH_RFront.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_TOUCH_RRear = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Right Rear",
        )
        self.button_TOUCH_RRear.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_TOUCH_Left_Arm = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Left Arm",
        )
        self.button_TOUCH_Left_Arm.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_TOUCH_Right_Arm = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Touch Right Arm",
        )
        self.button_TOUCH_Right_Arm.grid(row=row_count, column=col_count, sticky="w")

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
            command=lambda: self.robot.left_arm.motor.move(position=RobotArmPositions.UP),
        )
        self.button_LeftArm_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_LeftArm_Forward = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Left Arm Forward",
            command=lambda: self.robot.left_arm.motor.move(
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
            command=lambda: self.robot.left_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_LeftArm_Down.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1
        row_count += 1

        self.button_RightArm_Up = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Arm Up",
            command=lambda: self.robot.right_arm.motor.move(position=RobotArmPositions.UP),
        )
        self.button_RightArm_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_RightArm_Forward = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Right Arm Forward",
            command=lambda: self.robot.right_arm.motor.move(
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
            command=lambda: self.robot.right_arm.motor.move(position=RobotArmPositions.DOWN),
        )
        self.button_RightArm_Down.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Head moves
        # --------------------------------------------------------------------------------------
        row_count += 1
        row_count += 1

        self.button_Head_Up = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Up",
            command=lambda: self.robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_UP),
        )
        self.button_Head_Up.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Mid = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Tilt Mid",
            command=lambda: self.robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_MID),
        )
        self.button_Head_Mid.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Down = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Tilt Down",
            command=lambda: self.robot.head.tilt_motor.move(position=RobotHeadPositions.TILT_DOWN),
        )
        self.button_Head_Down.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Left = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Pan Left",
            command=lambda: self.robot.head.pan_motor.move(position=RobotHeadPositions.PAN_LEFT),
        )
        self.button_Head_Left.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Mid = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Pan Mid",
            command=lambda: self.robot.head.pan_motor.move(position=RobotHeadPositions.PAN_MID),
        )
        self.button_Head_Mid.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Head_Right = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Head Pan Right",
            command=lambda: self.robot.head.pan_motor.move(position=RobotHeadPositions.PAN_RIGHT),
        )
        self.button_Head_Right.grid(row=row_count, column=col_count, sticky="w")

        # --------------------------------------------------------------------------------------
        # Distance sensors
        # --------------------------------------------------------------------------------------
        col_count += 1
        row_count = 0

        self.distance_sensor_labels = []
        for i in range(13):
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
        labels = ["Motion Front", "Motion Back"]

        for i in range(2):
            label = tk.Button(
                self.frame, height=1, width=button_width, text=labels[i]
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
        # Compass rotations
        # --------------------------------------------------------------------------------------
        row_count += 1

        self.button_Rotate_North = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="North",
            command=lambda: self.robot.body.compass.rotate_absolute(0, wait_for_finish=False),
        )
        self.button_Rotate_North.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Rotate_East = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="East",
            command=lambda: self.robot.body.compass.rotate_absolute(90, wait_for_finish=False),
        )
        self.button_Rotate_East.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Rotate_South = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="South",
            command=lambda: self.robot.body.compass.rotate_absolute(180, wait_for_finish=False),
        )
        self.button_Rotate_South.grid(row=row_count, column=col_count, sticky="w")
        
        row_count += 1

        self.button_Rotate_West = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="West",
            command=lambda: self.robot.body.compass.rotate_absolute(270, wait_for_finish=False),
        )
        self.button_Rotate_West.grid(row=row_count, column=col_count, sticky="w")
        
        # --------------------------------------------------------------------------------------
        # Canvas for the rotating arrow
        # --------------------------------------------------------------------------------------
        col_count += 1
        row_count = 0

        canvas_width = 100
        canvas_height = 100
        self.canvas = tk.Canvas(self.frame, width=canvas_width, height=canvas_height, bg="white")
        self.canvas.grid(row=row_count, rowspan=4, column=col_count, sticky="nsew", padx=10, pady=10)  # Center the canvas in the column

        self.arrow_center = (canvas_width // 2, canvas_height // 2)

        row_count = 4
        self.compass_button = tk.Button(self.frame, height=1, width=button_width, text="inf")
        self.compass_button.grid(row=row_count, column=col_count, sticky="w")

        # # --------------------------------------------------------------------------------------
        # # streaming data area
        # # --------------------------------------------------------------------------------------
        # self.stream_area = scrolledtext.ScrolledText(
        #     self.frame, width=150, height=10, wrap="word"
        # )
        # self.stream_area.grid(row=20, column=0, columnspan=10, sticky="w")

        # --------------------------------------------------------------------------------------
        # Moves
        # --------------------------------------------------------------------------------------
        row_count += 3

        self.button_Move_BACK = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Move Forward",
            command=lambda: robot.base.move(Sideways_Velocity=0, Forward_Velocity=50, Rotation_Velocity=0),
        )
        self.button_Move_BACK.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Move_BACK = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Move Backward",
            command=lambda: robot.base.move(Sideways_Velocity=0, Forward_Velocity=-50, Rotation_Velocity=0),
        )
        self.button_Move_BACK.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Move_LEFT = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Move Left",
            command=lambda: robot.base.move(Sideways_Velocity=-50, Forward_Velocity=0, Rotation_Velocity=0),
        )
        self.button_Move_LEFT.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Move_RIGHT = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Move Right",
            command=lambda: robot.base.move(Sideways_Velocity=50, Forward_Velocity=0, Rotation_Velocity=0),
        )
        self.button_Move_RIGHT.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Rotate_LEFT = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Rotate Left",
            command=lambda: robot.base.move(Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=50),
        )
        self.button_Rotate_LEFT.grid(row=row_count, column=col_count, sticky="w")

        row_count += 1

        self.button_Rotate_RIGHT = tk.Button(
            self.frame,
            height=1,
            width=button_width,
            text="Rotate Right",
            command=lambda: robot.base.move(Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=-50),
        )
        self.button_Rotate_RIGHT.grid(row=row_count, column=col_count, sticky="w")

    def on_closing(self):
        time.sleep(1)
        self.root.destroy()  # This will close the window

    def run(self):
        self.root.mainloop()

    def distance_sensors_callback(self):
        distances = self.robot.body.distance_sensors.get_all_values()
        
        for i in range(11):
            self.SetDistanceButtonBGColor(self.distance_sensor_labels[i], distances[i])
            self.distance_sensor_labels[i].config(text=f"{distances[i]:.0f}")

        # cliff sensors
        i = 11
        self.distance_sensor_labels[i].config(text=f"{distances[i]:.0f}")
        if distances[i] > 25000:
           self.distance_sensor_labels[i].config(bg="red")
        else:
            self.distance_sensor_labels[i].config(bg="lime")            

        i = 12
        self.distance_sensor_labels[i].config(text=f"{distances[i]:.0f}")
        if distances[i] > 25000:
           self.distance_sensor_labels[i].config(bg="red")
        else:
            self.distance_sensor_labels[i].config(bg="lime")            

    def SetDistanceButtonBGColor(self, button, distance):
        button.config(text=str(distance))

        if distance < 15000:
            button.config(bg="red")
        elif distance < 30000:
            button.config(bg="yellow")
        else:
            button.config(bg="lime")            

    def motion_sensors_callback(self):
        motions = self.robot.body.motion_sensors.get_all_values()
        for i in range(2):
            if motions[i] > 0:
                self.motion_sensor_labels[i].config(bg="lime")
            else:
                self.motion_sensor_labels[i].config(bg=self.frame.cget("bg"))

    def battery_callback(self):
        battery_state = self.robot.body.battery.get_batterystate()

        if battery_state == Battery.EMPTY:
            self.battery_labels[0].config(bg="red", text="Empty")

        if battery_state == Battery.ERROR:
            self.battery_labels[0].config(bg="red", text="Error")

        if battery_state == Battery.DISCHARGE:
            self.battery_labels[0].config(bg="yellow", text="Discharging")

        if battery_state == Battery.CHARGE:
            self.battery_labels[0].config(bg="lime", text="Charging")


        battery_voltage = self.robot.body.battery.get_voltage()
        self.battery_labels[1].config(text=f"{battery_voltage:.0f} mV")

        battery_current = self.robot.body.battery.get_current()
        self.battery_labels[2].config(text=f"{battery_current:.0f} mA")

        return
    
    def compass_callback(self): 
        angle_degrees = self.robot.body.compass.read_abs_angle()   

        # print(f"Compass: {angle_degrees:.0f} Degree") 
        self.compass_button.config(text=f"{angle_degrees:.0f} Deg")

        self.draw_arrow(angle_degrees + 90)
        return
    
    # Draw the rotating arrow
    def draw_arrow(self, angle):
        self.canvas.delete("arrow")

        x, y = (63, 50)
        radians = math.radians(angle)
        x_end = x - 30 * math.cos(radians)
        y_end = y - 30 * math.sin(radians)  # Negative because canvas y-coordinates increase downwards
        self.canvas.create_line(
            x,
            y,
            x_end,
            y_end,
            arrow=tk.LAST,
            fill="blue",
            width=2,
            tags="arrow",
        )
        return
    

    def touch_sensors_head_callback(self):
        touchsensors = self.robot.head.touch_sensors.get_all_values()

        if touchsensors[0] > 0:
            self.button_TOUCH_LFront.config(bg="lime")
        else:
            self.button_TOUCH_LFront.config(bg=self.frame.cget("bg"))            

        if touchsensors[1] > 0:
            self.button_TOUCH_LRear.config(bg="lime")
        else:
            self.button_TOUCH_LRear.config(bg=self.frame.cget("bg"))            

        if touchsensors[4] > 0:
            self.button_TOUCH_RFront.config(bg="lime")
        else:
            self.button_TOUCH_RFront.config(bg=self.frame.cget("bg"))            

        if touchsensors[5] > 0:
            self.button_TOUCH_RRear.config(bg="lime")
        else:
            self.button_TOUCH_RRear.config(bg=self.frame.cget("bg"))            

        return

    def touch_sensors_body_callback(self):
        touchsensors = self.robot.body.touch_sensors.get_all_values()

        if touchsensors[0] > 0:
            self.button_TOUCH_Left_Arm.config(bg="lime")
        else:
            self.button_TOUCH_Left_Arm.config(bg=self.frame.cget("bg"))            

        if touchsensors[1] > 0:
            self.button_TOUCH_Right_Arm.config(bg="lime")
        else:
            self.button_TOUCH_Right_Arm.config(bg=self.frame.cget("bg"))            

        return

def main():
    robot = SaraRobot(logging=False)
    gui = SaraGUI(robot)

    robot.body.distance_sensors.set_callback(gui.distance_sensors_callback)
    robot.body.motion_sensors.set_callback(gui.motion_sensors_callback)
    robot.body.battery.set_callback(gui.battery_callback)
    robot.body.compass.set_callback(gui.compass_callback)
    robot.head.touch_sensors.set_callback(gui.touch_sensors_head_callback)
    robot.body.touch_sensors.set_callback(gui.touch_sensors_body_callback)  

    gui.run()


if __name__ == "__main__":
    main()

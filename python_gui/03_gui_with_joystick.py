import tkinter as tk
from tkinter import scrolledtext
from tkinter import font

import time
import pygame

from ModManager import ModManager
import numpy as np
import platform
import math

RESP_BIT = 0x80

CMD_VERSION = 0x01
CMD_LA_COLOR = 0x10
CMD_RA_COLOR = 0x11
CMD_BASE_COLOR = 0x12
CMD_BA_COLOR = 0x13

CMD_GET_ENCODERS = 0x20
CMD_GET_MOTIONSENSORS = 0x21
CMD_GET_DISTANCESENSORS = 0x22
CMD_GET_COMPASS = 0x23

CMD_LA_MOVE = 0x30
CMD_RA_MOVE = 0x31
CMD_BASE_MOVE = 0x32


CMD_RED = 1
CMD_GREEN = 2
CMD_BLUE = 3
CMD_WHITE = 4
CMD_ALL = 5

CMD_LED_NONE = 0
CMD_LED_OFF = 1
CMD_LED_ON = 2
CMD_LED_BLINK_OFF = 3
CMD_LED_BLINK_SLOW = 4
CMD_LED_BLINK_FAST = 5
CMD_LED_BLINK_VERYFAST = 6

axis0 = 0
axis1 = 0
axis2 = 0
axis3 = 0
axis4 = 0
axis5 = 0

button4_toggle = 0
button5_toggle = 0

button_version = []

button_debug1 = []
button_debug2 = []

button_Motion_Front = []
button_Motion_Back = []

distance1_Front = []
distance2_Front = []
distance3_Front = []
distance4_Front = []
distance5_Front = []
distance6_Front = []
distance7_Front = []
distance8_Front = []

compass_button = []

canvas = []
arrow_angle = 0


def SetDistanceButtonBGColor(button, distance):
    button.config(text=str(distance))

    if distance < 15000:
        button.config(bg="red")
    elif distance < 30000:
        button.config(bg="yellow")
    else:
        button.config(bg="lime")


# Function to calculate the angle
def calculate_angle(x, y):
    # atan2(y, x) returns the angle in radians
    angle_radians = math.atan2(y, x)
    # Convert radians to degrees (optional)
    angle_degrees = math.degrees(angle_radians)

    #  convert to compass angles

    compass_degree = -1 * angle_degrees

    if compass_degree < 0:
        compass_degree += 360

    return compass_degree, angle_degrees


# ===============================================================================
# Define the receive callback function
# ===============================================================================
def my_receive_callback(data, stream_area):
    # hex_values = " ".join([format(x, "02X") for x in data])
    # print("< " + hex_values)

    # Decode
    response = data[1]

    if response == (CMD_VERSION | RESP_BIT):

        string_from_bytearray = data[3:-2].decode("utf-8")
        stream_area.insert(tk.END, "< " + string_from_bytearray + "\n")
        stream_area.yview_moveto(1)  # Scrolling to the bottom

    if response == (CMD_GET_ENCODERS | RESP_BIT):
        new_byte_array = data[3:-2]
        int16_array_5 = np.frombuffer(new_byte_array, dtype=">i2")

        stream_area.insert(
            tk.END, "< Encoders   : " + str(int16_array_5) + "\n"
        )
        stream_area.yview_moveto(1)  # Scrolling to the bottom

    if response == (CMD_GET_COMPASS | RESP_BIT):
        new_byte_array = data[3:-2]

        try:
            int16_array_5 = np.frombuffer(new_byte_array, dtype=">i2")

            stream_area.insert(
                tk.END, "< Compass    : " + str(int16_array_5) + "\n"
            )
            stream_area.yview_moveto(1)  # Scrolling to the bottom

            compass_angle, angle_degrees = calculate_angle(
                int16_array_5[0], int16_array_5[1]
            )

            draw_arrow(angle_degrees + 90)

            compass_button.config(text=f"{compass_angle:.0f} Deg")
        except:
            print("Compass bytes error")

    if response == (CMD_GET_MOTIONSENSORS | RESP_BIT):
        new_byte_array = data[3:-2]

        try:
            int8_array = np.frombuffer(new_byte_array, dtype=">i1")

            stream_area.insert(
                tk.END, "< Motion     : " + str(int8_array) + "\n"
            )
            stream_area.yview_moveto(1)  # Scrolling to the bottom

            if int8_array[0] == 1:
                button_Motion_Front.config(bg="yellow")
            else:
                button_Motion_Front.config(bg=default_bg)

            if int8_array[1] == 1:
                button_Motion_Back.config(bg="yellow")
            else:
                button_Motion_Back.config(bg=default_bg)
        except:
            print("Motion sensors bytes error")

    if response == (CMD_GET_DISTANCESENSORS | RESP_BIT):
        new_byte_array = data[3:-2]
        uint8_array = np.frombuffer(new_byte_array, dtype=">u1")

        stream_area.insert(tk.END, "< Distance   : " + str(uint8_array) + "\n")
        stream_area.yview_moveto(1)  # Scrolling to the bottom

        # --------------------------------------------------------------------------
        combined1_int = int.from_bytes(
            new_byte_array[0:2], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance1_Front, combined1_int)

        # --------------------------------------------------------------------------
        combined2_int = int.from_bytes(
            new_byte_array[2:4], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance2_Front, combined2_int)

        # --------------------------------------------------------------------------
        combined3_int = int.from_bytes(
            new_byte_array[4:6], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance3_Front, combined3_int)

        # --------------------------------------------------------------------------
        combined4_int = int.from_bytes(
            new_byte_array[6:8], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance4_Front, combined4_int)

        # --------------------------------------------------------------------------
        combined5_int = int.from_bytes(
            new_byte_array[8:10], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance5_Front, combined5_int)

        # --------------------------------------------------------------------------
        combined6_int = int.from_bytes(
            new_byte_array[10:12], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance6_Front, combined6_int)

        # --------------------------------------------------------------------------
        combined7_int = int.from_bytes(
            new_byte_array[12:14], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance7_Front, combined7_int)

        # --------------------------------------------------------------------------
        combined8_int = int.from_bytes(
            new_byte_array[14:16], byteorder="big"
        )  # 'big' for big-endian, 'little' for little-endian
        SetDistanceButtonBGColor(distance8_Front, combined8_int)


def createCommand(mod_manager, InputCommmand, Parameters):

    if InputCommmand == CMD_VERSION:
        mod_manager.cmd_Generic(CMD_VERSION, 0, 0)

    return


def createLedCommand(mod_manager, Parameters):

    print(Parameters)
    mod_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

    return


def createMoveCommand(mod_manager, Parameters):
    high = (int(Parameters[1]) >> 8) & 0xFF
    low = int(Parameters[1]) & 0xFF

    mod_manager.cmd_Generic(Parameters[0], 2, np.array([high, low]))

    return


def createBaseCommand(mod_manager, Parameters):
    mod_manager.cmd_Generic(Parameters[0], 3, np.array(Parameters[1:]))

    return


def createDoubleMoveCommand(mod_manager, Parameters):
    high = (int(Parameters[1]) >> 8) & 0xFF
    low = int(Parameters[1]) & 0xFF

    mod_manager.cmd_Generic(Parameters[0], 2, np.array([high, low]))

    high2 = (int(Parameters[1 + 2]) >> 8) & 0xFF
    low2 = int(Parameters[1 + 2]) & 0xFF

    mod_manager.cmd_Generic(Parameters[0 + 2], 2, np.array([high2, low2]))

    return


# ===============================================================================
# Clear logging
# ===============================================================================
def clear_logging():
    # text_area.delete('1.0', 'end')  # Deletes from start to end
    stream_area.delete("1.0", "end")  # Deletes from start to end
    return


def on_closing():
    # Ask the user for confirmation before closing the window
    mod_manager.close_port()
    time.sleep(1)
    root.destroy()  # This will close the window


# ===============================================================================
# Show a basic GUI
# ===============================================================================
def show_gui(mod_manager):
    root = tk.Tk()
    root.title("Sara Developement User Interface")

    # Set up the window close protocol
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # root.tk.call('tk', 'scaling', 0.5)

    frame = tk.Frame(root)
    frame.pack(padx=20, pady=20)  # Adding padding for demonstration

    # --------------------------------------------------------------------------------------
    # Left arm
    # --------------------------------------------------------------------------------------
    global button_debug1
    button_debug1 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Debug 1",
        command=lambda t=np.array(
            [CMD_LA_MOVE, -500, CMD_RA_MOVE, 500]
        ): createDoubleMoveCommand(mod_manager, t),
    )
    button_debug1.grid(row=0, column=8, sticky="w")

    global button_debug2
    button_debug2 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Debug 2",
        command=lambda t=np.array(
            [CMD_LA_MOVE, -150, CMD_RA_MOVE, 150]
        ): createDoubleMoveCommand(mod_manager, t),
    )
    button_debug2.grid(row=1, column=8, sticky="w")

    button_LA_White = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left White",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_WHITE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_White.grid(row=0, column=0, sticky="w")

    button_LA_Red = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Red",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_RED, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_Red.grid(row=1, column=0, sticky="w")

    button_LA_Green = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Green",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_GREEN, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_Green.grid(row=2, column=0, sticky="w")

    button_LA_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Blue",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_BLUE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_Blue.grid(row=3, column=0, sticky="w")

    button_LA_Off = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Off",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_Off.grid(row=4, column=0, sticky="w")

    button_LA_Blue_Blink = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Blue Blink",
        command=lambda t=np.array(
            [CMD_LA_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]
        ): createLedCommand(mod_manager, t),
    )
    button_LA_Blue_Blink.grid(row=5, column=0, sticky="w")

    global button_version
    button_version = tk.Button(
        frame,
        height=1,
        width=10,
        text="Get Version",
        command=lambda t=[0]: createCommand(mod_manager, CMD_VERSION, t),
    )
    button_version.grid(row=8, column=0, sticky="w")

    global default_bg
    default_bg = button_version.cget("bg")

    # --------------------------------------------------------------------------------------
    # Right arm
    # --------------------------------------------------------------------------------------
    button_RA_White = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right White",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_WHITE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_White.grid(row=0, column=1, sticky="w")

    button_RA_Red = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Red",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_RED, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_Red.grid(row=1, column=1, sticky="w")

    button_RA_Green = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Green",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_GREEN, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_Green.grid(row=2, column=1, sticky="w")

    button_RA_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Blue",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_BLUE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_Blue.grid(row=3, column=1, sticky="w")

    button_RA_Off = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Off",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_Off.grid(row=4, column=1, sticky="w")

    button_RA_Blue_Blink = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Blue Blink",
        command=lambda t=np.array(
            [CMD_RA_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]
        ): createLedCommand(mod_manager, t),
    )
    button_RA_Blue_Blink.grid(row=5, column=1, sticky="w")

    # --------------------------------------------------------------------------------------
    # Right arm
    # --------------------------------------------------------------------------------------
    button_Base_Off = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base White",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_WHITE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Off.grid(row=0, column=2, sticky="w")

    button_Base_Red = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base Red",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_RED, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Red.grid(row=1, column=2, sticky="w")

    button_Base_Green = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base Green",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_GREEN, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Green.grid(row=2, column=2, sticky="w")

    button_Base_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base Blue",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_BLUE, CMD_LED_ON]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Blue.grid(row=3, column=2, sticky="w")

    button_Base_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base Off",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_ALL, CMD_LED_OFF]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Blue.grid(row=4, column=2, sticky="w")

    button_Base_Blue_Blink = tk.Button(
        frame,
        height=1,
        width=10,
        text="Base Blue Blink",
        command=lambda t=np.array(
            [CMD_BASE_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]
        ): createLedCommand(mod_manager, t),
    )
    button_Base_Blue_Blink.grid(row=5, column=2, sticky="w")

    # --------------------------------------------------------------------------------------
    # Move Left arm
    # --------------------------------------------------------------------------------------
    button_LA_Move_1 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Move 1",
        command=lambda t=np.array([CMD_LA_MOVE, -500]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_LA_Move_1.grid(row=0, column=3, sticky="w")

    button_LA_Move_2 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Move 2",
        command=lambda t=np.array([CMD_LA_MOVE, -350]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_LA_Move_2.grid(row=1, column=3, sticky="w")

    button_LA_Move_3 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left  Move 3",
        command=lambda t=np.array([CMD_LA_MOVE, -150]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_LA_Move_3.grid(row=2, column=3, sticky="w")

    # --------------------------------------------------------------------------------------
    # Move Right arm
    # --------------------------------------------------------------------------------------
    button_RA_Move_1 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Move 1",
        command=lambda t=np.array([CMD_RA_MOVE, 500]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_RA_Move_1.grid(row=0, column=4, sticky="w")

    button_RA_Move_2 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Move 2",
        command=lambda t=np.array([CMD_RA_MOVE, 350]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_RA_Move_2.grid(row=1, column=4, sticky="w")

    button_RA_Move_3 = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right  Move 3",
        command=lambda t=np.array([CMD_RA_MOVE, 150]): createMoveCommand(
            mod_manager, t
        ),
    )
    button_RA_Move_3.grid(row=2, column=4, sticky="w")

    # --------------------------------------------------------------------------------------
    # Motion sensors front & back
    # --------------------------------------------------------------------------------------
    global button_Motion_Front
    button_Motion_Front = tk.Button(
        frame, height=1, width=10, text="Motion Front"
    )
    button_Motion_Front.grid(row=0, column=5, sticky="w")

    global button_Motion_Back
    button_Motion_Back = tk.Button(
        frame, height=1, width=10, text="Motion Back"
    )
    button_Motion_Back.grid(row=1, column=5, sticky="w")

    # --------------------------------------------------------------------------------------
    # Distance sensors
    # --------------------------------------------------------------------------------------
    global distance1_Front
    distance1_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance1_Front.grid(row=0, column=6, sticky="w")

    global distance2_Front
    distance2_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance2_Front.grid(row=1, column=6, sticky="w")

    global distance3_Front
    distance3_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance3_Front.grid(row=2, column=6, sticky="w")

    global distance4_Front
    distance4_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance4_Front.grid(row=3, column=6, sticky="w")

    global distance5_Front
    distance5_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance5_Front.grid(row=4, column=6, sticky="w")

    global distance6_Front
    distance6_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance6_Front.grid(row=5, column=6, sticky="w")

    global distance7_Front
    distance7_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance7_Front.grid(row=6, column=6, sticky="w")

    global distance8_Front
    distance8_Front = tk.Button(frame, height=1, width=10, text="inf")
    distance8_Front.grid(row=7, column=6, sticky="w")

    # --------------------------------------------------------------------------------------
    # Generic buttons
    # --------------------------------------------------------------------------------------
    button_clear = tk.Button(
        frame,
        height=1,
        text="Clear",
        command=lambda t='"failed"': clear_logging(),
    )
    button_clear.grid(row=11, column=0, columnspan=1, sticky="w")

    # --------------------------------------------------------------------------------------
    # Canvas for the rotating arrow
    # --------------------------------------------------------------------------------------
    canvas_width = 100
    canvas_height = 100
    canvas = tk.Canvas(
        frame, width=canvas_width, height=canvas_height, bg="white"
    )
    canvas.grid(
        row=0, rowspan=3, column=7, sticky="w"
    )  # Place the canvas in the grid

    arrow_center = (canvas_width // 2, canvas_height // 2)

    global compass_button
    compass_button = tk.Button(frame, height=1, width=10, text="inf")
    compass_button.grid(row=3, column=7, sticky="w")

    # --------------------------------------------------------------------------------------
    # streaming data area
    # --------------------------------------------------------------------------------------
    stream_area = scrolledtext.ScrolledText(
        frame, width=200, height=20, wrap="word"
    )
    stream_area.grid(row=10, column=0, columnspan=10, sticky="w")

    return root, stream_area, canvas


# Draw the rotating arrow
def draw_arrow(angle):
    global canvas

    canvas.delete("arrow")

    x, y = (50, 50)
    radians = math.radians(angle)
    x_end = x + 30 * math.cos(radians)
    y_end = y - 30 * math.sin(
        radians
    )  # Negative because canvas y-coordinates increase downwards
    canvas.create_line(
        x,
        y,
        x_end,
        y_end,
        arrow=tk.LAST,
        fill="blue",
        width=2,
        tags="arrow",
    )


# def rotate_arrow():
#     global arrow_angle
#     arrow_angle = (arrow_angle + 5) % 360
#     draw_arrow(arrow_angle)
#     root.after(50, rotate_arrow)


# Function to handle pygame events
def handle_pygame_events():

    global axis0, axis1, axis2, axis3, axis4, axis5

    global button4_toggle
    global button5_toggle

    axis0_event = False
    axis1_event = False
    axis2_event = False
    axis3_event = False
    axis4_event = False
    axis5_event = False

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            root.quit()

        # Handle button presses
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed.")

            if event.button == 0:
                createLedCommand(
                    mod_manager,
                    np.array([CMD_BA_COLOR, CMD_GREEN, CMD_LED_ON]),
                )

            if event.button == 1:
                createLedCommand(
                    mod_manager, np.array([CMD_BA_COLOR, CMD_RED, CMD_LED_ON])
                )

            if event.button == 2:
                createLedCommand(
                    mod_manager, np.array([CMD_BA_COLOR, CMD_BLUE, CMD_LED_ON])
                )

            if event.button == 3:
                createLedCommand(
                    mod_manager, np.array([CMD_BA_COLOR, CMD_ALL, CMD_LED_OFF])
                )

            # Left forward
            if event.button == 4:
                button4_toggle += 1
                button4_toggle = button4_toggle & 1

                if button4_toggle == 1:
                    createMoveCommand(
                        mod_manager, np.array([CMD_LA_MOVE, -400])
                    )

                if button4_toggle == 0:
                    createMoveCommand(
                        mod_manager, np.array([CMD_LA_MOVE, -200])
                    )

            # Left forward
            if event.button == 5:
                button5_toggle += 1
                button5_toggle = button5_toggle & 1

                if button5_toggle == 1:
                    createMoveCommand(
                        mod_manager, np.array([CMD_RA_MOVE, 400])
                    )

                if button5_toggle == 0:
                    createMoveCommand(
                        mod_manager, np.array([CMD_RA_MOVE, 200])
                    )

            # if (event.button == 5):
            #     createMoveCommand(mod_manager, np.array([CMD_RA_MOVE, -100]))
            # if (event.button == 6):
            #     createMoveCommand(mod_manager, np.array([CMD_LA_MOVE, -100]))
            # if (event.button == 7):
            #     createMoveCommand(mod_manager, np.array([CMD_RA_MOVE, +100]))

        elif event.type == pygame.JOYBUTTONUP:
            print(f"Button {event.button} released.")

            # if (event.button == 4):
            #     createMoveCommand(mod_manager, np.array([CMD_LA_MOVE, 0]))
            # if (event.button == 5):
            #     createMoveCommand(mod_manager, np.array([CMD_RA_MOVE, 0]))
            # if (event.button == 6):
            #     createMoveCommand(mod_manager, np.array([CMD_LA_MOVE, 0]))
            # if (event.button == 7):
            #     createMoveCommand(mod_manager, np.array([CMD_RA_MOVE, 0]))

        # Handle joystick movements
        if event.type == pygame.JOYAXISMOTION:
            # print(f"Axis {event.axis} moved to {event.value}.")

            # Only keep the fist value of the queue
            if event.axis == 0:
                axis0_event = True
                axis0 = event.value

            if event.axis == 1:
                axis1_event = True
                axis1 = event.value

            if event.axis == 2:
                axis2_event = True
                axis2 = event.value

            if event.axis == 3:
                axis3_event = True
                axis3 = event.value

            if event.axis == 4:
                axis4_event = True
                axis4 = event.value

            if event.axis == 5:
                axis5_event = True
                axis5 = event.value

    if (axis0_event == True) or (axis1_event == True) or (axis3_event == True):
        print(
            f"Axis 0 : {axis0:.2f}, Axis 1 : {axis1:.2f}, Axis 3 : {-1*axis3:.2f}"
        )
        createBaseCommand(
            mod_manager,
            np.array(
                [
                    CMD_BASE_MOVE,
                    int(axis0 * 50),
                    int(-1 * axis1 * 50),
                    int(-1 * axis3 * 50),
                ]
            ),
        )

    if axis2_event == True:
        print(f"Axis 2 : {-1*axis2:.2f}")
        createMoveCommand(
            mod_manager, np.array([CMD_LA_MOVE, int(axis2 + 1) * -200])
        )

    if axis5_event == True:
        print(f"Axis 5 : {-1*axis5:.2f}")
        createMoveCommand(
            mod_manager, np.array([CMD_RA_MOVE, int(axis5 + 1) * 200])
        )

    # Schedule the function to run again after 100 milliseconds
    root.after(100, handle_pygame_events)


def main():
    # Initialize pygame
    pygame.init()

    # Initialize the joysticks
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("No joystick detected.")
    else:
        print(f"Detected {joystick_count} joystick(s).")

        # Select the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Using joystick: {joystick.get_name()}")

        # Get the number of buttons
        num_buttons = joystick.get_numbuttons()
        print(f"Number of buttons: {num_buttons}")

    # mod_manager = ModManager(port='/dev/ttyACM0', baudrate=115200)
    global mod_manager
    global root
    # global text_area
    global stream_area
    global canvas

    if "LINUX" in platform.system().upper():
        print("Linux detected!")
        mod_manager = ModManager(
            port1="/dev/ttyACM0", port2="/dev/ttyACM1", baudrate=115200
        )
    else:
        mod_manager = ModManager(port="COM9", baudrate=115200)

    root, stream_area, canvas = show_gui(mod_manager)

    # Set the receive callback
    mod_manager.open_port()
    mod_manager.set_receive_callback(my_receive_callback, stream_area)

    time.sleep(0.1)

    # Call the handle_pygame_events function initially
    root.after(100, handle_pygame_events)

    root.mainloop()


if __name__ == "__main__":
    main()

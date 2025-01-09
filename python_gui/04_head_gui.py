import tkinter as tk
from tkinter import scrolledtext
from tkinter import font

import time
import pygame

from ModManager import ModManager
import numpy as np
import platform
import math

DEBUG = True
# DEBUG = False

RESP_BIT = 0x80

CMD_VERSION = 0x01
CMD_LA_COLOR = 0x10
CMD_RA_COLOR = 0x11
CMD_BASE_COLOR = 0x12
CMD_BA_COLOR = 0x13  # ?
CMD_LARA_COLOR = 0x14  # Left arm & Right arm
CMD_LEFTHEAD_COLOR = 0x15
CMD_RIGHTHEAD_COLOR = 0x16

CMD_GET_ENCODERS = 0x20
CMD_GET_MOTIONSENSORS = 0x21
CMD_GET_DISTANCESENSORS = 0x22
CMD_GET_COMPASS = 0x23
CMD_GET_BATTERY = 0x24
CMD_GET_HEAD_ENCODERS = 0x20


CMD_LA_MOVE = 0x30
CMD_RA_MOVE = 0x31
CMD_BASE_MOVE = 0x32
CMD_COMP_MOVE = 0x33
CMD_BASE_BRAKE = 0x34

CMD_COLOR_NONE = 0
CMD_RED = 1
CMD_GREEN = 2
CMD_BLUE = 3
CMD_WHITE = 4
CMD_REDGREEN = 5

CMD_LED_NONE = 0
CMD_LED_OFF = 1
CMD_LED_ON = 2
CMD_LED_BLINK_OFF = 3
CMD_LED_BLINK_SLOW = 4
CMD_LED_BLINK_FAST = 5
CMD_LED_BLINK_VERYFAST = 6

button_version = []

compass_button = []

canvas = []


# ===============================================================================
# Define the receive callback function
# ===============================================================================
def my_receive_callback(data, stream_area):
    # hex_values = " ".join([format(x, "02X") for x in data])
    # print("< " + hex_values)

    # Decode
    response = data[1]

    if response == (CMD_VERSION | RESP_BIT):
        try:
            string_from_bytearray = data[3:-2].decode("utf-8")
            stream_area.insert(tk.END, "< " + string_from_bytearray + "\n")
            stream_area.yview_moveto(1)  # Scrolling to the bottom
        except:
            print("Version bytes error")

    if response == (CMD_GET_HEAD_ENCODERS | RESP_BIT):
        try:
            new_byte_array = data[3:-2]
            int16_array_5 = np.frombuffer(new_byte_array, dtype=">i2")

            if DEBUG:
                stream_area.insert(tk.END, "< Encoders   : " + str(int16_array_5) + "\n")

            stream_area.yview_moveto(1)  # Scrolling to the bottom
        except:
            print("Encoders bytes error")


def createCommand(mod_manager, InputCommmand, Parameters):

    if InputCommmand == CMD_VERSION:
        mod_manager.cmd_Generic(CMD_VERSION, 0, 0)

    return


def createLedCommand(mod_manager, Parameters):

    print(Parameters)
    mod_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

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
    root.title("Sara Developement User Interface - HEAD")

    # Set up the window close protocol
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # root.tk.call('tk', 'scaling', 0.5)

    frame = tk.Frame(root)
    frame.pack(padx=20, pady=20)  # Adding padding for demonstration

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
    # Left Head colors
    # --------------------------------------------------------------------------------------
    button_LH_White = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left White",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_LH_White.grid(row=0, column=0, sticky="w")

    button_LH_Red = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Red",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_RED, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_LH_Red.grid(row=1, column=0, sticky="w")

    button_LH_Green = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Green",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_GREEN, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_LH_Green.grid(row=2, column=0, sticky="w")

    button_LH_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Blue",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_BLUE, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_LH_Blue.grid(row=3, column=0, sticky="w")

    button_LH_Off = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Off",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_COLOR_NONE, CMD_LED_OFF]): createLedCommand(mod_manager, t),
    )
    button_LH_Off.grid(row=4, column=0, sticky="w")

    button_LH_Blue_Blink = tk.Button(
        frame,
        height=1,
        width=10,
        text="Left Blue Blink",
        command=lambda t=np.array([CMD_LEFTHEAD_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(
            mod_manager, t
        ),
    )
    button_LH_Blue_Blink.grid(row=5, column=0, sticky="w")

    # --------------------------------------------------------------------------------------
    # Right Head colors
    # --------------------------------------------------------------------------------------
    button_RH_White = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right White",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_RH_White.grid(row=0, column=1, sticky="w")

    button_RH_Red = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Red",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_RED, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_RH_Red.grid(row=1, column=1, sticky="w")

    button_RH_Green = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Green",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_GREEN, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_RH_Green.grid(row=2, column=1, sticky="w")

    button_RH_Blue = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Blue",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_BLUE, CMD_LED_ON]): createLedCommand(mod_manager, t),
    )
    button_RH_Blue.grid(row=3, column=1, sticky="w")

    button_RH_Off = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Off",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_COLOR_NONE, CMD_LED_OFF]): createLedCommand(mod_manager, t),
    )
    button_RH_Off.grid(row=4, column=1, sticky="w")

    button_RH_Blue_Blink = tk.Button(
        frame,
        height=1,
        width=10,
        text="Right Blue Blink",
        command=lambda t=np.array([CMD_RIGHTHEAD_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(
            mod_manager, t
        ),
    )
    button_RH_Blue_Blink.grid(row=5, column=1, sticky="w")

    # --------------------------------------------------------------------------------------
    # streaming data area
    # --------------------------------------------------------------------------------------
    stream_area = scrolledtext.ScrolledText(frame, width=200, height=20, wrap="word")
    stream_area.grid(row=10, column=0, columnspan=10, sticky="w")

    return root, stream_area, canvas


def main():
    global mod_manager
    global root
    global stream_area
    global canvas

    if "LINUX" in platform.system().upper():
        print("Linux detected!")
        mod_manager = ModManager(port1="/dev/ttyACM0", port2="/dev/ttyACM99", baudrate=115200)
    else:
        mod_manager = ModManager(port1="COM7", port2="COM10", baudrate=115200)

    root, stream_area, canvas = show_gui(mod_manager)

    # Set the receive callback
    mod_manager.open_port()
    mod_manager.set_receive_callback(my_receive_callback, stream_area)

    time.sleep(0.1)

    root.mainloop()


if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import scrolledtext
from tkinter import font

import time
# from time import gmtime, strftime
# import sys
# import json
# from datetime import date, datetime, timedelta
# import math
# import random
# import string


from ModManager import ModManager
import numpy as np
import platform

RESP_BIT = 0x80

CMD_VERSION     = 0x01
CMD_LA_COLOR    = 0x10
CMD_RA_COLOR    = 0x11
CMD_BASE_COLOR  = 0x12

CMD_RED     = 1
CMD_GREEN   = 2
CMD_BLUE    = 3
CMD_WHITE   = 4
CMD_ALL     = 5

CMD_LED_NONE            = 0
CMD_LED_OFF             = 1
CMD_LED_ON              = 2
CMD_LED_BLINK_OFF       = 3
CMD_LED_BLINK_SLOW      = 4
CMD_LED_BLINK_FAST      = 5
CMD_LED_BLINK_VERYFAST  = 6

#===============================================================================
# Define the receive callback function
#===============================================================================
def my_receive_callback(data, stream_area):
    hex_values = ' '.join([format(x, '02X') for x in data])
    print("< " + hex_values)
    
    # Decode
    response = data[1]
    
    if (response == (CMD_VERSION | RESP_BIT)):
        
        string_from_bytearray = data[3:-2].decode('utf-8') 
        
        text_area.insert(tk.END, "< " + string_from_bytearray + "\n")
        text_area.yview_moveto(1)  # Scrolling to the bottom        
    
def createCommand(mod_manager, InputCommmand, Parameters):
    
    if (InputCommmand == CMD_VERSION):
        mod_manager.cmd_Generic(CMD_VERSION, 0, 0)

    return


def createLedCommand(mod_manager, Parameters):
    
    print(Parameters)
    mod_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

    return


#===============================================================================
# Clear logging
#===============================================================================
def clear_logging():
    text_area.delete('1.0', 'end')  # Deletes from start to end
    stream_area.delete('1.0', 'end')  # Deletes from start to end
    return


def on_closing():
    # Ask the user for confirmation before closing the window
    mod_manager.close_port()
    time.sleep(1)
    root.destroy()  # This will close the window
        
#===============================================================================
# Show a basic GUI
#===============================================================================
def show_gui(mod_manager):
    root = tk.Tk()
    root.title("Sara Developement User Interface")
    
    
    # Set up the window close protocol
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # root.tk.call('tk', 'scaling', 0.5) 

    frame = tk.Frame(root)
    frame.pack(padx=20, pady=20)  # Adding padding for demonstration

    # default_font = font.nametofont("TkDefaultFont")  # Get the default font
    # default_font.configure(family="courier", size=10)  # Change family and size
    # font.nametofont("TkHeadingFont").configure(family="Ubuntu", size=10)  # Change heading font

    #--------------------------------------------------------------------------------------
    # Left arm
    #--------------------------------------------------------------------------------------
    button_version = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Get Version", 
                               command=lambda t=[0]: createCommand(mod_manager, CMD_VERSION, t))
    button_version.grid(row=0, column=0, sticky="w")

    button_LA_Off = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Left White", 
                              command=lambda t=np.array([CMD_LA_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_LA_Off.grid(row=1, column=0, sticky="w")

    button_LA_Red = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Left Red", 
                              command=lambda t=np.array([CMD_LA_COLOR, CMD_RED, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_LA_Red.grid(row=2, column=0, sticky="w")

    button_LA_Green = tk.Button(frame, 
                                height= 1, 
                                width=10, 
                                text="Left Green", 
                                command=lambda t=np.array([CMD_LA_COLOR, CMD_GREEN, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_LA_Green.grid(row=3, column=0, sticky="w")

    button_LA_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Left Blue", 
                               command=lambda t=np.array([CMD_LA_COLOR, CMD_BLUE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_LA_Blue.grid(row=4, column=0, sticky="w")


    button_LA_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Left Off", 
                               command=lambda t=np.array([CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]): createLedCommand(mod_manager, t))
    button_LA_Blue.grid(row=5, column=0, sticky="w")


    button_LA_Blue_Blink = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Left Blue Blink", 
                               command=lambda t=np.array([CMD_LA_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(mod_manager, t))
    button_LA_Blue_Blink.grid(row=6, column=0, sticky="w")

    #--------------------------------------------------------------------------------------
    # Right arm
    #--------------------------------------------------------------------------------------
    button_RA_Off = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Right White", 
                              command=lambda t=np.array([CMD_RA_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_RA_Off.grid(row=1, column=1, sticky="w")

    button_RA_Red = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Right Red", 
                              command=lambda t=np.array([CMD_RA_COLOR, CMD_RED, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_RA_Red.grid(row=2, column=1, sticky="w")

    button_RA_Green = tk.Button(frame, 
                                height= 1, 
                                width=10, 
                                text="Right Green", 
                                command=lambda t=np.array([CMD_RA_COLOR, CMD_GREEN, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_RA_Green.grid(row=3, column=1, sticky="w")

    button_RA_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Right Blue", 
                               command=lambda t=np.array([CMD_RA_COLOR, CMD_BLUE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_RA_Blue.grid(row=4, column=1, sticky="w")


    button_RA_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Right Off", 
                               command=lambda t=np.array([CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]): createLedCommand(mod_manager, t))
    button_RA_Blue.grid(row=5, column=1, sticky="w")


    button_RA_Blue_Blink = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Right Blue Blink", 
                               command=lambda t=np.array([CMD_RA_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(mod_manager, t))
    button_RA_Blue_Blink.grid(row=6, column=1, sticky="w")

    #--------------------------------------------------------------------------------------
    # Right arm
    #--------------------------------------------------------------------------------------
    button_Base_Off = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Base White", 
                              command=lambda t=np.array([CMD_BASE_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_Base_Off.grid(row=1, column=2, sticky="w")

    button_Base_Red = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Base Red", 
                              command=lambda t=np.array([CMD_BASE_COLOR, CMD_RED, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_Base_Red.grid(row=2, column=2, sticky="w")

    button_Base_Green = tk.Button(frame, 
                                height= 1, 
                                width=10, 
                                text="Base Green", 
                                command=lambda t=np.array([CMD_BASE_COLOR, CMD_GREEN, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_Base_Green.grid(row=3, column=2, sticky="w")

    button_Base_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Base Blue", 
                               command=lambda t=np.array([CMD_BASE_COLOR, CMD_BLUE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_Base_Blue.grid(row=4, column=2, sticky="w")


    button_Base_Blue = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Base Off", 
                               command=lambda t=np.array([CMD_BASE_COLOR, CMD_ALL, CMD_LED_OFF]): createLedCommand(mod_manager, t))
    button_Base_Blue.grid(row=5, column=2, sticky="w")


    button_Base_Blue_Blink = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Base Blue Blink", 
                               command=lambda t=np.array([CMD_BASE_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(mod_manager, t))
    button_Base_Blue_Blink.grid(row=6, column=2, sticky="w")

    #--------------------------------------------------------------------------------------
    # Generic buttons
    #--------------------------------------------------------------------------------------
    button_clear = tk.Button(frame, height= 1, text="Clear", command=lambda t="\"failed\"": clear_logging())
    button_clear.grid(row=11, column=0, columnspan=1, sticky="w")


    #--------------------------------------------------------------------------------------
    # event data area
    #--------------------------------------------------------------------------------------
    text_area = scrolledtext.ScrolledText(frame, width = 100, height=20, wrap="word")
    text_area.grid(row=9, column=0, columnspan=10, sticky="w")

    #--------------------------------------------------------------------------------------
    # streaming data area
    #--------------------------------------------------------------------------------------
    stream_area = scrolledtext.ScrolledText(frame, width = 100, height=10, wrap="word")
    stream_area.grid(row=10, column=0, columnspan=10, sticky="w")

    return root, text_area, stream_area


def main():
    # mod_manager = ModManager(port='/dev/ttyACM0', baudrate=115200)
    global mod_manager 
    global root
    global text_area
    global stream_area

   
    if ("LINUX" in platform.system().upper()):
        print ("Linux detected!")
        mod_manager = ModManager(port='/dev/ttyACM0', baudrate=115200)
        # mod_manager = ModManager(port='/dev/ttyACM1', baudrate=115200)
    else:
        mod_manager = ModManager(port='COM9', baudrate=115200)
        
   
    root, text_area, stream_area = show_gui(mod_manager)

    # # List available fonts (optional, for debugging)
    # available_fonts = font.families()
    # print("Available Fonts:", available_fonts)
  
    # Set the receive callback
    mod_manager.set_receive_callback(my_receive_callback, stream_area)
    mod_manager.open_port()
    time.sleep(0.1)
    
    root.mainloop()

if __name__== "__main__":
    main()
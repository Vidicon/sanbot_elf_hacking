import tkinter as tk
from tkinter import scrolledtext
from tkinter import font

import time
import pygame

from ModManager import ModManager
import numpy as np
import platform

RESP_BIT = 0x80

CMD_VERSION     = 0x01
CMD_LA_COLOR    = 0x10
CMD_RA_COLOR    = 0x11
CMD_BASE_COLOR  = 0x12

CMD_GET_ENCODERS  = 0x20

CMD_LA_MOVE		= 0x30
CMD_RA_MOVE		= 0x31
CMD_BASE_MOVE   = 0x32


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

axis0 = 0
axis1 = 0
axis2 = 0
axis3 = 0
axis4 = 0
axis5 = 0


#===============================================================================
# Define the receive callback function
#===============================================================================
def my_receive_callback(data, stream_area):
    hex_values = ' '.join([format(x, '02X') for x in data])
    # print("< " + hex_values)
    
    # Decode
    response = data[1]
    
    if (response == (CMD_VERSION | RESP_BIT)):
        
        string_from_bytearray = data[3:-2].decode('utf-8') 
        
        text_area.insert(tk.END, "< " + string_from_bytearray + "\n")
        text_area.yview_moveto(1)  # Scrolling to the bottom        
    
    
    if (response == (CMD_GET_ENCODERS | RESP_BIT)):
        new_byte_array = data[3:-2]
        int16_array_5 = np.frombuffer(new_byte_array, dtype='>i2')
        
        stream_area.insert(tk.END, "< Encoders : " + str(int16_array_5) + "\n")
        stream_area.yview_moveto(1)  # Scrolling to the bottom        
    
    
def createCommand(mod_manager, InputCommmand, Parameters):
    
    if (InputCommmand == CMD_VERSION):
        mod_manager.cmd_Generic(CMD_VERSION, 0, 0)

    return


def createLedCommand(mod_manager, Parameters):
    
    print(Parameters)
    mod_manager.cmd_Generic(Parameters[0], 2, np.array(Parameters[1:]))

    return

def createMoveCommand(mod_manager, Parameters):
    high = (int(Parameters[1]) >> 8)  & 0xff;
    low  = (int(Parameters[1]) & 0xff);
            
    mod_manager.cmd_Generic(Parameters[0], 2, np.array([high, low]))

    return

def createBaseCommand(mod_manager, Parameters):
    mod_manager.cmd_Generic(Parameters[0], 3, np.array(Parameters[1:]))

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

    button_LA_White = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Left White", 
                              command=lambda t=np.array([CMD_LA_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_LA_White.grid(row=1, column=0, sticky="w")

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


    button_LA_Off = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Left Off", 
                               command=lambda t=np.array([CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]): createLedCommand(mod_manager, t))
    button_LA_Off.grid(row=5, column=0, sticky="w")


    button_LA_Blue_Blink = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Left Blue Blink", 
                               command=lambda t=np.array([CMD_LA_COLOR, CMD_BLUE, CMD_LED_BLINK_VERYFAST]): createLedCommand(mod_manager, t))
    button_LA_Blue_Blink.grid(row=6, column=0, sticky="w")

    #--------------------------------------------------------------------------------------
    # Right arm
    #--------------------------------------------------------------------------------------
    button_RA_White = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Right White", 
                              command=lambda t=np.array([CMD_RA_COLOR, CMD_WHITE, CMD_LED_ON]): createLedCommand(mod_manager, t))
    button_RA_White.grid(row=1, column=1, sticky="w")

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


    button_RA_Off = tk.Button(frame, 
                               height= 1, 
                               width=10, 
                               text="Right Off", 
                               command=lambda t=np.array([CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]): createLedCommand(mod_manager, t))
    button_RA_Off.grid(row=5, column=1, sticky="w")


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
    # Move Left arm
    #--------------------------------------------------------------------------------------
    button_LA_Move_1 = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Left Move 1", 
                              command=lambda t=np.array([CMD_LA_MOVE, 0]): createMoveCommand(mod_manager, t))
    button_LA_Move_1.grid(row=1, column=3, sticky="w")

    button_LA_Move_2 = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Left Move 2", 
                              command=lambda t=np.array([CMD_LA_MOVE, 100]): createMoveCommand(mod_manager, t))
    button_LA_Move_2.grid(row=2, column=3, sticky="w")

    button_LA_Move_3 = tk.Button(frame, 
                                height= 1, 
                                width=10, 
                                text="Left  Move 3", 
                                command=lambda t=np.array([CMD_LA_MOVE, 300]): createMoveCommand(mod_manager, t))
    button_LA_Move_3.grid(row=3, column=3, sticky="w")

    #--------------------------------------------------------------------------------------
    # Move Right arm
    #--------------------------------------------------------------------------------------
    button_RA_Move_1 = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Right Move 1", 
                              command=lambda t=np.array([CMD_RA_MOVE, 0]): createMoveCommand(mod_manager, t))
    button_RA_Move_1.grid(row=1, column=4, sticky="w")

    button_RA_Move_2 = tk.Button(frame, 
                              height= 1, 
                              width=10, 
                              text="Right Move 2", 
                              command=lambda t=np.array([CMD_RA_MOVE, -100]): createMoveCommand(mod_manager, t))
    button_RA_Move_2.grid(row=2, column=4, sticky="w")

    button_RA_Move_3 = tk.Button(frame, 
                                height= 1, 
                                width=10, 
                                text="Right  Move 3", 
                                command=lambda t=np.array([CMD_RA_MOVE, -300]): createMoveCommand(mod_manager, t))
    button_RA_Move_3.grid(row=3, column=4, sticky="w")
    
    #--------------------------------------------------------------------------------------
    # Generic buttons
    #--------------------------------------------------------------------------------------
    button_clear = tk.Button(frame, height= 1, text="Clear", command=lambda t="\"failed\"": clear_logging())
    button_clear.grid(row=11, column=0, columnspan=1, sticky="w")


    #--------------------------------------------------------------------------------------
    # event data area
    #--------------------------------------------------------------------------------------
    text_area = scrolledtext.ScrolledText(frame, width = 200, height=20, wrap="word")
    text_area.grid(row=9, column=0, columnspan=10, sticky="w")

    #--------------------------------------------------------------------------------------
    # streaming data area
    #--------------------------------------------------------------------------------------
    stream_area = scrolledtext.ScrolledText(frame, width = 200, height=20, wrap="word")
    stream_area.grid(row=10, column=0, columnspan=10, sticky="w")

    return root, text_area, stream_area

# Function to handle pygame events
def handle_pygame_events():
    
    global axis0, axis1, axis2, axis3, axis4, axis5
    
    axis0_event= False
    axis1_event= False
    axis2_event= False
    axis3_event= False
    axis4_event= False
    axis5_event= False
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            root.quit()

        # Handle button presses
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed.")

            if (event.button == 0):
                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]))

                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_GREEN, CMD_LED_ON]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_GREEN, CMD_LED_ON]))

            if (event.button == 1):
                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]))

                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_RED, CMD_LED_ON]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_RED, CMD_LED_ON]))

            if (event.button == 2):
                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_ALL, CMD_LED_OFF]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_ALL, CMD_LED_OFF]))

                createLedCommand(mod_manager, np.array([CMD_LA_COLOR, CMD_BLUE, CMD_LED_ON]))
                createLedCommand(mod_manager, np.array([CMD_RA_COLOR, CMD_BLUE, CMD_LED_ON]))

            # if (event.button == 4):
            #     createMoveCommand(mod_manager, np.array([CMD_LA_MOVE, +100]))
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
            if (event.axis == 0):
                axis0_event = True
                axis0 = event.value

            if (event.axis == 1):
                axis1_event = True
                axis1 = event.value

            if (event.axis == 2):
                axis2_event = True
                axis2 = event.value

            if (event.axis == 3):
                axis3_event = True
                axis3 = event.value
            
            if (event.axis == 4):
                axis4_event = True
                axis4 = event.value
           
            if (event.axis == 5):
                axis5_event = True
                axis5 = event.value

    if (axis0_event == True) or (axis1_event == True) or (axis3_event == True): 
        print(f"Axis 0 : {axis0:.2f}, Axis 1 : {axis1:.2f}, Axis 3 : {-1*axis3:.2f}")
        createBaseCommand(mod_manager, np.array([CMD_BASE_MOVE, int(axis0*50), int(-1*axis1*50), int(-1*axis3*50)]))


    if (axis2_event == True): 
        print(f"Axis 2 : {-1*axis2:.2f}")
        createMoveCommand(mod_manager, np.array([CMD_LA_MOVE, int(axis2+1)*-200]))

    if (axis5_event == True): 
        print(f"Axis 5 : {-1*axis5:.2f}")
        createMoveCommand(mod_manager, np.array([CMD_RA_MOVE, int(axis5+1)*200]))


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
  
    # Call the handle_pygame_events function initially
    root.after(100, handle_pygame_events)  
  
    root.mainloop()

if __name__== "__main__":
    main()
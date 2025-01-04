#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  7 21:09:03 2024

@author: matthijs
"""

import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)  # Adjust COM_PORT and baudrate as needed

# Wait briefly to avoid initial noise or boot data
time.sleep(0.5)  

# Flush the buffers to clear any unwanted data
ser.reset_input_buffer()
ser.reset_output_buffer()

# Start your intended communication here
# e.g., ser.write(b'\xAA')

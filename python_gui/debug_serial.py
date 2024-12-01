#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  7 20:38:48 2024

@author: matthijs
"""

import time

from ModManager import ModManager
import numpy as np
import platform

def main():

    if ("LINUX" in platform.system().upper()):
        print ("Linux detected!")
        mod_manager = ModManager(port1='/dev/ttyACM0', port2='/dev/ttyACM1', baudrate=115200)
    else:
        mod_manager = ModManager(port1='COM9', baudrate=115200)
        
    # Set the receive callback
    mod_manager.open_port()

    time.sleep(100)
    
    mod_manager.close_port()

if __name__== "__main__":
    main()
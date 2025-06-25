import time
import numpy as np
from ModManager import ModManager

# Define the receive callback function
def my_receive_callback(data):
    hex_values = ' '.join([format(x, '02X') for x in data])
    print(hex_values)
    

def main():
    # mod_manager = ModManager(port='/dev/ttyACM0', baudrate=115200)
    mod_manager = ModManager(port='COM9', baudrate=115200)
    
    # Set the receive callback
    mod_manager.set_receive_callback(my_receive_callback)
 
    mod_manager.open_port()
    time.sleep(0.1)

    mod_manager.cmd_GetVersion()
    time.sleep(0.1)
    mod_manager.cmd_Generic(0x42, 4, np.array([0, 1, 2, 3]))
    time.sleep(0.1)


    time.sleep(3)
    # mod_manager.send_enable(ID=3, enable=0)

    mod_manager.close_port()
    
    # try:
    #     while(1):
    #         time.sleep(0.1)
            
    # except KeyboardInterrupt:
    #     print ('Interrupted!')
    
    return


if __name__== "__main__":
    main()
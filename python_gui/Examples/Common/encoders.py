import os
import sys
import platform
import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames


class Encoders:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".encoders"
        print("Adding " + "robot." + self.full_bodypart_name)

        self.encoders = np.zeros(5)

        self.valid_data = False
        self.error_counter = 0
        self.rx_counter = 0

    def new_data(self, data):
        try:
            datalength = data[2]
            assert datalength == 10, (
                self.full_bodypart_name + " data length not correct!"
            )

            for i in range(5):
                new_byte_array = data[3 + i * 2 : -2]

                uint8_array = np.frombuffer(new_byte_array, dtype=">u1")
                combined_int = int.from_bytes(new_byte_array[0:2], byteorder="big")
                self.encoders[i] = float(combined_int)

            self.valid_data = True
            self.error_counter = 0
            self.rx_counter += 1
        except:
            print("Encoders data processing error")
            self.valid_data = False

    def print_values(self):
        print("Encoders   :", [int(value) for value in self.encoders])

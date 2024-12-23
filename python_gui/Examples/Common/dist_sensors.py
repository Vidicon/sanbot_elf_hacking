import os
import platform
import numpy as np
from Common.mod_manager import ModManager

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames


class DistanceSensors:
    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".distancesensors"

        self.sensors = np.ones(11) * 65295
        self.valid_data = False
        self.error_counter = 0

        print("Adding " + self.full_bodypart_name)

    def new_data(self, data):
        try:
            datalength = data[2]
            assert datalength == 22, self.full_bodypart_name + " data length not correct!"

            for i in range(11):
                new_byte_array = data[3 + i * 2 : -2]

                uint8_array = np.frombuffer(new_byte_array, dtype=">u1")
                combined_int = int.from_bytes(new_byte_array[0:2], byteorder="big")
                self.sensors[i] = float(combined_int)

            self.valid_data = True
            self.error_counter = 0
        except:
            print("Distance sensors data processing error")

            self.error_counter += 1

            if self.error_counter > 3:
                self.valid_data = False

    def sensor_warning(self, threshold=25000):

        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors < threshold)

    def sensor_collision(self, threshold=15000):
        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors < threshold)

    def print_values(self):
        print(self.sensors)

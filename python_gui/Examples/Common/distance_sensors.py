import os
import platform
import numpy as np
from Common.mod_manager import ModManager

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames


class DistanceSensors:
    hoek_bottom = 22.5
    sensor_angles_bottom = np.array(
        [
            [-4 * hoek_bottom, -3 * hoek_bottom],
            [-3 * hoek_bottom, -2 * hoek_bottom],
            [-2 * hoek_bottom, -1 * hoek_bottom],
            [-1 * hoek_bottom, -0 * hoek_bottom],
            [0 * hoek_bottom, 1 * hoek_bottom],
            [1 * hoek_bottom, 2 * hoek_bottom],
            [2 * hoek_bottom, 3 * hoek_bottom],
            [3 * hoek_bottom, 4 * hoek_bottom],
        ]
    )

    hoek_mid = 22.5
    sensor_angles_mid = np.array(
        [
            [-0.5 * hoek_mid, 0.5 * hoek_mid],
            [-1.5 * hoek_mid, -0.5 * hoek_mid],
            [0.5 * hoek_mid, 1.5 * hoek_mid],
        ]
    )

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".distancesensors"

        self.sensors = np.ones(11) * 65295
        self.valid_data = False
        self.error_counter = 0
        self.rx_counter = 0

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
            self.rx_counter += 1
        except:
            print("Distance sensors data processing error")

            self.error_counter += 1

            if self.error_counter > 3:
                self.valid_data = False

    def sensor_warning(self, threshold=30000):

        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors < threshold)

    def sensor_collision(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors < threshold)

    def print_values(self):
        print(self.sensors)

    def get_all_values(self):
        return self.sensors

    def get_rx_counter(self):
        return self.rx_counter

    # --------------------------------------------------------------------------------
    # Basic checks where collision is
    # --------------------------------------------------------------------------------
    def is_collision_left(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return (self.sensors[0] < threshold) or (self.sensors[1] < threshold)

    def is_collision_frontleft(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return self.sensors[2] < threshold

    def is_collision_front(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return (
                (self.sensors[3] < threshold)
                or (self.sensors[4] < threshold)
                or (self.sensors[8] < threshold)
                or (self.sensors[9] < threshold)
                or (self.sensors[10] < threshold)
            )

    def is_collision_frontright(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return self.sensors[5] < threshold

    def is_collision_right(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return (self.sensors[6] < threshold) or (self.sensors[7] < threshold)

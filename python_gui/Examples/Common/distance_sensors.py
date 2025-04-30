import os
import sys
import platform
import numpy as np

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
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.sensors = np.ones(13) * 65295

        # Cliff sensors need to start at 0.
        self.sensors[10] = 0
        self.sensors[11] = 0

        self.valid_data = False
        self.error_counter = 0
        self.rx_counter = 0


    def new_data(self, data):
        try:
            datalength = data[2]

            assert datalength == 26, (
                self.full_bodypart_name + " data length not correct!"
            )

            for i in range(13):
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

        # ------------------------------------------------------------------------
        # Cliff sensors
        # ------------------------------------------------------------------------
        if self.sensors[11] >= 40000:
            print("Left cliff sensor too large value!")

        if self.sensors[12] >= 40000:
            print("Right cliff sensor too large value!")

    def sensor_warning(self, threshold=30000):

        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors[0:-2] < threshold)

    def sensor_collision(self, threshold=20000):
        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors[0:-2] < threshold)

    def sensor_cliffwarning(self):
        if self.valid_data == False:
            return True
        else:
            return (self.sensors[11] >= 40000) or self.sensors[12] >= 40000

    def print_values(self):
        print("Distances  :", [int(value) for value in self.sensors])

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

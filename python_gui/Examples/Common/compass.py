import os
import platform
import numpy as np
import math
import time

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.bridge_manager import BridgeManager

class Compass:
    def __init__(self, bridge_manager, bodypart):
        self.bridge_manager = bridge_manager
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".compass"

        self.sensors = np.zeros(1)
        self.abs_angle = 0
        self.valid_data = False
        self.error_counter = 0

        self.target_rotation = 0
        self.rotation_tmo_counter = 0
        self.rotation_tmo_threshold = 100

        print("Adding " + self.full_bodypart_name)

    def new_data(self, data):
        try:
            datalength = data[2]

            assert datalength == 6, self.full_bodypart_name + " data length not correct!"

            # for i in range(11):
            new_byte_array = data[3:6+3]

            int16_array_5 = np.frombuffer(new_byte_array, dtype=">i2")
            compass_angle, angle_degrees = self.calculate_angle(int16_array_5[0], int16_array_5[1])

            self.abs_angle = float(compass_angle)
            self.valid_data = True
            self.error_counter = 0

            print(f"Compass angle: {self.abs_angle:.0f} Deg ")

        except:
            hex_values = " ".join([format(x, "02X") for x in data])
            print("< " + hex_values)

            print("Compass data processing error")

            self.error_counter += 1

            if self.error_counter > 3:
                self.valid_data = False

    def read_abs_angle(self):
        return self.abs_angle

    # Function to calculate the angle
    def calculate_angle(self, x, y):
        # atan2(y, x) returns the angle in radians
        angle_radians = math.atan2(y, x)
        # Convert radians to degrees (optional)
        angle_degrees = math.degrees(angle_radians)

        #  convert to compass angles
        compass_degree = -1 * angle_degrees

        if compass_degree < 0:
            compass_degree += 360

        return compass_degree, angle_degrees

    # Rotate to an absolute angle using the compass
    def rotate_absolute(self, abs_rotation_angle=0, wait_for_finish=True, rotation_tmo_threshold=10):
        assert abs_rotation_angle < 360, "Invalid abs_rotation_angle (>360 Deg is not allowed)!"
        assert abs_rotation_angle >= 0, "Invalid abs_rotation_angle (<0 Deg is not allowed)!"

        self.target_rotation = int(abs_rotation_angle)
        self.rotate_ready = False
        self.rotate_result = False
        self.rotation_tmo_counter = 0

        print(f"Compass rotation to {self.target_rotation :.0f} Deg ", end="")

        # Send command
        self.bridge_manager.cmd_createCompassMoveCommand(
            SaraRobotCommands.CMD_COMP_MOVE, self.target_rotation, rotation_tmo_threshold
        )

        # Wait for the response message or a timeout of 20 seconds
        while self.rotate_ready == False:
            assert self.rotation_tmo_counter < (rotation_tmo_threshold * 10), "Rotate absolute timeout!"

            time.sleep(0.1)
            self.rotation_tmo_counter += 1
            print("-", end="")  # Print without a newline

        # Check the rotation result
        if self.rotate_result == 1:
            print("> ready")
        else:
            print("> failed")

        return

    def rotate_absolute_ready(self, data):
        try:
            datalength = data[2]

            try:
                assert datalength == 1, self.full_bodypart_name + " data length not correct!"

                self.rotate_result = data[3] == 1
            except AssertionError as e:
                self.rotate_result = 0

            self.rotate_ready = True
        except:
            print(self.full_bodypart_name + " data processing error")

        return

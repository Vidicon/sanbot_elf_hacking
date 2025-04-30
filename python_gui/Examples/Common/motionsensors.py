import numpy as np
from Common.sara_common import bodypart_to_string


class MotionSensors:
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.sensors = np.zeros(2)
        self.abs_angle = 0.0
        self.valid_data = False
        self.error_counter = 0


    def new_data(self, data):
        try:
            datalength = data[2]

            assert (
                datalength == 2
            ), f"{self.full_bodypart_name} data length not correct!"

            new_byte_array = data[3:-2]
            uint8_array = np.frombuffer(new_byte_array, dtype=">u1")

            self.sensors[0] = float(uint8_array[0])
            self.sensors[1] = float(uint8_array[1])

            self.valid_data = True
            self.error_counter = 0
        except Exception as e:
            print(f"Motion sensors data processing error: {e}")

            self.error_counter += 1
            if self.error_counter > 3:
                self.valid_data = False

    def print_values(self):
        print("Motion     :", [int(value) for value in self.sensors])

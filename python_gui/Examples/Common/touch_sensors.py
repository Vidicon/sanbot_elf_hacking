import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands

class TouchSensorsHead:
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

        self.sensors = np.zeros(8)
        self.callback = None


    def new_data(self, data):
        try:
            # print("Raw data received for body sensors:", [f"0x{byte:02X}" for byte in data])
            datalength = data[2]

            assert (
                datalength == 8
            ), f"{self.full_bodypart_name} data length not correct!"

            new_byte_array = data[3 : 3 + datalength]
            uint8_array = np.frombuffer(new_byte_array, dtype=">u1")

            self.sensors[0] = float(uint8_array[0])
            self.sensors[1] = float(uint8_array[1])
            self.sensors[2] = float(uint8_array[2])
            self.sensors[3] = float(uint8_array[3])
            self.sensors[4] = float(uint8_array[4])
            self.sensors[5] = float(uint8_array[5])
            self.sensors[6] = float(uint8_array[6])
            self.sensors[7] = float(uint8_array[7])

            self.valid_data = True
            self.error_counter = 0

            if self.callback is not None:
                self.callback()

        except Exception as e:
            print(f"Touch sensors head data processing error: {e}")

            self.error_counter += 1
            if self.error_counter > 3:
                self.valid_data = False

    def print_values(self):
        print("Touch head :", [int(value) for value in self.sensors])

    def get_all_values(self):
        return self.sensors

    def set_callback(self, callback):
        self.callback = callback
        return
    

class TouchSensorsBody:
    def __init__(self, bridge_manager, parent_name, instance_ENUM):
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_enum = instance_ENUM
        self.instance_name = f"{self.parent_name}.{bodypart_to_string(self.instance_enum)}"

        print(f"Adding {self.instance_name}")

        self.sensors = np.zeros(2)
        self.callback = None


    def new_data(self, data):
        try:
            # print("Raw data received for body sensors:", [f"0x{byte:02X}" for byte in data])
            datalength = data[2]

            assert (
                datalength == 2
            ), f"{self.full_bodypart_name} data length not correct!"

            new_byte_array = data[3 : 3 + datalength]
            uint8_array = np.frombuffer(new_byte_array, dtype=">u1")

            self.sensors[0] = float(uint8_array[0])
            self.sensors[1] = float(uint8_array[1])

            self.valid_data = True
            self.error_counter = 0

            if self.callback is not None:
                self.callback()

        except Exception as e:
            print(f"Touch sensors body data processing error: {e}")

            self.error_counter += 1
            if self.error_counter > 3:
                self.valid_data = False

    def print_values(self):
        print("Touch body:", [int(value) for value in self.sensors])

    def get_all_values(self):
        return self.sensors

    def set_callback(self, callback):
        self.callback = callback
        return
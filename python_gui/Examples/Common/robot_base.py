import os
import platform
import numpy as np
from datetime import datetime
import math

from Common.mod_manager import ModManager
from Common.colorled import ColorLed
from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands


class RobotBase:
    UP = 500
    FORWARD = 350
    DOWN = 100

    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.bodypart = bodypart
        self.full_bodypart_name = bodypart_to_string(bodypart)
        print("Adding " + self.full_bodypart_name)

        self.led = ColorLed(self.mod_manager, self.bodypart)
        self.motors = BaseMotors(self.mod_manager, self.bodypart)

    def move_stop(self):
        self.mod_manager.cmd_Generic(SaraRobotCommands.CMD_BASE_MOVE, 3, np.array([0, 0, 0]))

    def move(self, Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=0):

        assert abs(Sideways_Velocity <= 100), "Abs(Sideways) velocity too high"
        assert abs(Forward_Velocity <= 100), "Abs(Forward) velocity too high"
        assert abs(Rotation_Velocity <= 100), "Abs(Rotation) velocity too high"

        self.mod_manager.cmd_Generic(
            SaraRobotCommands.CMD_BASE_MOVE, 4, np.array([Sideways_Velocity, Forward_Velocity, Rotation_Velocity, 0])
        )

    def move_debug(self, Sideways_Velocity=0, Forward_Velocity=0, Rotation_Velocity=0, tmo=999):

        assert abs(Sideways_Velocity <= 100), "Abs(Sideways) velocity too high"
        assert abs(Forward_Velocity <= 100), "Abs(Forward) velocity too high"
        assert abs(Rotation_Velocity <= 100), "Abs(Rotation) velocity too high"

        self.mod_manager.cmd_Generic(
            SaraRobotCommands.CMD_BASE_MOVE, 4, np.array([Sideways_Velocity, Forward_Velocity, Rotation_Velocity, tmo])
        )

    # Not sure if robot has brakes on base motors!!!
    def brake(self, ApplyBrake=False):
        Brake = 1 if ApplyBrake else 0
        self.mod_manager.cmd_Generic(SaraRobotCommands.CMD_BASE_BRAKE, 1, np.array([Brake]))


class BaseMotors:
    def __init__(self, mod_manager, bodypart):
        self.mod_manager = mod_manager
        self.full_bodypart_name = bodypart_to_string(bodypart) + ".motors"
        print("Adding " + self.full_bodypart_name)

        self.encoders = np.zeros(3)

    def new_data(self, data):
        # try:
        #     datalength = data[2]
        #     assert datalength == 22, self.full_bodypart_name + " data length not correct!"

        #     for i in range(11):
        #         new_byte_array = data[3 + i * 2 : -2]

        #         uint8_array = np.frombuffer(new_byte_array, dtype=">u1")
        #         combined_int = int.from_bytes(new_byte_array[0:2], byteorder="big")
        #         self.sensors[i] = float(combined_int)

        #     self.valid_data = True
        #     self.error_counter = 0
        #     # print(datetime.now())
        # except:
        #     print("Distance sensors data processing error")

        #     self.error_counter += 1

        #     if self.error_counter > 3:
        #         self.valid_data = False
        return

body_parts_names = ["left_arm", "right_arm", "base", "head", "body", "battery", "distance_sensors"]


class SaraRobotCommands:
    RESP_BIT = 0x80

    CMD_VERSION = 0x01
    
    CMD_LA_COLOR = 0x10
    CMD_RA_COLOR = 0x11
    CMD_BASE_COLOR = 0x12
    CMD_BA_COLOR = 0x13
    CMD_LARA_COLOR = 0x14

    CMD_GET_ENCODERS = 0x20
    CMD_GET_MOTIONSENSORS = 0x21
    CMD_GET_DISTANCESENSORS = 0x22
    CMD_GET_COMPASS = 0x23
    CMD_GET_BATTERY = 0x24

    CMD_LA_MOVE = 0x30
    CMD_RA_MOVE = 0x31
    CMD_BASE_MOVE = 0x32
    CMD_COMP_MOVE = 0x33
    CMD_BASE_BRAKE = 0x34


def bodypart_to_string(bodypart):
    return body_parts_names[bodypart]


class SaraRobotPartNames:
    LEFTARM = 0
    RIGHTARM = 1
    BASE = 2
    HEAD = 3
    BODY = 4
    BATTERY = 5
    BODYDISTANCESENSORS = 6


class RobotArmPositions:
    UP = 450
    FORWARD = 350
    DOWN = 150

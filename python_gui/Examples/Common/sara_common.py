body_parts_names = [
    "left_arm",
    "right_arm",
    "base",
    "head",
    "body",
    "battery",
    "distance_sensors",
    "motion_sensors",
    "head",
    "left_led",
    "right_led",
    "pan_motor",
    "tilt_motor",
]   


class SaraRobotCommands:
    RESP_BIT = 0x80

    CMD_VERSION_BODY = 0x01
    CMD_LA_COLOR = 0x10
    CMD_RA_COLOR = 0x11
    CMD_BASE_COLOR = 0x12
    CMD_BA_COLOR = 0x13
    CMD_LARA_COLOR = 0x14
    CMD_COLOR_LAST = 0x1F

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
    CMD_LA_HOME = 0x35
    CMD_RA_HOME = 0x36
    CMD_BODY_LAST = 0x3F

    CMD_VERSION_HEAD = 0x40
    CMD_HEAD_PAN_MOVE = 0x41
    CMD_HEAD_PAN_BRAKE = 0x42
    CMD_HEAD_TILT_MOVE = 0x43
    CMD_HEAD_TILT_BRAKE = 0x44
    CMD_HEAD_LEFT_COLOR = 0x45
    CMD_HEAD_RIGHT_COLOR = 0x46
    CMD_HEAD_TILT_HOME = 0x47
    CMD_HEAD_PAN_HOME = 0x48
    CMD_HEAD_LAST = 0x4F


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
    MOTIONSENSORS = 7
    HEAD = 8
    LEFT_LED = 9
    RIGHT_LED = 10
    PAN_MOTOR = 11
    TILT_MOTOR = 12

class RobotArmPositions:
    UP = 450
    FORWARD = 350
    DOWN = 150
    LEFT = 100
    RIGHT = 200
    UP = 100
    DOWN = 200
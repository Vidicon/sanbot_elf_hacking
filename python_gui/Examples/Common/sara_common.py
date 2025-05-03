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
    CMD_HEAD_EYES = 0x49
    CMD_HEAD_LAST = 0x4F


def bodypart_to_string(bodypart):
    return body_parts_names[bodypart]

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
    "led",
    "led",
    "led",
    "left_led",
    "right_led",
    "pan_motor",
    "tilt_motor",
    "motor",
    "motor",
    "motors",
    "encoders",
    "compass",
    "battery",
    "eyes",
]   

class SaraRobotPartNames:
    LEFT_ARM = 0
    RIGHT_ARM = 1
    BASE = 2
    HEAD = 3
    BODY = 4
    BATTERY = 5
    BODYDISTANCESENSORS = 6
    MOTIONSENSORS = 7
    HEAD = 8
    
    LEFT_ARM_LED = 9
    RIGHT_ARM_LED = 10
    BASE_LED = 11
    HEAD_LEFT_LED = 12
    HEAD_RIGHT_LED = 13

    PAN_MOTOR = 14
    TILT_MOTOR = 15
    LEFT_ARM_MOTOR = 16
    RIGHT_ARM_MOTOR = 17
    BASE_MOTORS = 18

    ENCODERS = 19
    COMPASS = 20
    BATTERY = 21
    HEAD_EYES = 22

class RobotArmPositions:
    UP = 450
    FORWARD = 350
    DOWN = 150

class RobotHeadPositions:
    PAN_LEFT = 10
    PAN_RIGHT = 970
    PAN_MID = PAN_LEFT + (PAN_RIGHT - PAN_LEFT) / 2
    TILT_UP = 350
    TILT_DOWN = 10
    TILT_MID = TILT_UP + (TILT_DOWN - TILT_UP) / 2
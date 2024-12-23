body_parts_names = ["left_arm", "right_arm", "base", "head", "body", "battery", "distance_sensors"]


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

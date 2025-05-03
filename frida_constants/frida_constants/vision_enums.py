from enum import Enum


class Gestures(Enum):
    """Enum for gestures."""

    UNKNOWN = "unknown"
    WAVING = "waving"
    RAISING_LEFT_ARM = "raising_left_arm"
    RAISING_RIGHT_ARM = "raising_right_arm"
    POINTING_LEFT = "pointing_left"
    POINTING_RIGHT = "pointing_right"


class Poses(Enum):
    """Enum for poses"""

    UNKNOWN = "unknown"
    STANDING = "standing"
    SITTING = "sitting"
    LYING_DOWN = "lying_down"


class DetectBy(Enum):
    """Enum for detection types"""

    GESTURES = "gestures"
    POSES = "poses"
    COLOR = "color"

def is_value_in_enum(value, enum_class):
    return value in [member.value for member in enum_class]

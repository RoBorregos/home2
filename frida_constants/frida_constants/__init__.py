from enum import Enum

import frida_constants.hri_constants as hri_constants
from frida_constants.utils import parse_ros_config


class ModuleNames(Enum):
    HRI = hri_constants.__name__


__all__ = [hri_constants.__name__, parse_ros_config.__name__, "ModuleNames"]

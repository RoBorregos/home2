from enum import Enum

# Navigation Manager
# -----------------------------------------------------------------
###Check door service
CHECK_DOOR_SERVICE = "/is_door_open"


class DOOR_CHECK(Enum):
    TIMEOUT_SENSOR = 5.0
    TIMEOUT_TO_OPEN = 60.0  # Increase in case of required
    LIDAR_RANGE_MIN = 670
    LIDAR_RANGE_MAX = 70
    CHECKING_RATE = 0.5
    DOOR_DISTANCE = 1.0


###Map areas service
AREAS_SERVICE = "/areas_json"

# ------------------------------------------------------------------

# General constants
GOAL_TOPIC = "/navigate_to_pose"
SCAN_TOPIC = "/scan"


# To be changed
FOLLOWING_SERVICE = "/navigation/activate_following"

from enum import Enum

# Navigation Central
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

### General Constants
TIMEOUT_REQUIREMENTS = 0.8 
TIMEOUT_RTABMAP = 30.0
# ------------------------------------------------------------------


# Navigation Subtask Manager
# ------------------------------------------------------------------
class SUBTASK_MANAGER(Enum):
    SERVICE_TIMEOUT = 2.0
    AREAS_RETRIEVE_TIMEOUT = 2.0


# General constants
GOAL_TOPIC = "/navigate_to_pose"
SCAN_TOPIC = "/scan"
CAMERA_RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"
CAMERA_DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered" 

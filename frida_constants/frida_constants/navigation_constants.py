from enum import Enum

# Navigation Central
# -----------------------------------------------------------------
###Check door service
CHECK_DOOR_SERVICE = "/navigation/is_door_open"


class DOOR_CHECK(Enum):
    TIMEOUT_SENSOR = 5.0
    TIMEOUT_TO_OPEN = 60.0  # Increase in case of required
    LIDAR_RANGE_MIN = 670
    LIDAR_RANGE_MAX = 70
    CHECKING_RATE = 0.5
    DOOR_DISTANCE = 1.0


###Map areas service
AREAS_SERVICE = "/navigation/areas_json"

### Move to location service
MOVE_LOCATION_SERVICE = "/navigation/go_to_map_area"
GOAL_NAV_ACTION_SERVER = "/navigate_to_pose"

### Open-loop base motion (bypass Nav2)
MOVE_DISTANCE_SERVICE = "/navigation/move_distance"
MOVE_UNTIL_OBJECT_SERVICE = "/navigation/move_until_object"
CMD_VEL_TOPIC = "/cmd_vel"
BASE_FRAME = "base_link"
ODOM_FRAME = "odom"
MAP_FRAME = "map"
TIMEOUT_MOVE_DISTANCE_TF = 5.0


class MOVE_DISTANCE(Enum):
    LINEAR_SPEED = 0.12  # m/s while moving
    CONTROL_RATE = 20.0  # Hz of control loop publishing cmd_vel
    DISTANCE_TOLERANCE = 0.02  # m, stop when within tolerance of target
    MAX_EXTRA_TIME = 10.0  # s of slack over the ideal travel time


class MOVE_UNTIL_OBJECT(Enum):
    LINEAR_SPEED = 0.08  # m/s (slower than move_distance for safety)
    CONTROL_RATE = 20.0  # Hz
    DEFAULT_STOP_DISTANCE = 0.30  # m, default target clearance to the object
    LIDAR_TIMEOUT = 5.0  # s to wait for first scan
    MAX_TRAVEL_TIME = 30.0  # s deadline before aborting
    # Angular window (degrees, 0 = front of robot) used to watch for the
    # obstacle. Defaults: rear-facing ±30° when moving backward; front-facing
    # ±15° when moving forward.
    REAR_ANGLE_CENTER_DEG = 180.0
    REAR_ANGLE_HALFWIDTH_DEG = 30.0
    FRONT_ANGLE_CENTER_DEG = 0.0
    FRONT_ANGLE_HALFWIDTH_DEG = 15.0


### Nav2 goal with obstacle sources overridden (camera / rear-of-lidar)
MOVE_LOCATION_IGNORE_OBSTACLES_SERVICE = "/navigation/move_location_ignore_obstacles"
LOCAL_COSTMAP_NODE = "/local_costmap/local_costmap"
GLOBAL_COSTMAP_NODE = "/global_costmap/global_costmap"
CAMERA_OBSTACLE_LAYER = "rgbd_obstacle_layer"
LASER_OBSTACLE_LAYER = "obstacle_layer"

### Scan masker. nav2_standard.yaml uses SCAN_MASKED_TOPIC as the
### obstacle_layer source; when the mask is off this is a pass-through of
### SCAN_TOPIC, so it is safe for regular navigation.
SCAN_MASKED_TOPIC = "/scan_masked"


class SCAN_MASK(Enum):
    REAR_CENTER_DEG = 180.0  # rear of robot when angle=0 is forward
    REAR_HALFWIDTH_DEG = 60.0  # ±60° around the rear direction is masked


### Initial pose topic
INITIAL_POSE_TOPIC = "/initialpose"

### Manual resume service (nav_central exposes this so the UI can unpause nav2+rtabmap)
RESUME_NAV_SERVICE = "/navigation/resume_nav"

### General Constants
MONITOR_RATE = 1.0
NO_TF_LIMIT = 2
NO_TOPICS_LIMIT = 2
TIMEOUT_REQUIREMENTS = 0.8
TIMEOUT_RTABMAP = 30.0
TIMEOUT_RTAB_SERVICE = 5.0
TIMEOUT_NAV2_LIFECYCLE = 10.0
RTAB_PAUSE_SERVICE = "/rtabmap/pause"
RTAB_RESUME_SERVICE = "/rtabmap/resume"
RTAB_CHECK_TOPIC = "/rtabmap/republish_node_data"
RTAB_MAPS_PATH = "/workspace/src/navigation/rtabmapdbs/"
RTAB_CONTAINER_NODE = "/rtabmap_container/_container/load_node"
NAV2_LIFECYCLE_SERVICE = "/lifecycle_manager_navigation/manage_nodes"
# ------------------------------------------------------------------


# Navigation Subtask Manager
# ------------------------------------------------------------------
class SUBTASK_MANAGER(Enum):
    SERVICE_TIMEOUT = 2.0
    AREAS_RETRIEVE_TIMEOUT = 2.0
    MOVE_DISTANCE_TIMEOUT = 60.0
    MOVE_UNTIL_OBJECT_TIMEOUT = 45.0
    MOVE_LOCATION_IGNORE_OBSTACLES_TIMEOUT = 120.0


# General constants
GOAL_TOPIC = "/navigate_to_pose"
SCAN_TOPIC = "/scan"
CAMERA_RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"
CAMERA_DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered"

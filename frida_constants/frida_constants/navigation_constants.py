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

### Initial pose topic
INITIAL_POSE_TOPIC = "/initialpose"

### Manual resume service (nav_central exposes this so the UI can unpause nav2+rtabmap)
RESUME_NAV_SERVICE = "/navigation/resume_nav"

### Table/shelf docking (perpendicular approach) — see table_docker.py
DOCK_SERVICE = "/navigation/dock_to_surface"        # std_srvs/Trigger: align + approach
UNDOCK_SERVICE = "/navigation/undock_from_surface"  # std_srvs/Trigger: back off so nav2 can plan
DOCK_PREVIEW_SERVICE = "/navigation/preview_dock"    # std_srvs/Trigger: detect + show markers, no motion
DOCKED_TOPIC = "/navigation/docked"                  # std_msgs/Bool (latched): currently docked?
POINT_CLOUD_TOPIC = "/point_cloud"                   # filtered ZED cloud also used by nav2

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


# Default docking offset (table_docker front_offset) used when a location enables
# approach but does not specify its own offset.
DEFAULT_DOCK_OFFSET = 0.16


def parse_location(value):
    """Parse an areas.json location entry (backward compatible).

    An entry is either:
      * a 7-element pose list [x, y, z, qx, qy, qz, qw]              (no docking), or
      * a dict {"pose": [7], "approach": bool, "offset": float}     (docking on).
    Returns (pose7, approach, offset); offset is None when unspecified.
    """
    if isinstance(value, dict):
        return value.get("pose"), bool(value.get("approach", False)), value.get("offset", None)
    return value, False, None


def make_location(pose, approach=False, offset=None):
    """Build an areas.json location entry. Stays a plain 7-list when docking is
    off (preserves the legacy format); becomes a dict only when approach/offset
    are set."""
    pose = list(pose)
    if not approach and offset is None:
        return pose
    entry = {"pose": pose, "approach": bool(approach)}
    if offset is not None:
        entry["offset"] = float(offset)
    return entry


# General constants
GOAL_TOPIC = "/navigate_to_pose"
SCAN_TOPIC = "/scan"
CAMERA_RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"
CAMERA_DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered"

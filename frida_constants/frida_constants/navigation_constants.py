from enum import Enum

# Navigation Central
# -----------------------------------------------------------------
###Check door service
CHECK_DOOR_SERVICE = "/navigation/is_door_open"


class DOOR_CHECK(Enum):
    TIMEOUT_SENSOR = 5.0
    TIMEOUT_TO_OPEN = 60.0  # Increase in case of required
    # Beam index window into LaserScan.ranges[] that points at the door.
    # Calibrated for the ~501-beam lidar (angle_increment ~0.01257 rad,
    # idx 250 = 0 deg / straight ahead). Robot faces the door head-on:
    # closed panel fills idx ~203-264 (~0.48 m); window inset to avoid frame edges.
    LIDAR_RANGE_MIN = 210
    LIDAR_RANGE_MAX = 255
    CHECKING_RATE = 0.5
    # Distance threshold (m): avg window reading above this -> door open.
    # Closed door reads ~0.48 m; open reads 2-5 m+ / inf. 2.0 sits safely between.
    DOOR_DISTANCE = 2.0


###Map areas service
AREAS_SERVICE = "/navigation/areas_json"

### Move to location service
MOVE_LOCATION_SERVICE = "/navigation/go_to_map_area"

### Dock to table/shelf service (nav_central -> table_docker), with desired offset
DOCK_TABLE_SERVICE = "/navigation/dock_table"
GOAL_NAV_ACTION_SERVER = "/navigate_to_pose"

### Point-based navigation services
GO_TO_POSE_SERVICE = "/navigation/go_to_pose"
GET_ROBOT_POSE_SERVICE = "/navigation/get_robot_pose"
APPROACH_POINT_SERVICE = "/navigation/approach_point"
GLOBAL_COSTMAP_TOPIC = "/global_costmap/costmap"

### Short relative base displacement (MoveRelative: dx/dy/dyaw in the current
### base frame). Direct cmd_vel closed loop on odom TF — bypasses Nav2 and the
### costmaps; for small deliberate sidesteps, not navigation.
MOVE_RELATIVE_SERVICE = "/navigation/move_relative"

### Washing-machine precision aligner (wall_aligner.py) — live-lidar servo,
### independent of table_docker: align perpendicular to the NEAREST straight
### segment the lidar sees in front (the machine's front panel, endpoints
### visible), then close to an EXACT perpendicular distance while staying
### square on the live fit. Direct cmd_vel, slow, cm-level.
WALL_ALIGN_SERVICE = "/navigation/wall_align"  # AlignToWall
WALL_CLOSE_SERVICE = "/navigation/wall_close"  # CloseToWall

### Toggle live obstacle marking (SetBool). data=False disables the lidar
### obstacle_layer + ZED rgbd_obstacle_layer on BOTH costmaps (static map still
### applies) so a bag/basket carried by the arm doesn't wall the robot in;
### data=True restores them. Costmaps are cleared on every toggle.
SET_OBSTACLE_AVOIDANCE_SERVICE = "/navigation/set_obstacle_avoidance"

### Initial pose topic
INITIAL_POSE_TOPIC = "/initialpose"

### Manual resume service (nav_central exposes this so the UI can unpause nav2+rtabmap)
RESUME_NAV_SERVICE = "/navigation/resume_nav"

### Path query service (distance between two map areas without moving)
NAV_QUERY_SERVICE = "/navigation/query_path"
COMPUTE_PATH_ACTION_SERVER = "/compute_path_to_pose"
TIMEOUT_NAV_QUERY = 10.0  # seconds to wait for ComputePathToPose result

### Table/shelf docking (perpendicular approach) — see table_docker.py
DOCK_SERVICE = "/navigation/dock_to_surface"  # std_srvs/Trigger: align + approach
UNDOCK_SERVICE = (
    "/navigation/undock_from_surface"  # std_srvs/Trigger: back off so nav2 can plan
)
DOCK_PREVIEW_SERVICE = (
    "/navigation/preview_dock"  # std_srvs/Trigger: detect + show markers, no motion
)
DOCKED_TOPIC = "/navigation/docked"  # std_msgs/Bool (latched): currently docked?
RETREAT_DISTANCE = 0.2  # m the base backs off on undock (table_docker retreat_distance)
POINT_CLOUD_TOPIC = "/point_cloud"  # filtered ZED cloud also used by nav2

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
    # Path query may resume nav2 (up to TIMEOUT_NAV2_LIFECYCLE), plan
    # (up to TIMEOUT_NAV_QUERY) and re-pause, so give it ample room
    NAV_QUERY_TIMEOUT = 30.0


# Follow person node constants
# -------------------------------------------------------------------
GOAL_UPDATE_TOPIC = "/goal_update"
FOLLOW_MODE_SERVICE = "/navigation/set_follow_mode"
FOLLOW_PERSON_NAV_SERVICE = "/navigation/follow_person"

# Timeouts for the follow-person orchestration
FOLLOW_MODE_SERVICE_TIMEOUT = 2.0  # Wait for /navigation/set_follow_mode
FOLLOW_GOAL_UPDATE_TIMEOUT = (
    2.0  # Wait for first /goal_update before seeding initial goal
)
FOLLOW_ACTION_SERVER_TIMEOUT = 3.0  # Wait for NavigateToPose action server

# Default docking offset (table_docker front_offset).
DEFAULT_DOCK_OFFSET = 0.16

# General constants
GOAL_TOPIC = "/navigate_to_pose"
SCAN_TOPIC = "/scan"
CAMERA_RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"
CAMERA_DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered"
MAP_TOPIC = "/map"

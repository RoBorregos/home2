from math import pi as PI

EEF_LINK_NAME = "link_eef"
EEF_CONTACT_LINKS = ["link_eef", "link_6"]

DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI

# xArm Constants
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
MOVEIT_MODE = 1
JOINT_VELOCITY_MODE = 4

XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
SET_JOINT_VELOCITY_SERVICE = "/manipulation/set_joint_velocity"

ALWAYS_SET_MODE = True
JOINTN_VELOCITY_MODE = 4

MOVE_JOINTS_ACTION_SERVER = "/manipulation/move_joints_action_server"
MOVE_TO_POSE_ACTION_SERVER = "/manipulation/move_to_pose_action_server"
GET_JOINT_SERVICE = "/manipulation/get_joints"
TOGGLE_SERVO_SERVICE = "/manipulation/toggle_servo"

PICK_VELOCITY = 0.15
PICK_ACCELERATION = 0.15
PICK_PLANNER = "RRTConnect"

# Pick
PICK_MOTION_ACTION_SERVER_NODE = "manipulation/pick_motion_server"
PICK_MOTION_ACTION_SERVER = "/manipulation/pick_motion_action_server"
CLUSTER_OBJECT_SERVICE = "/manipulation/cluster_object"
PICK_PERCEPTION_SERVICE = "/manipulation/pick_perception_service"
GRASP_DETECTION_SERVICE = "/manipulation/detect_grasps"
GRIPPER_SET_STATE_SERVICE = "/manipulation/gripper/set_state"
XARM_SET_DIGITAL_TGPIO_SERVICE = "/xarm/set_digital_tgpio"

# Place
PLACE_PERCEPTION_SERVICE = "/manipulation/place_perception_service"
HEATMAP_PLACE_SERVICE = "/manipulation/heatmap_place_service"
PLACE_POINT_DEBUG_TOPIC = "/manipulation/table_place_point_debug"
PLACE_MAX_DISTANCE = 0.85
PLACE_MOTION_ACTION_SERVER = "/manipulation/place_motion_action_server"

MANIPULATION_ACTION_SERVER = "/manipulation/manipulation_action_server"

ATTACH_COLLISION_OBJECT_SERVICE = "/manipulation/attach_collision_object"
ADD_COLLISION_OBJECT_SERVICE = "/manipulation/add_collision_objects"
GET_COLLISION_OBJECTS_SERVICE = "/manipulation/get_collision_objects"
REMOVE_COLLISION_OBJECT_SERVICE = "/manipulation/remove_collision_object"
PICK_OBJECT_NAMESPACE = "frida_pick_object_"
PLANE_NAMESPACE = "plane"
PLANE_OBJECT_COLLISION_TOLERANCE = (
    0.025  # Tolerance to delete collision objects if they are too close to the plane
)

DEBUG_POSE_GOAL_TOPIC = "/manipulation/debug_pose_goal"
GET_JOINT_TOPIC = "/manipulation/get_joints"

TOGGLE_SERVO_TOPIC = "/manipulation/toggle_servo"

ZED_POINT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"

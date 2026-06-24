from math import pi as PI

EEF_LINK_NAME = "link_eef"
EEF_CONTACT_LINKS = ["link_eef", "link_6", "gripper", "left_finger", "right_finger"]

DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI

# xArm Constants
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_SETMODE_MOVEIT_SERVICE = "/manipulation/xarm/set_moveit_mode"
XARM_ROBOT_STATES_TOPIC = "/xarm/robot_states"
XARM_CLEAN_ERROR_SERVICE = "/xarm/clean_error"
XARM_MOTION_ENABLE_SERVICE = "/xarm/motion_enable"
XARM_SET_SERVO_ANGLE_SERVICE = "/xarm/set_servo_angle"
XARM_POSITION_MODE = 0  # mode 0: direct position control, bypasses MoveIt
MOVEIT_MODE = 1
JOINT_VELOCITY_MODE = 4

# xArm arm state values (from xArm SDK)
XARM_STATE_READY = 1
XARM_STATE_MOVING = 2
XARM_STATE_PAUSED = 3
XARM_STATE_STOPPED = 4  # error / e-stop

XARM_ALL_JOINTS_ID = 8  # pass to motion_enable to target all joints

FACE_RECOGNITION_LIFETIME = 0.1
FOLLOW_FACE_SPEED = 1.5
FOLLOW_FACE_TOLERANCE = 0.15

XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
SET_JOINT_VELOCITY_SERVICE = "/manipulation/set_joint_velocity"

ALWAYS_SET_MODE = False
JOINTN_VELOCITY_MODE = 4

MOVE_JOINTS_ACTION_SERVER = "/manipulation/move_joints_action_server"
ESTOP_TOPIC = "/manipulation/estop"
MOVE_TO_POSE_ACTION_SERVER = "/manipulation/move_to_pose_action_server"
GET_JOINT_SERVICE = "/manipulation/get_joints"
TOGGLE_SERVO_SERVICE = "/manipulation/toggle_servo"

MIN_CONFIGURATION_DISTANCE_TRESHOLD = 0.01
PICK_VELOCITY = 0.5
PICK_ACCELERATION = 0.15
PICK_PLANNER = "RRTConnect"

# Scan
SCAN_ANGLE_VERTICAL = 30.0  # degrees
SCAN_ANGLE_HORIZONTAL = 30.0  # degrees

ARM_HIGHEST_0_0_HEIGHT = 1.45
ARM_LOWEST_0_0_HEIGHT = 0.95

# Pick
PICK_MOTION_ACTION_SERVER_NODE = "manipulation/pick_motion_server"
PICK_MOTION_ACTION_SERVER = "/manipulation/pick_motion_action_server"
GO_TO_HAND_ACTION_SERVER = "/manipulation/go_to_hand_action_server"
ALIGN_ARM_TO_CENTROID_ACTION_SERVER = (
    "/manipulation/align_arm_to_centroid_action_server"
)
CLUSTER_OBJECT_SERVICE = "/manipulation/cluster_object"
PICK_PERCEPTION_SERVICE = "/manipulation/pick_perception_service"
GRASP_DETECTION_SERVICE = "/manipulation/detect_grasps"
GRIPPER_SET_STATE_SERVICE = "/manipulation/gripper/set_state"
XARM_SET_DIGITAL_TGPIO_SERVICE = "/xarm/set_tgpio_digital"
GRIPPER_GRASP_STATE_TOPIC = "/gripper/grasp_state"
SAFETY_HEIGHT = 0.05
PICK_MIN_HEIGHT = 0.04
CUTLERY_PICK_MIN_HEIGHT = 0.002
CUTLERY_NAMES = ["fork", "knife", "spoon", "cutlery"]
# Objects picked with the flat-grasp estimator
FLAT_OBJECT_NAMES = CUTLERY_NAMES + ["plate", "red_plate"]
POUR_OBJECT_NAMES = {"blue_cereal_box", "cereal", "chocomilk_box", "milk"}
GRASP_LINK_FRAME = "gripper_grasp_frame"

# Rim pick
RIM_NAMES = ["basket", "laundry_basket"]
RIM_PRE_GRASP_HEIGHT = 0.10
RIM_GRASP_Z_TWEAK = -0.05  # m: target ~3 cm below rim top so fingers straddle the wall
RIM_DESCENT_SPEED = 20.0  # mm/s
RIM_DESCENT_DISTANCE = RIM_PRE_GRASP_HEIGHT - RIM_GRASP_Z_TWEAK

# Peak pick
PEAK_NAMES = ["clothes"]  # task-level object_name aliases
PEAK_PRE_GRASP_HEIGHT = 0.05  # m
PEAK_DESCENT_SPEED = RIM_DESCENT_SPEED  # mm/s (reuse rim close-loop)

# Fixed-distance cartesian move service
FIXED_DISTANCE_MOVE_SERVICE = "/manipulation/fixed_distance_move"
# Ascent used by the clothes pick to exit the basket workspace
CLOTHES_BASKET_EXIT_HEIGHT = 0.1  # m

# Place
PLACE_PERCEPTION_SERVICE = "/manipulation/place_perception_service"
HEATMAP_PLACE_SERVICE = "/manipulation/heatmap_place_service"
PLACE_POINT_DEBUG_TOPIC = "/manipulation/table_place_point_debug"
PICK_MAX_DISTANCE = 1.0
PLACE_MAX_DISTANCE = 0.8
PLACE_MOTION_ACTION_SERVER = "/manipulation/place_motion_action_server"

# Special request for place
CLOSE_BY_MAX_DISTANCE = 0.3  # Maximum distance for close-by heatmap generation

MANIPULATION_ACTION_SERVER = "/manipulation/manipulation_action_server"
MANIPULATION_ENSURE_ARM_READY_SERVICE = "/manipulation/ensure_arm_ready"

ATTACH_COLLISION_OBJECT_SERVICE = "/manipulation/attach_collision_object"
ADD_COLLISION_OBJECT_SERVICE = "/manipulation/add_collision_objects"
GET_COLLISION_OBJECTS_SERVICE = "/manipulation/get_collision_objects"
REMOVE_COLLISION_OBJECT_SERVICE = "/manipulation/remove_collision_object"

PICK_OBJECT_NAMESPACE = "frida_pick_object_"
PLANE_NAMESPACE = "plane"
PLANE_OBJECT_COLLISION_TOLERANCE = (
    0.025  # Tolerance to delete collision objects if they are too close to the plane
)

# Pour
POUR_MOTION_ACTION_SERVER = "/manipulation/pour_motion_action_server"
POUR_VELOCITY = 0.5
POUR_ACCELERATION = 0.15

SHELF_POSITION_PREPLACE_POSE = -0.25

DEBUG_POSE_GOAL_TOPIC = "/manipulation/debug_pose_goal"
GET_JOINT_TOPIC = "/manipulation/get_joints"

TOGGLE_SERVO_TOPIC = "/manipulation/toggle_servo"

ZED_POINT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"

# Similar to what you have on stare poses, gripper looking front, camera looking front-down
AIM_STRAIGHT_FRONT_QUAT = [0.650, -0.290, 0.636, -0.299]

# Shelf reachability clamp: max_x = max(SHELF_MIN_REACH, SHELF_REACH_BASE - SHELF_REACH_SLOPE * z)
SHELF_MIN_REACH = 0.40
SHELF_REACH_BASE = 0.75
SHELF_REACH_SLOPE = 0.25

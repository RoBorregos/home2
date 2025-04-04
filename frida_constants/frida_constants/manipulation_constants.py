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

PICK_MOTION_ACTION_SERVER_NODE = "manipulation/pick_motion_server"
PICK_MOTION_ACTION_SERVER = "/manipulation/pick_motion_action_server"
CLUSTER_OBJECT_SERVICE = "/manipulation/cluster_object"
PERCEPTION_SERVICE = "/manipulation/perception_service"
GRASP_DETECTION_SERVICE = "/manipulation/detect_grasps"

PICK_ACTION_SERVER = "/manipulation/pick_action_server"

ATTACH_COLLISION_OBJECT_SERVICE = "/manipulation/attach_collision_object"
ADD_COLLISION_OBJECT_SERVICE = "/manipulation/add_collision_objects"
GET_COLLISION_OBJECTS_SERVICE = "/manipulation/get_collision_objects"
REMOVE_COLLISION_OBJECT_SERVICE = "/manipulation/remove_collision_object"
PICK_OBJECT_NAMESPACE = "frida_pick_object_"
PLANE_NAMESPACE = "plane"
PLANE_OBJECT_COLLISION_TOLERANCE = 0.025 # Tolerance to delete collision objects if they are too close to the plane

DEBUG_POSE_GOAL_TOPIC = "/manipulation/debug_pose_goal"
GET_JOINT_TOPIC = "/manipulation/get_joints"

TOGGLE_SERVO_TOPIC = "/manipulation/toggle_servo"

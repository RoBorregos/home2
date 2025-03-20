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

PICK_VELOCITY = 0.15
PICK_ACCELERATION = 0.15
PICK_PLANNER = "RRTConnect"

PICK_MOTION_ACTION_SERVER_NODE = "manipulation/pick_motion_server"
PICK_MOTION_ACTION_SERVER = "/manipulation/pick_motion_action_server"
CLUSTER_OBJECT_SERVICE = "/manip/cluster_object"

PICK_ACTION_SERVER = "/manipulation/pick_action_server"

ATTACH_COLLISION_OBJECT_SERVICE = "/manipulation/attach_collision_object"
GET_COLLISION_OBJECTS_SERVICE = "/manipulation/get_collision_objects"
PICK_OBJECT_NAMESPACE = "frida_pick_object_"

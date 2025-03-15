from math import pi as PI

DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI
# xArm Constants
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
MOVEIT_MODE = 1
JOINTN_VELOCITY_MODE = 4

MOVE_JOINTS_ACTION_SERVER = "/manipulation/move_joints_action_server"
MOVE_TO_POSE_ACTION_SERVER = "/manipulation/move_to_pose_action_server"

PICK_VELOCITY = 0.15
PICK_ACCELERATION = 0.15
PICK_PLANNER = "RRTConnect"
PICK_ACTION_SERVER_NODE = "manipulation/pick_server"
PICK_ACTION_SERVER = "/manipulation/pick_action_server"

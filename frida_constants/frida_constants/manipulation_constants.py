from math import pi as PI

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

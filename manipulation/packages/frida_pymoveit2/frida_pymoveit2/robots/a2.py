import math
from typing import List

from frida_pymoveit2.robots.xarm6 import (  # noqa: F401
    CLOSED_GRIPPER_JOINT_POSITIONS,
    MOVE_GROUP_GRIPPER,
    OPEN_GRIPPER_JOINT_POSITIONS,
    camera_frame_name,
    end_effector_name,
    gripper_joint_names,
    left_camera_frame_name,
    left_camera_optical_frame_name,
    right_camera_frame_name,
    right_camera_optical_frame_name,
)

# Agility A2 (OpenArm v1.0) right arm; it carries the FRIDA gripper and ZED on link_eef
MOVE_GROUP_ARM: str = "a2_arm"

# Position limits per joint (radians), right arm of openarm_description v1.0 (joint2 shifted by pi/2)
JOINT_POSITION_LIMITS = {
    "openarm_right_joint1": (-1.396263, 3.490659),
    "openarm_right_joint2": (-1.745329 + math.pi / 2, 1.745329 + math.pi / 2),
    "openarm_right_joint3": (-1.570796, 1.570796),
    "openarm_right_joint4": (0.0, 2.443461),
    "openarm_right_joint5": (-1.570796, 1.570796),
    "openarm_right_joint6": (-0.785398, 0.785398),
    "openarm_right_joint7": (-1.570796, 1.570796),
}


def joint_names(prefix: str = "") -> List[str]:
    return [prefix + f"openarm_right_joint{i}" for i in range(1, 8)]


def base_link_name(prefix: str = "") -> str:
    return prefix + "a2_shoulder_mount"

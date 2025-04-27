from typing import List

MOVE_GROUP_ARM: str = "xarm6"
MOVE_GROUP_GRIPPER: str = "xarm_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.85]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "link_eef"


def camera_frame_name(prefix: str = "") -> str:
    return prefix + "zed_camera_link"


def left_camera_frame_name(prefix: str = "") -> str:
    return prefix + "zed_left_camera_frame"


def left_camera_optical_frame_name(prefix: str = "") -> str:
    return prefix + "zed_left_camera_optical_frame"


def right_camera_frame_name(prefix: str = "") -> str:
    return prefix + "zed_right_camera_frame"


def right_camera_optical_frame_name(prefix: str = "") -> str:
    return prefix + "zed_right_camera_optical_frame"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "drive_joint",
    ]

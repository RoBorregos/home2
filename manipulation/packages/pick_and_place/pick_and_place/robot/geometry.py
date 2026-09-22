"""Pose maths shared by the manipulation pipelines."""

import copy

import numpy as np
from geometry_msgs.msg import PoseStamped
from transforms3d.quaternions import quat2mat


def offset_z(pose: PoseStamped, delta_z: float) -> PoseStamped:
    """Copy ``pose`` shifted by ``delta_z`` along world Z.

    Pre-grasp heights and descent tweaks are vertical in the world frame, not
    along the gripper's approach axis.
    """
    shifted = copy.deepcopy(pose)
    shifted.pose.position.z += delta_z
    return shifted


def approach_axis(pose: PoseStamped) -> np.ndarray:
    """Unit vector of the gripper's local Z axis (its approach direction)."""
    orientation = pose.pose.orientation
    # quat2mat expects scalar-first [w, x, y, z].
    rotation = quat2mat([orientation.w, orientation.x, orientation.y, orientation.z])
    return rotation[:, 2]


def offset_along_approach(pose: PoseStamped, distance: float) -> PoseStamped:
    """Copy ``pose`` translated ``distance`` along its own approach axis.

    Used to back a perceived grasp point off to the EE link frame; ``distance``
    is negative for the tip offsets, which move the arm away from the object.
    """
    shifted = copy.deepcopy(pose)
    position = np.array(
        [shifted.pose.position.x, shifted.pose.position.y, shifted.pose.position.z]
    )
    position = position + approach_axis(pose) * distance
    shifted.pose.position.x = float(position[0])
    shifted.pose.position.y = float(position[1])
    shifted.pose.position.z = float(position[2])
    return shifted


def set_quaternion(pose: PoseStamped, quat) -> PoseStamped:
    """Set ``pose``'s orientation from an [x, y, z, w] quaternion, in place."""
    pose.pose.orientation.x = float(quat[0])
    pose.pose.orientation.y = float(quat[1])
    pose.pose.orientation.z = float(quat[2])
    pose.pose.orientation.w = float(quat[3])
    return pose

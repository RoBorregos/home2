"""Pose maths shared by the manipulation pipelines."""

import copy

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from transforms3d.quaternions import quat2mat

TOP_DOWN_AXIS_Z = -0.99


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


def is_tilted(pose: PoseStamped) -> bool:
    """True when the gripper does not point straight down."""
    return approach_axis(pose)[2] > TOP_DOWN_AXIS_Z


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


def inset_extent(extent, front: float, back: float, side: float, ahead: float) -> tuple:
    """(min_x, max_x, min_y, max_y) of a point held ``ahead`` of it inside ``extent``.

    ``extent`` is (front_x, back_x, y_min, y_max); the held point stays ``front``,
    ``back`` and ``side`` inside its edges.
    """
    front_x, back_x, y_min, y_max = extent
    min_x = front_x + front - ahead
    min_y = y_min + side
    return min_x, max(min_x, back_x - back - ahead), min_y, max(min_y, y_max - side)


def keep_up_quat(held_quat, aim_quat, grasp_yaw_from_eef: float):
    """link_eef orientation along ``aim_quat``'s approach, rolled so the object keeps
    the "up" it had when grasped at ``held_quat``; None after a top-down grasp."""
    up = Rotation.from_quat(held_quat).inv().apply([0.0, 0.0, 1.0])
    up[2] = 0.0  # only "up" across the approach (grasp local z) can be set by the roll
    if np.linalg.norm(up) < 0.3:
        return None
    # A roll about local z keeps the approach, so it is the same for link_eef and the grasp frame.
    approach = Rotation.from_quat(aim_quat).apply([0.0, 0.0, 1.0])
    world_up = np.array([0.0, 0.0, 1.0]) - approach[2] * approach
    # Grasp frame: local z -> approach, local "up" -> world up.
    local = np.column_stack((up, np.cross([0.0, 0.0, 1.0], up), [0.0, 0.0, 1.0]))
    world = np.column_stack((world_up, np.cross(approach, world_up), approach))
    local /= np.linalg.norm(local, axis=0)
    world /= np.linalg.norm(world, axis=0)
    grasp = Rotation.from_matrix(world @ local.T)
    return list((grasp * Rotation.from_euler("z", -grasp_yaw_from_eef)).as_quat())

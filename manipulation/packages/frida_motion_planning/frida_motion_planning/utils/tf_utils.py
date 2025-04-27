import math
from typing import Union
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from numpy.typing import NDArray


def quat_to_rpy(quat):
    """
    Convert a quaternion to roll, pitch, and yaw (in radians).

    Args:
        quat (iterable): A quaternion represented as [x, y, z, w].

    Returns:
        tuple: (roll, pitch, yaw) in radians.
    """
    x, y, z, w = quat

    # Compute roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Compute pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Compute yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def look_at(
    source_pose: Union[PointStamped, PoseStamped],
    target_pose: Union[PointStamped, PoseStamped],
    world_up: NDArray = np.array([0, 0, 1]),
) -> PoseStamped:
    """
    Receives two points, a source and a target.
    Returns the pose that represents the correct orientation so source looks at target, with its original position.
    Looks at means the x-axis of the source points towards the target point.
    Args:
        source (PointStamped|PoseStamped): The source point.
        target (PointStamped|PoseStamped): The target point.
        world_up (NDArray): The up vector of the world. Default is [0, 0, 1], it will mostly be like this
    """

    if isinstance(source_pose, PointStamped):
        source_pose_ = PoseStamped()
        source_pose_.header = source_pose.header
        source_pose_.pose.position.x = source_pose.point.x
        source_pose_.pose.position.y = source_pose.point.y
        source_pose_.pose.position.z = source_pose.point.z
        source_pose_.pose.orientation.w = 1.0
        source_pose_.pose.orientation.x = 0.0
        source_pose_.pose.orientation.y = 0.0
        source_pose_.pose.orientation.z = 0.0
    else:
        source_pose_ = source_pose

    if isinstance(target_pose, PointStamped):
        target_pose_ = PoseStamped()
        target_pose_.header = target_pose.header
        target_pose_.pose.position.x = target_pose.point.x
        target_pose_.pose.position.y = target_pose.point.y
        target_pose_.pose.position.z = target_pose.point.z
        target_pose_.pose.orientation.w = 1.0
        target_pose_.pose.orientation.x = 0.0
        target_pose_.pose.orientation.y = 0.0
        target_pose_.pose.orientation.z = 0.0
    else:
        target_pose_ = target_pose

    # Ensure the source and target poses have the same frame_id -> Will NOT run transform here
    if source_pose_.header.frame_id != target_pose_.header.frame_id:
        raise ValueError("Source and target poses must have the same frame_id")

    source_position = np.array(
        [
            source_pose_.pose.position.x,
            source_pose_.pose.position.y,
            source_pose_.pose.position.z,
        ]
    )
    target_position = np.array(
        [
            target_pose_.pose.position.x,
            target_pose_.pose.position.y,
            target_pose_.pose.position.z,
        ]
    )

    # vector (x axis front)
    forward = target_position - source_position
    forward = forward / np.linalg.norm(forward)
    right = np.cross(world_up, forward)
    right = right / np.linalg.norm(right)
    up = np.cross(forward, right)
    up = up / np.linalg.norm(up)

    # R matrix
    rotation_matrix = np.array(
        [
            forward,  # Camera's x-axis aligned with forward direction
            right,  # Camera's y-axis aligned with the 'right' direction
            up,  # Camera's z-axis aligned with the 'up' direction
        ]
    ).T

    rotation_scipy = R.from_matrix(rotation_matrix)
    quat = rotation_scipy.as_quat()

    result_pose = PoseStamped()
    result_pose.header = source_pose_.header
    result_pose.pose.position.x = source_pose_.pose.position.x
    result_pose.pose.position.y = source_pose_.pose.position.y
    result_pose.pose.position.z = source_pose_.pose.position.z
    result_pose.pose.orientation.x = quat[0]
    result_pose.pose.orientation.y = quat[1]
    result_pose.pose.orientation.z = quat[2]
    result_pose.pose.orientation.w = quat[3]

    return result_pose

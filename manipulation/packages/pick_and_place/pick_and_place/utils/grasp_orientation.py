"""Frontal grasp check for shelf compartment picks (no ROS, unit-testable offline).
Top-down hits the compartment ceiling; a frontal grasp reaches in and clears it."""

from scipy.spatial.transform import Rotation as R

FRONTAL_GRASP_MAX_VERTICAL = 0.5  # |approach_z|; 0.5 ~ within 60 deg of horizontal


def is_frontal_grasp(
    quat_xyzw, max_vertical: float = FRONTAL_GRASP_MAX_VERTICAL
) -> bool:
    """True if the grasp approaches roughly horizontally (front grasp), not top-down."""
    rot = R.from_quat([quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]])
    approach_z = float(rot.apply([0.0, 0.0, 1.0])[2])
    return abs(approach_z) <= max_vertical

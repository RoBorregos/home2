"Point-cloud <-> GIGA conversion helpers"

from typing import Optional, Tuple

import numpy as np

_NEIGHBOR_OFFSETS = (
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
)

_GIGA_WORKSPACE_SIZE_M = 0.3
_GIGA_VOXEL_RESOLUTION = 40

_OUTSIDE_SDF_VALUE = 1.0
_SURFACE_SDF_VALUE = 0.0
_NEAR_SURFACE_SDF_VALUE = 0.25

_MARKER_BASE_SIZE_M = (0.1, 0.1, 0.05)
_MARKER_FINGER_SIZE_M = (0.02, 0.02, 0.17)
_MARKER_HUE_RED_TO_GREEN = 0.333
_MARKER_MIN_COLOR_CHANNEL = 0.1
_MARKER_BASE_DARKEN = 0.5
_MARKER_ALPHA = 0.6
_NUM_FINGERS = 2
_FINGER_SIDE_SIGNS = (1.0, -1.0)


def points_to_tsdf_grid(
    points: np.ndarray,
    size: float = _GIGA_WORKSPACE_SIZE_M,
    resolution: int = _GIGA_VOXEL_RESOLUTION,
    origin: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Voxelize a point cloud into a GIGA-shaped (approximate) TSDF grid."""
    resolution = int(resolution)
    voxel_size = size / resolution
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)

    if origin is None:
        centroid = points.mean(axis=0) if len(points) else np.zeros(3)
        origin = centroid - size / 2.0
    origin = np.asarray(origin, dtype=np.float64).reshape(3)

    grid = np.full(
        (1, resolution, resolution, resolution), _OUTSIDE_SDF_VALUE, dtype=np.float32
    )

    if len(points) == 0:
        return grid, origin

    idx = np.floor((points - origin) / voxel_size).astype(np.int64)
    in_bounds = np.all((idx >= 0) & (idx < resolution), axis=1)
    idx = idx[in_bounds]
    if len(idx) == 0:
        return grid, origin

    grid[0, idx[:, 0], idx[:, 1], idx[:, 2]] = _SURFACE_SDF_VALUE
    for dx, dy, dz in _NEIGHBOR_OFFSETS:
        n = idx + np.array([dx, dy, dz])
        n_in_bounds = np.all((n >= 0) & (n < resolution), axis=1)
        n = n[n_in_bounds]
        if len(n):
            current = grid[0, n[:, 0], n[:, 1], n[:, 2]]
            grid[0, n[:, 0], n[:, 1], n[:, 2]] = np.minimum(
                current, _NEAR_SURFACE_SDF_VALUE
            )

    return grid, origin


def giga_grasp_to_pose_stamped(grasp, origin: np.ndarray, frame_id: str, stamp):
    """Convert a GIGA Grasp into a geometry_msgs/PoseStamped. Orientation convention unverified."""
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp

    position = np.asarray(origin, dtype=np.float64) + grasp.pose.translation
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])

    qx, qy, qz, qw = grasp.pose.rotation.as_quat()
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)

    return pose


def build_gripper_markers(poses, scores, widths, ns_prefix="giga_grasp"):
    """Build a MarkerArray visualizing each grasp as a simple gripper shape."""
    import colorsys
    import copy

    from scipy.spatial.transform import Rotation
    from visualization_msgs.msg import Marker, MarkerArray

    markers = MarkerArray()

    for i, (pose, score, width) in enumerate(zip(poses, scores, widths)):
        hue = _MARKER_HUE_RED_TO_GREEN * max(0.0, min(1.0, float(score)))
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        r = max(r, _MARKER_MIN_COLOR_CHANNEL)
        g = max(g, _MARKER_MIN_COLOR_CHANNEL)
        b = max(b, _MARKER_MIN_COLOR_CHANNEL)
        ns = f"{ns_prefix}_{i}"

        base = Marker()
        base.header = pose.header
        base.ns = ns
        base.id = 0
        base.type = Marker.CUBE
        base.action = Marker.ADD
        base.pose = pose.pose
        base.scale.x, base.scale.y, base.scale.z = _MARKER_BASE_SIZE_M
        base.color.r, base.color.g, base.color.b, base.color.a = (
            r * _MARKER_BASE_DARKEN,
            g * _MARKER_BASE_DARKEN,
            b * _MARKER_BASE_DARKEN,
            _MARKER_ALPHA,
        )
        markers.markers.append(base)

        q = pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        for finger_i in range(_NUM_FINGERS):
            x_offset = _FINGER_SIDE_SIGNS[finger_i] * (float(width) / 2.0)
            offset_world = rot.apply([x_offset, 0.0, 0.0])
            finger = Marker()
            finger.header = pose.header
            finger.ns = ns
            finger.id = finger_i + 1
            finger.type = Marker.CUBE
            finger.action = Marker.ADD
            finger.pose = copy.deepcopy(pose.pose)
            finger.pose.position.x += float(offset_world[0])
            finger.pose.position.y += float(offset_world[1])
            finger.pose.position.z += float(offset_world[2])
            finger.scale.x, finger.scale.y, finger.scale.z = _MARKER_FINGER_SIZE_M
            finger.color.r, finger.color.g, finger.color.b, finger.color.a = (
                r,
                g,
                b,
                _MARKER_ALPHA,
            )
            markers.markers.append(finger)

    return markers
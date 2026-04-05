"""Symmetry-based point cloud completion for partial single-view clusters.

Objects seen from one viewpoint (e.g. ZED camera) only have points on the
visible side. This module reflects the visible points across the estimated
symmetry plane so that GPD can sample grasps on the full surface.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# --- tunables ---------------------------------------------------------------
MIN_POINTS_FOR_COMPLETION = 20  # skip if cluster is too small
MIN_HEIGHT_FOR_COMPLETION = 0.02  # 2 cm – flat objects don't need completion
VOXEL_LEAF_SIZE = 0.005  # 5 mm voxel grid for dedup after merging

# Approximate camera position in base_link at table_stare.
# Must match camera_position in GPD config.
DEFAULT_CAMERA_POSITION = np.array([0.0, 0.06, 1.17])


def complete_point_cloud_symmetry(
    points: np.ndarray,
    plane_z: float | None = None,
    camera_position: np.ndarray = DEFAULT_CAMERA_POSITION,
) -> np.ndarray:
    """Return *new* points (reflected only) that complete the partial cloud.

    Uses the direction from the camera to the cluster centroid to determine
    which side is visible and which side needs completion.  The symmetry plane
    is placed at the deepest visible extent (farthest from camera), and the
    visible-side points are reflected beyond it to create the hidden back
    surface.

    Args:
        points: (N, 3) array of the visible points (in base_link frame).
        plane_z: Z coordinate of the support surface (table). Reflected points
                 below this value are discarded.  Pass ``None`` to skip.
        camera_position: (3,) camera position in the same frame as *points*.

    Returns:
        (M, 3) array of reflected points to *append* to the original cloud.
        May be empty if reflection produced nothing useful.
    """
    centroid = points.mean(axis=0)
    centered = points - centroid

    # PCA – eigenvalues sorted ascending by np.linalg.eigh
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Smallest-eigenvalue axis ≈ viewing direction for a partial cloud.
    symmetry_normal = eigenvectors[:, 0]

    # Orient the normal to point FROM camera TOWARD object (the actual
    # viewing direction).  Using camera_position instead of the origin
    # is critical when the camera is eye-in-hand above the workspace.
    view_dir = centroid - camera_position
    norm = np.linalg.norm(view_dir)
    if norm > 1e-6:
        view_dir /= norm
    if np.dot(symmetry_normal, view_dir) < 0:
        symmetry_normal = -symmetry_normal

    # After orientation: positive projections = farther from camera,
    # negative = closer to camera.
    projections = centered @ symmetry_normal
    max_proj = projections.max()  # deepest visible extent ≈ object centre

    # Reflect every point through the plane at max_proj.
    reflected = centered + 2.0 * np.outer(max_proj - projections, symmetry_normal)

    # Keep only reflected points that land beyond the original cloud.
    reflected_proj = reflected @ symmetry_normal
    mask = reflected_proj > max_proj
    new_points = reflected[mask] + centroid if mask.any() else np.empty((0, 3))

    # Discard reflected points that fall below the table plane.
    if plane_z is not None and new_points.size > 0:
        new_points = new_points[new_points[:, 2] >= plane_z]

    return new_points


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    """Fast numpy-only voxel-grid downsampling."""
    if points.size == 0:
        return points
    indices = np.floor(points / leaf).astype(np.int64)
    _, unique_idx = np.unique(indices, axis=0, return_index=True)
    return points[unique_idx]


def complete_cluster(
    cloud_msg: PointCloud2,
    enable: bool = True,
    plane_z: float | None = None,
    camera_position: np.ndarray = DEFAULT_CAMERA_POSITION,
    logger=None,
) -> PointCloud2:
    """Symmetry-complete a PointCloud2 cluster and return a new message.

    If completion is disabled, the cluster is too small, or anything goes
    wrong, the *original* message is returned unchanged (safe fallback).

    Args:
        cloud_msg: The partial object cluster from perception.
        enable: Master switch – ``False`` skips completion entirely.
        plane_z: Z of the table surface (in the same frame as the cloud).
        camera_position: Camera position in the cloud's frame (base_link).
        logger: Optional ROS logger for info/warning messages.
    """
    if not enable:
        return cloud_msg

    try:
        gen = point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.array([list(p) for p in gen], dtype=np.float64)

        if len(points) < MIN_POINTS_FOR_COMPLETION:
            if logger:
                logger.info(
                    f"Point cloud completion: skipped (only {len(points)} points, "
                    f"need >= {MIN_POINTS_FOR_COMPLETION})"
                )
            return cloud_msg

        height = points[:, 2].max() - points[:, 2].min()
        if height < MIN_HEIGHT_FOR_COMPLETION:
            if logger:
                logger.info(
                    f"Point cloud completion: skipped (height {height:.3f} m < "
                    f"{MIN_HEIGHT_FOR_COMPLETION} m, flat object)"
                )
            return cloud_msg

        new_points = complete_point_cloud_symmetry(
            points, plane_z=plane_z, camera_position=camera_position
        )

        if new_points.size == 0:
            if logger:
                logger.info("Point cloud completion: no reflected points generated")
            return cloud_msg

        # Downsample only the reflected points to avoid removing originals
        new_points = voxel_downsample(new_points, VOXEL_LEAF_SIZE)
        completed = np.vstack([points, new_points])

        n_original = len(points)
        n_added = len(new_points)
        n_completed = n_original + n_added

        if logger:
            logger.info(
                f"Point cloud completion: {n_original} -> {n_completed} points "
                f"(+{n_added} reflected)"
            )

        # NOTE: creates XYZ-only cloud (no RGB).  GPD's 15-channel mode
        # uses only geometric features so this is fine.
        header = Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = cloud_msg.header.frame_id
        return point_cloud2.create_cloud_xyz32(header, completed.tolist())

    except Exception as e:
        if logger:
            logger.warn(f"Point cloud completion failed ({e}), using original cloud")
        return cloud_msg

"""Dense point cloud extraction from ZED depth image using detection bounding box.

Crops the raw ZED depth image to the 2D detection bbox, deprojects valid pixels
to 3D, isolates the object from the table using adaptive depth segmentation,
and returns a clean PointCloud2 ready for GPD.
"""

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# Margin (pixels) around the detection bbox
BBOX_MARGIN_PX = 10
# Depth range (metres) — reject anything outside
MIN_DEPTH = 0.15
MAX_DEPTH = 1.5
# Minimum depth band (metres) — floor for small objects
MIN_DEPTH_BAND = 0.04
# Maximum depth band (metres) — cap to avoid including background
MAX_DEPTH_BAND = 0.15
# Voxel leaf for the output cloud (metres)
OUTPUT_VOXEL_LEAF = 0.006
# Hard cap on output points
MAX_OUTPUT_POINTS = 400


def _voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    if pts.shape[0] == 0:
        return pts
    idx = np.floor(pts / leaf).astype(np.int64)
    _, uni = np.unique(idx, axis=0, return_index=True)
    return pts[uni]


def make_dense_cluster(
    depth_msg: Image,
    camera_info: CameraInfo,
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    tf_matrix: np.ndarray | None = None,
    table_z: float | None = None,
) -> PointCloud2 | None:
    """Build a dense, clean PointCloud2 from a depth-image bbox crop.

    Pipeline: crop → adaptive depth segmentation → deproject → TF →
    table filter → voxel downsample → PointCloud2.

    Args:
        table_z: Z height of the table surface in base_link frame.
                 Points at or below this height are removed.

    Returns a PointCloud2 in base_link, or ``None`` on failure.
    """
    # --- Decode depth image without cv_bridge overhead ----------------------
    if depth_msg.encoding != "32FC1":
        return None
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
        depth_msg.height, depth_msg.width
    )
    h, w = depth.shape

    # --- Normalised bbox → pixel coords with margin ------------------------
    u0 = max(0, int(xmin * w) - BBOX_MARGIN_PX)
    u1 = min(w, int(xmax * w) + BBOX_MARGIN_PX)
    v0 = max(0, int(ymin * h) - BBOX_MARGIN_PX)
    v1 = min(h, int(ymax * h) + BBOX_MARGIN_PX)
    if u1 <= u0 or v1 <= v0:
        return None

    crop = depth[v0:v1, u0:u1]
    valid = np.isfinite(crop) & (crop > MIN_DEPTH) & (crop < MAX_DEPTH)
    if not valid.any():
        return None

    # --- Depth segmentation -------------------------------------------------
    # Use the 10th percentile as the nearest object surface and a fixed
    # band that captures tall objects.  The table_z filter (in base_link
    # after TF) removes any table points that leak through.
    valid_depths = crop[valid]
    near_depth = np.percentile(valid_depths, 10)

    depth_mask_2d = valid & (crop >= near_depth - 0.01) & (
        crop <= near_depth + MAX_DEPTH_BAND
    )
    if not depth_mask_2d.any():
        return None

    # --- Deproject only the object pixels to 3D (vectorised) ----------------
    uu, vv = np.meshgrid(
        np.arange(u0, u1, dtype=np.float64),
        np.arange(v0, v1, dtype=np.float64),
    )
    d = crop[depth_mask_2d].astype(np.float64)
    u = uu[depth_mask_2d]
    v = vv[depth_mask_2d]

    fx, fy = camera_info.k[0], camera_info.k[4]
    cx, cy = camera_info.k[2], camera_info.k[5]

    points = np.column_stack([
        (u - cx) * d / fx,
        (v - cy) * d / fy,
        d,
    ])

    # --- Transform to base_link --------------------------------------------
    if tf_matrix is not None:
        pts_h = np.empty((points.shape[0], 4), dtype=np.float64)
        pts_h[:, :3] = points
        pts_h[:, 3] = 1.0
        points = (tf_matrix @ pts_h.T).T[:, :3]
        frame_id = "base_link"
    else:
        frame_id = "zed_left_camera_optical_frame"

    # --- Remove table points (in base_link, Z = up) -------------------------
    if table_z is not None:
        # Keep only points above the table surface (small margin for noise)
        points = points[points[:, 2] > table_z + 0.005]
        if points.shape[0] == 0:
            return None

    # --- Voxel downsample + hard cap to keep GPD fast on Orin CPU -----------
    points = _voxel_downsample(points, OUTPUT_VOXEL_LEAF)
    if points.shape[0] == 0:
        return None
    if points.shape[0] > MAX_OUTPUT_POINTS:
        idx = np.random.choice(points.shape[0], MAX_OUTPUT_POINTS, replace=False)
        points = points[idx]

    # --- Build PointCloud2 --------------------------------------------------
    header = Header()
    header.stamp = depth_msg.header.stamp
    header.frame_id = frame_id
    return pc2.create_cloud_xyz32(header, points.tolist())

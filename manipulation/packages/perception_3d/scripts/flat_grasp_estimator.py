#!/usr/bin/env python3

import copy
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import scipy.ndimage as ndi
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)
from frida_constants.manipulation_constants import (
    RIM_NAMES,
    FLAT_OBJECT_NAMES,
    PEAK_NAMES,
)

from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import EstimateFlatGrasp

# Fixed offset above table surface for grasp contact point (meters)
GRASP_SURFACE_OFFSET = 0.003

# Minimum valid points in ROI for PCA orientation
MIN_POINTS_FOR_PCA = 10

# Number of recent table height readings to average for stability
TABLE_HEIGHT_BUFFER_SIZE = 15

# Maximum allowed deviation from the running median (reject outliers)
TABLE_HEIGHT_OUTLIER_THRESH = 0.015  # 15mm

# --- Rim grasp estimation ---
RIM_MIN_HEIGHT = 0.05  # Minimum height above the floor
RIM_NEAR_FRACTION = 0.05  # Fraction of nearest points used to estimate the near rim
RIM_TOP_PERCENTILE = 90  # Percentile of Z used as the rim-top height
RIM_TOP_BAND = 0.03  # Vertical band below the rim-top kept as the rim ring (meters)

# --- Peak grasp estimation (highest content inside a cavity, e.g. clothes in a basket) ---
PEAK_GRID_RES = 0.05  # meters per cell of the elevation (height) grid
PEAK_NBR = 3  # neighborhood size (cells) for local-maximum detection
PEAK_MIN_HEIGHT = RIM_MIN_HEIGHT  # ignore peaks too close to the floor

# --- Service collection ---
FLAT_GRASP_DEFAULT_SAMPLES = 10  # frames to median when the caller passes <= 0
FLAT_GRASP_TIMEOUT = 5.0  # s: max wait to collect samples on one service call


class FlatGraspEstimator(Node):
    def __init__(self):
        super().__init__("flat_grasp_estimator")
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = "link_base"

        self.latest_depth = None
        self.intrinsics = None
        self.depth_frame_id = "zed_left_camera_optical_frame"

        self.target_classes = [n.lower() for n in FLAT_OBJECT_NAMES]
        self.rim_classes = [n.lower() for n in RIM_NAMES]
        self.peak_classes = [n.lower() for n in PEAK_NAMES]

        # Collection state. The node only does the heavy per-frame work while a
        # service request is collecting samples; it is idle otherwise (saves CPU).
        self._collecting = False
        self._target_label = None
        self._peak_mode = False
        self._samples = []

        # Rolling buffer for table height stabilization
        self.table_height_buffer = deque(maxlen=TABLE_HEIGHT_BUFFER_SIZE)

        # Reentrant group so the service handler can wait while the depth /
        # detection callbacks keep filling the sample buffer (needs MultiThreadedExecutor).
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10, callback_group=cb_group
        )
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1
        )
        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
            callback_group=cb_group,
        )

        # Debug visualization (published while collecting).
        self.pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/flat_grasp_pose", 10
        )
        self.rim_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/rim_grasp_pose", 10
        )

        self.peak_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/peak_grasp_pose", 10
        )

        self.estimate_srv = self.create_service(
            EstimateFlatGrasp,
            "/manipulation/estimate_flat_grasp",
            self.estimate_flat_grasp_cb,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"Flat Grasp Estimator ready (service). Mapping to: {self.target_frame}"
        )

    def estimate_flat_grasp_cb(self, request, response):
        """Collect several frames for request.object_name and return a single
        stabilized grasp pose (median position, latest PCA orientation)."""
        label = request.object_name.lower()
        is_peak = label in self.peak_classes
        if (
            label not in self.target_classes
            and label not in self.rim_classes
            and not is_peak
        ):
            response.success = False
            response.message = f"'{request.object_name}' is not a flat/rim/peak object"
            return response
        if self.intrinsics is None:
            response.success = False
            response.message = "camera intrinsics not received yet"
            return response

        n = (
            request.num_samples
            if request.num_samples > 0
            else FLAT_GRASP_DEFAULT_SAMPLES
        )

        # Start a fresh collection session.
        self._samples = []
        self._target_label = label
        self._peak_mode = is_peak
        self.table_height_buffer.clear()
        self.latest_depth = None
        self._collecting = True

        deadline = time.time() + FLAT_GRASP_TIMEOUT
        while time.time() < deadline and len(self._samples) < n:
            time.sleep(0.05)
        self._collecting = False

        samples = list(self._samples)
        self.get_logger().info(
            f"Collected {len(samples)} grasp samples for '{request.object_name}'"
        )
        if not samples:
            response.success = False
            response.message = f"no grasp samples for '{request.object_name}' within {FLAT_GRASP_TIMEOUT}s"
            return response

        pose = copy.deepcopy(samples[-1])  # latest PCA orientation
        pose.pose.position.x = float(np.median([s.pose.position.x for s in samples]))
        pose.pose.position.y = float(np.median([s.pose.position.y for s in samples]))
        pose.pose.position.z = float(np.median([s.pose.position.z for s in samples]))
        response.pose = pose
        response.success = True
        response.samples_collected = len(samples)
        response.message = f"averaged {len(samples)} samples"
        return response

    def depth_callback(self, msg):
        if not self._collecting:
            return
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.depth_frame_id = msg.header.frame_id

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                "fx": msg.k[0],
                "fy": msg.k[4],
                "cx": msg.k[2],
                "cy": msg.k[5],
            }

    def detections_callback(self, msg):
        if not self._collecting:
            return
        if self.latest_depth is None or self.intrinsics is None:
            return
        for det in msg.detections:
            label = det.label_text.lower()
            if self._peak_mode:
                # Peak content (e.g. clothes) is found inside a basket/rim
                # detection, so the requested label never matches directly.
                if label not in self.rim_classes:
                    continue
                pose = self.process_peak_object(det)
                pub = self.peak_pose_pub
            elif label != self._target_label:
                continue
            elif label in self.rim_classes:
                pose = self.process_rim_object(det)
                pub = self.rim_pose_pub
            else:
                pose = self.process_flat_object(det)
                pub = self.pose_pub
            if pose is not None:
                self._samples.append(pose)
                pub.publish(pose)  # debug viz

    def get_stable_table_height(self, new_reading):
        """Add a new table height reading and return a stable averaged value.
        Rejects outliers that deviate too much from the running median."""
        if len(self.table_height_buffer) >= 3:
            current_median = np.median(list(self.table_height_buffer))
            if abs(new_reading - current_median) > TABLE_HEIGHT_OUTLIER_THRESH:
                # Outlier — skip it, return current stable estimate
                return current_median

        self.table_height_buffer.append(new_reading)
        return np.median(list(self.table_height_buffer))

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _parse_bbox(self, detection: ObjectDetection):
        """Return (xmin, ymin, xmax, ymax) in pixels, or None if degenerate."""
        h_img, w_img = self.latest_depth.shape
        if detection.xmax <= 1.0 and detection.ymax <= 1.0:
            xmin = int(max(0, detection.xmin * w_img))
            ymin = int(max(0, detection.ymin * h_img))
            xmax = int(min(w_img, detection.xmax * w_img))
            ymax = int(min(h_img, detection.ymax * h_img))
        else:
            xmin = int(max(0, detection.xmin))
            ymin = int(max(0, detection.ymin))
            xmax = int(min(w_img, detection.xmax))
            ymax = int(min(h_img, detection.ymax))
        if xmax <= xmin or ymax <= ymin:
            return None
        return xmin, ymin, xmax, ymax

    def _deproject_pixels(self, u_global, v_global, z_vals) -> np.ndarray:
        """Deproject image pixels + depth to Nx3 points in camera frame."""
        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx, cy = self.intrinsics["cx"], self.intrinsics["cy"]
        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        return np.vstack((x, y, z_vals)).T

    def _lookup_T_cam_to_base(self):
        """Return 4x4 homogeneous transform (camera → base), or None on TF failure."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF tree from camera to base... {e}")
            return None
        q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]
        t = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = t
        return T

    def _apply_transform(self, points_3d_cam: np.ndarray, T_mat: np.ndarray):
        """Transform Nx3 camera-frame points to base frame. Returns filtered array or None."""
        hom = np.hstack((points_3d_cam, np.ones((len(points_3d_cam), 1))))
        points_base = (T_mat @ hom.T).T[:, :3]
        finite = np.isfinite(points_base).all(axis=1)
        points_base = points_base[finite]
        return points_base if len(points_base) >= MIN_POINTS_FOR_PCA else None

    def _make_grasp_pose(self, x: float, y: float, z: float, quat) -> PoseStamped:
        """Build a stamped PoseStamped in the target frame."""
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    # ------------------------------------------------------------------
    # Per-class grasp estimation
    # ------------------------------------------------------------------

    def process_flat_object(self, detection: ObjectDetection):
        bbox = self._parse_bbox(detection)
        if bbox is None:
            return
        xmin, ymin, xmax, ymax = bbox

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        valid_roi = roi_depth[valid_mask]
        if len(valid_roi) < MIN_POINTS_FOR_PCA:
            return

        # Table depth: 85th percentile (table is the farthest/deepest surface)
        table_z_cam = np.percentile(valid_roi, 85)

        # Object points: anything 1-50mm closer than table
        object_mask_roi = (
            valid_mask
            & (roi_depth < table_z_cam - 0.001)
            & (roi_depth > table_z_cam - 0.05)
        )
        v_local, u_local = np.where(object_mask_roi)
        if len(v_local) < MIN_POINTS_FOR_PCA:
            v_local, u_local = np.where(valid_mask)

        points_3d_cam = self._deproject_pixels(
            u_local + xmin, v_local + ymin, roi_depth[v_local, u_local]
        )

        T_mat = self._lookup_T_cam_to_base()
        if T_mat is None:
            return
        points_base = self._apply_transform(points_3d_cam, T_mat)
        if points_base is None:
            return

        # --- LONG AXIS via PCA on the object's XY points ---
        centroid_xy = np.mean(points_base[:, :2], axis=0)
        points_2d_xy = points_base[:, :2] - centroid_xy
        cov_matrix = np.cov(points_2d_xy.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        principal_vector = eigenvectors[:, np.argmax(eigenvalues)]
        spoon_dir = np.array([principal_vector[0], principal_vector[1], 0.0])
        spoon_dir = spoon_dir / np.linalg.norm(spoon_dir)
        perp_dir = np.array([-spoon_dir[1], spoon_dir[0]])  # in-plane perpendicular

        # --- GRASP POSITION: aim at the HANDLE (thin end), not the centroid ---
        # Project on the long axis; the handle end has the smaller perpendicular
        # spread (narrower), so offset the grasp toward it.
        handle_frac = 0.60  # fraction from center toward the handle tip (0..1)
        proj = points_2d_xy @ spoon_dir[:2]
        perp = points_2d_xy @ perp_dir
        pos = proj >= 0
        spread_pos = perp[pos].std() if np.count_nonzero(pos) > 2 else np.inf
        spread_neg = perp[~pos].std() if np.count_nonzero(~pos) > 2 else np.inf
        handle_sign = 1.0 if spread_pos <= spread_neg else -1.0
        side = pos if handle_sign > 0 else ~pos
        handle_extent = np.percentile(np.abs(proj[side]), 90) if np.any(side) else 0.0
        grasp_xy = (
            centroid_xy + (handle_sign * handle_frac * handle_extent) * spoon_dir[:2]
        )

        # Table surface height at the bbox center -> grasp Z
        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx, cy = self.intrinsics["cx"], self.intrinsics["cy"]
        bbox_cu, bbox_cv = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
        table_point_cam = np.array(
            [
                (bbox_cu - cx) * table_z_cam / fx,
                (bbox_cv - cy) * table_z_cam / fy,
                table_z_cam,
                1.0,
            ]
        )
        raw_table_height = (T_mat @ table_point_cam)[2]
        grasp_z = self.get_stable_table_height(raw_table_height) + GRASP_SURFACE_OFFSET

        # --- ORIENTATION: fingers close PERPENDICULAR to the long axis ---
        Z_grasp = np.array([0.0, 0.0, -1.0])
        Y_grasp = np.cross(Z_grasp, spoon_dir)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)
        X_grasp = np.cross(Y_grasp, Z_grasp)
        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        return self._make_grasp_pose(
            float(grasp_xy[0]), float(grasp_xy[1]), float(grasp_z), new_quat
        )

    def process_rim_object(self, detection: ObjectDetection):
        """Top-down straddle grasp on the near rim/edge of a cylindrical object."""
        bbox = self._parse_bbox(detection)
        if bbox is None:
            return
        xmin, ymin, xmax, ymax = bbox

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        if np.count_nonzero(valid_mask) < MIN_POINTS_FOR_PCA:
            return

        # Whole-frustum prism: deproject every valid pixel (no table seg).
        v_local, u_local = np.where(valid_mask)
        points_3d_cam = self._deproject_pixels(
            u_local + xmin, v_local + ymin, roi_depth[v_local, u_local]
        )

        T_mat = self._lookup_T_cam_to_base()
        if T_mat is None:
            return
        points_base = self._apply_transform(points_3d_cam, T_mat)
        if points_base is None:
            return

        # --- FLOOR REJECTION ---
        floor_z = np.percentile(points_base[:, 2], 5)
        rim_points = points_base[points_base[:, 2] > floor_z + RIM_MIN_HEIGHT]
        if len(rim_points) < MIN_POINTS_FOR_PCA:
            rim_points = points_base

        # --- RIM TOP RING ---
        # The rim is the TOP edge of the object. Anchor to the top:
        # take the rim-top height as a high Z percentile and keep only the top ring.
        top_z = np.percentile(rim_points[:, 2], RIM_TOP_PERCENTILE)
        top_ring = rim_points[rim_points[:, 2] > top_z - RIM_TOP_BAND]
        if len(top_ring) < MIN_POINTS_FOR_PCA:
            top_ring = rim_points

        # --- NEAR RIM POINT (closest to robot, within the top ring) ---
        horiz_dist = np.sqrt(top_ring[:, 0] ** 2 + top_ring[:, 1] ** 2)
        k = min(
            max(MIN_POINTS_FOR_PCA, int(RIM_NEAR_FRACTION * len(horiz_dist))),
            len(horiz_dist),
        )
        rim_point = np.median(top_ring[np.argsort(horiz_dist)[:k]], axis=0)
        rim_x, rim_y, rim_z = (
            float(rim_point[0]),
            float(rim_point[1]),
            float(rim_point[2]),
        )

        # --- STRADDLE ORIENTATION (top-down, fingers close radially across the wall) ---
        Z_grasp = np.array([0.0, 0.0, -1.0])
        radial = np.array([rim_x, rim_y, 0.0])
        radial_norm = np.linalg.norm(radial)
        if radial_norm < 1e-6:
            return
        radial = radial / radial_norm
        Y_grasp = radial
        X_grasp = np.cross(Y_grasp, Z_grasp)
        X_grasp = X_grasp / np.linalg.norm(X_grasp)
        Y_grasp = np.cross(Z_grasp, X_grasp)

        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        return self._make_grasp_pose(rim_x, rim_y, rim_z, new_quat)

    def process_peak_object(self, detection: ObjectDetection):
        """Top-down grasp on the highest content inside a cavity."""
        bbox = self._parse_bbox(detection)
        if bbox is None:
            return
        xmin, ymin, xmax, ymax = bbox

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        if np.count_nonzero(valid_mask) < MIN_POINTS_FOR_PCA:
            return

        v_local, u_local = np.where(valid_mask)
        points_3d_cam = self._deproject_pixels(
            u_local + xmin, v_local + ymin, roi_depth[v_local, u_local]
        )

        T_mat = self._lookup_T_cam_to_base()
        if T_mat is None:
            return
        points_base = self._apply_transform(points_3d_cam, T_mat)
        if points_base is None:
            return

        # --- FLOOR REJECTION (same as rim) ---
        floor_z = np.percentile(points_base[:, 2], 5)
        inside_points = points_base[points_base[:, 2] > floor_z + RIM_MIN_HEIGHT]
        if len(inside_points) < MIN_POINTS_FOR_PCA:
            inside_points = points_base

        # --- 2.5D ELEVATION GRID (max Z per XY cell) ---
        xs, ys, zs = (
            inside_points[:, 0],
            inside_points[:, 1],
            inside_points[:, 2],
        )
        x0, y0 = xs.min(), ys.min()
        nx = int((xs.max() - x0) / PEAK_GRID_RES) + 1
        ny = int((ys.max() - y0) / PEAK_GRID_RES) + 1
        if nx < PEAK_NBR or ny < PEAK_NBR:
            return
        ix = np.clip(((xs - x0) / PEAK_GRID_RES).astype(int), 0, nx - 1)
        iy = np.clip(((ys - y0) / PEAK_GRID_RES).astype(int), 0, ny - 1)

        grid = np.full((nx, ny), -np.inf, dtype=float)
        np.maximum.at(grid, (ix, iy), zs)
        filled = np.isfinite(grid)
        if np.count_nonzero(filled) < MIN_POINTS_FOR_PCA:
            return

        # --- LOCAL MAXIMA (peaks): cell equals the max in its neighborhood ---
        smoothed = ndi.maximum_filter(np.where(filled, grid, -np.inf), size=PEAK_NBR)
        peak_mask = filled & (grid >= smoothed) & (grid > floor_z + PEAK_MIN_HEIGHT)
        peak_ix, peak_iy = np.where(peak_mask)
        if len(peak_ix) == 0:
            return

        peak_x = x0 + (peak_ix + 0.5) * PEAK_GRID_RES
        peak_y = y0 + (peak_iy + 0.5) * PEAK_GRID_RES
        peak_z = grid[peak_ix, peak_iy]

        # --- CAVITY CENTER and nearest peak to it ---
        center_x = np.median(xs)
        center_y = np.median(ys)
        dist_to_center = np.sqrt((peak_x - center_x) ** 2 + (peak_y - center_y) ** 2)
        best = int(np.argmin(dist_to_center))
        peak_grasp_x, peak_grasp_y, peak_grasp_z = (
            float(peak_x[best]),
            float(peak_y[best]),
            float(peak_z[best]),
        )

        # --- TOP-DOWN ORIENTATION (point is central, no radial component) ---
        Z_grasp = np.array([0.0, 0.0, -1.0])
        X_grasp = np.array([1.0, 0.0, 0.0])
        Y_grasp = np.cross(Z_grasp, X_grasp)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)
        X_grasp = np.cross(Y_grasp, Z_grasp)

        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        return self._make_grasp_pose(peak_grasp_x, peak_grasp_y, peak_grasp_z, new_quat)


def main(args=None):
    rclpy.init(args=args)
    node = FlatGraspEstimator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

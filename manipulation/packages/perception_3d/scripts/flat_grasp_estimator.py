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
    MOONDREAM_DETECTION_TOPIC,
)
from frida_constants.manipulation_constants import (
    RIM_NAMES,
    FLAT_OBJECT_NAMES,
    PEAK_NAMES,
    HANDLE_NAMES,
    TRASH_BIN_NAME,
    PLACE_TRASH_HEIGHT_OFFSET,
)

from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import EstimateFlatGrasp, MoondreamDetection

# Fixed offset above table surface for grasp contact point (meters)
GRASP_SURFACE_OFFSET = 0.003

# Minimum valid points in ROI for PCA orientation
MIN_POINTS_FOR_PCA = 10

# Min eigenvalue ratio (long/short variance) for a trustworthy long axis. Below
# this the object looks square from this view, so the orientation is rejected.
# Live-tunable via the 'elongation_min_ratio' ROS parameter.
ELONGATION_MIN_RATIO = 2.0

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

# --- Trash bin place estimation ---
# Fraction of the bbox height (from the top) kept to estimate the bin center.
TRASH_TOP_BBOX_FRACTION = 0.5

# --- Handle (door lever) grasp estimation ---
# Horizontal-approach grasp on a lever handle: the bbox comes from a single
# Moondream open-vocabulary query (too slow for the per-frame YOLO loop).
HANDLE_DETECT_PROMPT = "door handle"  # Moondream subject
HANDLE_DETECT_TIMEOUT = 12.0  # s - Moondream is ~1-3 s per query
HANDLE_DEPTH_SAMPLES = 5  # depth frames to median for stability
HANDLE_DEPTH_TIMEOUT = 3.0  # s - max wait to collect the depth samples
HANDLE_PROTRUSION_MIN = 0.02  # m closer than the door plane to count as handle
HANDLE_PROTRUSION_MAX = 0.12  # m - reject spurious near returns
HANDLE_BAR_MAX_TILT_DEG = 30.0  # PCA long axis must be near-horizontal, else fallback
HANDLE_GRASP_Z_TWEAK = 0.0  # m fine height tuning (analog of FLAT_GRASP_Z_TWEAK)

# --- Service collection ---
FLAT_GRASP_DEFAULT_SAMPLES = 10  # frames to median when the caller passes <= 0
FLAT_GRASP_TIMEOUT = 5.0  # s: max wait to collect samples on one service call


class FlatGraspEstimator(Node):
    def __init__(self):
        super().__init__("flat_grasp_estimator")
        # Live-tunable min long/short variance ratio to trust the PCA long axis.
        self.declare_parameter("elongation_min_ratio", ELONGATION_MIN_RATIO)
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
        self.handle_classes = [n.lower() for n in HANDLE_NAMES]
        self.trash_classes = [TRASH_BIN_NAME.lower()]

        # Collection state. The node only does the heavy per-frame work while a
        # service request is collecting samples; it is idle otherwise (saves CPU).
        self._collecting = False
        self._target_label = None
        self._peak_mode = False
        self._trash_mode = False
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

        self.trash_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/trash_place_pose", 10
        )

        self.handle_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/handle_grasp_pose", 10
        )

        # Handle bboxes come from Moondream (open vocabulary), not the YOLO stream.
        self._moondream_detect_client = self.create_client(
            MoondreamDetection, MOONDREAM_DETECTION_TOPIC, callback_group=cb_group
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
        is_trash = label in self.trash_classes
        is_handle = label in self.handle_classes
        if (
            label not in self.target_classes
            and label not in self.rim_classes
            and not is_peak
            and not is_trash
            and not is_handle
        ):
            response.success = False
            response.message = (
                f"'{request.object_name}' is not a flat/rim/peak/trash/handle object"
            )
            return response
        if self.intrinsics is None:
            response.success = False
            response.message = "camera intrinsics not received yet"
            return response

        # Handles bypass the per-frame YOLO collection loop: a single Moondream
        # query provides the bbox, then a few depth frames are snapshotted.
        if is_handle:
            return self._estimate_handle_grasp(request, response)

        n = (
            request.num_samples
            if request.num_samples > 0
            else FLAT_GRASP_DEFAULT_SAMPLES
        )

        # Start a fresh collection session.
        self._samples = []
        self._target_label = label
        self._peak_mode = is_peak
        self._trash_mode = is_trash
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

    def _estimate_handle_grasp(self, request, response):
        """Handle mode: one Moondream bbox + a few depth frames -> a single
        horizontal-approach grasp pose on the handle bar."""
        if not self._moondream_detect_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "moondream detect service unavailable"
            return response

        det_req = MoondreamDetection.Request()
        det_req.subject = HANDLE_DETECT_PROMPT
        future = self._moondream_detect_client.call_async(det_req)
        deadline = time.time() + HANDLE_DETECT_TIMEOUT
        while time.time() < deadline and not future.done():
            time.sleep(0.05)
        det_resp = future.result() if future.done() else None
        if det_resp is None or not det_resp.success or not det_resp.detections:
            response.success = False
            response.message = (
                f"moondream found no '{HANDLE_DETECT_PROMPT}' "
                f"within {HANDLE_DETECT_TIMEOUT}s"
            )
            return response

        # Largest-area bbox wins when several handles are visible.
        detection = max(
            det_resp.detections,
            key=lambda d: max(0.0, d.xmax - d.xmin) * max(0.0, d.ymax - d.ymin),
        )

        n = request.num_samples if request.num_samples > 0 else HANDLE_DEPTH_SAMPLES

        # Reactivate depth_callback only: with no target label set, the YOLO
        # detections_callback matches nothing while we snapshot depth frames.
        self._samples = []
        self._target_label = None
        self._peak_mode = False
        self._trash_mode = False
        self.latest_depth = None
        self._collecting = True
        try:
            deadline = time.time() + HANDLE_DEPTH_TIMEOUT
            while time.time() < deadline and len(self._samples) < n:
                if self.latest_depth is None:
                    time.sleep(0.02)
                    continue
                pose = self.process_handle_object(detection)
                self.latest_depth = None  # force a fresh frame per sample
                if pose is not None:
                    self._samples.append(pose)
                    self.handle_pose_pub.publish(pose)  # debug viz
        finally:
            self._collecting = False

        samples = list(self._samples)
        self.get_logger().info(
            f"Collected {len(samples)} handle grasp samples for "
            f"'{request.object_name}'"
        )
        if not samples:
            response.success = False
            response.message = f"no handle grasp samples within {HANDLE_DEPTH_TIMEOUT}s"
            return response

        pose = copy.deepcopy(samples[-1])  # latest orientation
        pose.pose.position.x = float(np.median([s.pose.position.x for s in samples]))
        pose.pose.position.y = float(np.median([s.pose.position.y for s in samples]))
        pose.pose.position.z = float(np.median([s.pose.position.z for s in samples]))
        response.pose = pose
        response.success = True
        response.samples_collected = len(samples)
        response.message = f"averaged {len(samples)} handle samples"
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
            if self._trash_mode:
                if label not in self.trash_classes:
                    continue
                pose = self.process_trash_bin(det)
                pub = self.trash_pose_pub
            elif self._peak_mode:
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
        # Keep the largest connected blob (the object body); drop scattered table
        # noise. No whole-bbox fallback: a square bbox yields an arbitrary axis.
        labeled, num = ndi.label(object_mask_roi)
        if num == 0:
            return
        largest = int(np.argmax(np.bincount(labeled.ravel())[1:])) + 1
        v_local, u_local = np.where(labeled == largest)
        if len(v_local) < MIN_POINTS_FOR_PCA:
            return

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
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)  # ascending, real
        # Reject near-square point sets: the long axis is unreliable there and
        # would grasp parallel to the object instead of across it.
        elongation_min = self.get_parameter("elongation_min_ratio").value
        if eigenvalues[0] <= 1e-9 or eigenvalues[1] / eigenvalues[0] < elongation_min:
            return
        principal_vector = eigenvectors[:, 1]
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

    def process_trash_bin(self, detection: ObjectDetection):
        """Place pose for dropping an object into a trash bin: centered in XY
        over the bin, a fixed offset above its highest point, gripper top-down."""
        bbox = self._parse_bbox(detection)
        if bbox is None:
            return
        xmin, ymin, xmax, ymax = bbox

        # Use only the upper part of the bbox: the front wall fills the lower
        ymid = ymin + max(1, int((ymax - ymin) * TRASH_TOP_BBOX_FRACTION))

        roi_depth = self.latest_depth[ymin:ymid, xmin:xmax]
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

        # --- FLOOR REJECTION ---
        floor_z = np.percentile(points_base[:, 2], 5)
        bin_points = points_base[points_base[:, 2] > floor_z + RIM_MIN_HEIGHT]
        if len(bin_points) < MIN_POINTS_FOR_PCA:
            bin_points = points_base

        xs, ys, zs = bin_points[:, 0], bin_points[:, 1], bin_points[:, 2]

        # --- CENTER (XY) and HIGHEST POINT (Z) ---
        center_x = float(np.median(xs))
        center_y = float(np.median(ys))
        top_z = float(np.percentile(zs, RIM_TOP_PERCENTILE))

        # Place pose: fixed offset above the rim top, centered over the bin.
        place_z = top_z + PLACE_TRASH_HEIGHT_OFFSET

        # --- TOP-DOWN ORIENTATION ---
        Z_grasp = np.array([0.0, 0.0, -1.0])
        X_grasp = np.array([1.0, 0.0, 0.0])
        Y_grasp = np.cross(Z_grasp, X_grasp)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)
        X_grasp = np.cross(Y_grasp, Z_grasp)

        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        return self._make_grasp_pose(center_x, center_y, place_z, new_quat)

    def process_handle_object(self, detection: ObjectDetection):
        """Horizontal-approach grasp on a door lever handle: approach axis
        points from the robot base at the door, fingers close vertically over
        the (near-horizontal) handle bar."""
        bbox = self._parse_bbox(detection)
        if bbox is None:
            return
        xmin, ymin, xmax, ymax = bbox

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        valid_roi = roi_depth[valid_mask]
        if len(valid_roi) < MIN_POINTS_FOR_PCA:
            return

        # Door plane depth: the door fills the bbox background (same trick as
        # the table in process_flat_object); the handle protrudes toward us.
        door_z_cam = np.percentile(valid_roi, 85)
        handle_mask = (
            valid_mask
            & (roi_depth < door_z_cam - HANDLE_PROTRUSION_MIN)
            & (roi_depth > door_z_cam - HANDLE_PROTRUSION_MAX)
        )
        labeled, num = ndi.label(handle_mask)
        if num == 0:
            return
        largest = int(np.argmax(np.bincount(labeled.ravel())[1:])) + 1
        v_local, u_local = np.where(labeled == largest)
        if len(v_local) < MIN_POINTS_FOR_PCA:
            return

        points_3d_cam = self._deproject_pixels(
            u_local + xmin, v_local + ymin, roi_depth[v_local, u_local]
        )

        T_mat = self._lookup_T_cam_to_base()
        if T_mat is None:
            return
        points_base = self._apply_transform(points_3d_cam, T_mat)
        if points_base is None:
            return

        # Bar center: median is robust to specular depth holes on metal.
        hx = float(np.median(points_base[:, 0]))
        hy = float(np.median(points_base[:, 1]))
        hz = float(np.median(points_base[:, 2]))

        # --- APPROACH AXIS: horizontal base->handle ray. The robot docks
        # facing the door, so this approximates the door normal. ---
        approach = np.array([hx, hy, 0.0])
        approach_norm = np.linalg.norm(approach)
        if approach_norm < 1e-6:
            return
        approach = approach / approach_norm

        # --- BAR DIRECTION via PCA in the door plane (tangent, up) basis ---
        z_up = np.array([0.0, 0.0, 1.0])
        tangent = np.cross(z_up, approach)
        tangent = tangent / np.linalg.norm(tangent)
        centered = points_base - np.array([hx, hy, hz])
        plane_2d = np.column_stack((centered @ tangent, centered[:, 2]))
        cov_matrix = np.cov(plane_2d.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        elongation_min = self.get_parameter("elongation_min_ratio").value
        bar_dir = tangent  # fallback: assume a horizontal bar
        if eigenvalues[0] > 1e-9 and eigenvalues[1] / eigenvalues[0] >= elongation_min:
            long_axis = eigenvectors[:, 1]  # in (tangent, up) coords
            tilt = np.degrees(np.arctan2(abs(long_axis[1]), abs(long_axis[0])))
            if tilt <= HANDLE_BAR_MAX_TILT_DEG:
                bar_dir = long_axis[0] * tangent + long_axis[1] * z_up
                bar_dir = bar_dir / np.linalg.norm(bar_dir)

        # --- ORIENTATION: approach horizontally, fingers close over the bar ---
        Z_grasp = approach
        X_grasp = bar_dir - (bar_dir @ Z_grasp) * Z_grasp
        x_norm = np.linalg.norm(X_grasp)
        if x_norm < 1e-6:
            return
        X_grasp = X_grasp / x_norm
        Y_grasp = np.cross(Z_grasp, X_grasp)
        if Y_grasp[2] > 0:  # deterministic roll; the alt candidate flips it
            X_grasp, Y_grasp = -X_grasp, -Y_grasp
        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        return self._make_grasp_pose(hx, hy, hz + HANDLE_GRASP_Z_TWEAK, new_quat)


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

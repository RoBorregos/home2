#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from std_srvs.srv import SetBool

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)

from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection

# Fixed offset above table surface for grasp contact point (meters)
GRASP_SURFACE_OFFSET = 0.003

# Minimum valid points in ROI for PCA orientation
MIN_POINTS_FOR_PCA = 10

# Number of recent table height readings to average for stability
TABLE_HEIGHT_BUFFER_SIZE = 15

# Maximum allowed deviation from the running median (reject outliers)
TABLE_HEIGHT_OUTLIER_THRESH = 0.015  # 15mm

# --- Basket (rim/edge) grasp estimation ---
# Minimum height above the detected floor for a point to count as basket (not floor)
RIM_MIN_HEIGHT = 0.05  # m
# Fraction of points closest to the base used to estimate the near rim point
BASKET_NEAR_FRACTION = 0.05
# Percentile of Z used as the rim-top height (robust to a few high outliers)
BASKET_TOP_PERCENTILE = 90
# Vertical band below the rim-top kept as the rim ring (meters)
RIM_TOP_BAND = 0.03
# Half-size (pixels) of the window used to median-filter the bbox-center depth for clothes
CLOTHES_CENTER_WINDOW = 5


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

        self.target_classes = ["spoon", "fork", "knife"]
        self.basket_classes = ["aundry_basket", "basket"]

        # Start disabled — only enabled explicitly before a cutlery pick.
        # Saves CPU/network when no manipulation is in progress.
        self.enabled = False

        # Rolling buffer for table height stabilization
        self.table_height_buffer = deque(maxlen=TABLE_HEIGHT_BUFFER_SIZE)

        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1
        )
        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/flat_grasp_pose", 10
        )

        self.basket_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/basket_grasp_pose", 10
        )

        # Clothes inside the basket are located from the SAME basket detection
        # (bbox centroid), published on a separate topic. PickManager samples
        # whichever topic matches the requested pick (rim vs clothes).
        self.clothes_pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/clothes_grasp_pose", 10
        )

        self.enable_srv = self.create_service(
            SetBool, "/flat_grasp_estimator/enable", self.enable_callback
        )

        self.get_logger().info(
            f"Flat Grasp Estimator initialized (disabled). Mapping to: {self.target_frame}"
        )

    def enable_callback(self, request, response):
        """Enable/disable detection processing. When disabled the node stays
        alive and keeps its subscriptions but skips all work in the hot path."""
        self.enabled = bool(request.data)
        # Clear stale buffers when toggling so the next enable starts clean.
        if not self.enabled:
            self.table_height_buffer.clear()
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Flat Grasp Estimator {response.message}")
        return response

    def depth_callback(self, msg):
        if not self.enabled:
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
        if not self.enabled:
            return
        if self.latest_depth is None or self.intrinsics is None:
            return
        for det in msg.detections:
            label = det.label_text.lower()
            if label in self.target_classes:
                self.process_flat_object(det)
            elif label in self.basket_classes:
                self.process_basket_object(det)

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

    def process_flat_object(self, detection: ObjectDetection):
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
            return

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

        use_full_roi = len(v_local) < MIN_POINTS_FOR_PCA
        if use_full_roi:
            v_local, u_local = np.where(valid_mask)

        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx, cy = self.intrinsics["cx"], self.intrinsics["cy"]
        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_3d_cam = np.vstack((x, y, z_vals)).T

        # --- TRANSFORM TO BASE FRAME ---
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF tree from camera to base... {e}")
            return

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
        rot_mat = Rotation.from_quat(q).as_matrix()

        T_mat = np.eye(4)
        T_mat[:3, :3] = rot_mat
        T_mat[:3, 3] = t

        points_3d_hom = np.hstack((points_3d_cam, np.ones((points_3d_cam.shape[0], 1))))
        points_base = (T_mat @ points_3d_hom.T).T[:, :3]

        # Filter out NaN/Inf points
        valid_mask = np.isfinite(points_base).all(axis=1)
        points_base = points_base[valid_mask]
        if len(points_base) < MIN_POINTS_FOR_PCA:
            return

        # --- GRASP POSITION ---
        centroid_xy = np.mean(points_base[:, :2], axis=0)

        # Transform table surface point at bbox center to base frame
        bbox_center_u = (xmin + xmax) / 2.0
        bbox_center_v = (ymin + ymax) / 2.0
        table_x_cam = (bbox_center_u - cx) * table_z_cam / fx
        table_y_cam = (bbox_center_v - cy) * table_z_cam / fy
        table_point_cam = np.array([table_x_cam, table_y_cam, table_z_cam, 1.0])
        table_point_base = T_mat @ table_point_cam
        raw_table_height = table_point_base[2]

        # Stabilize table height with rolling median + outlier rejection
        stable_table_height = self.get_stable_table_height(raw_table_height)

        grasp_z = stable_table_height + GRASP_SURFACE_OFFSET

        # --- GRASP ORIENTATION (PCA on XY to find long axis) ---
        points_2d_xy = points_base[:, :2] - centroid_xy
        cov_matrix = np.cov(points_2d_xy.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        principal_vector = eigenvectors[:, np.argmax(eigenvalues)]

        spoon_dir = np.array([principal_vector[0], principal_vector[1], 0.0])
        spoon_dir = spoon_dir / np.linalg.norm(spoon_dir)

        Z_grasp = np.array([0.0, 0.0, -1.0])
        Y_grasp = np.cross(Z_grasp, spoon_dir)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)
        X_grasp = np.cross(Y_grasp, Z_grasp)

        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        # --- PUBLISH ---
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.target_frame
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        grasp_pose.pose.position.x = float(centroid_xy[0])
        grasp_pose.pose.position.y = float(centroid_xy[1])
        grasp_pose.pose.position.z = float(grasp_z)

        grasp_pose.pose.orientation.x = new_quat[0]
        grasp_pose.pose.orientation.y = new_quat[1]
        grasp_pose.pose.orientation.z = new_quat[2]
        grasp_pose.pose.orientation.w = new_quat[3]

        self.pose_pub.publish(grasp_pose)
        self.get_logger().info(
            f"Grasp (link_base): X={centroid_xy[0]:.3f}, "
            f"Y={centroid_xy[1]:.3f}, Z={grasp_z:.4f} "
            f"(raw_table={raw_table_height:.4f}, stable_table={stable_table_height:.4f}, "
            f"buf={len(self.table_height_buffer)})"
        )

    def process_basket_object(self, detection: ObjectDetection):
        """Estimate a top-down grasp on the near rim/edge of a basket on the floor.

        Deprojects the whole detection bbox into a 3D prism, transforms it to the
        base frame, rejects the floor, and selects the point closest to the base
        (the near rim). The grasp is oriented so the gripper fingers close radially
        across the rim wall (one finger inside, one outside the basket).
        """
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
            return

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]

        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        if np.count_nonzero(valid_mask) < MIN_POINTS_FOR_PCA:
            return

        # Whole-frustum prism: deproject every valid pixel in the bbox (no table seg).
        v_local, u_local = np.where(valid_mask)
        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx, cy = self.intrinsics["cx"], self.intrinsics["cy"]
        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_3d_cam = np.vstack((x, y, z_vals)).T

        # --- TRANSFORM TO BASE FRAME ---
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF tree from camera to base... {e}")
            return

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
        rot_mat = Rotation.from_quat(q).as_matrix()

        T_mat = np.eye(4)
        T_mat[:3, :3] = rot_mat
        T_mat[:3, 3] = t

        points_3d_hom = np.hstack((points_3d_cam, np.ones((points_3d_cam.shape[0], 1))))
        points_base = (T_mat @ points_3d_hom.T).T[:, :3]

        # Filter out NaN/Inf points
        finite_mask = np.isfinite(points_base).all(axis=1)
        points_base = points_base[finite_mask]
        if len(points_base) < MIN_POINTS_FOR_PCA:
            return

        # --- FLOOR REJECTION ---
        # The floor is the lowest surface in the frustum; keep only points that sit
        # clearly above it so nearer floor returns aren't mistaken for the rim.
        floor_z = np.percentile(points_base[:, 2], 5)
        rim_mask = points_base[:, 2] > floor_z + RIM_MIN_HEIGHT
        rim_points = points_base[rim_mask]
        if len(rim_points) < MIN_POINTS_FOR_PCA:
            self.get_logger().warn(
                "Not enough basket points above the floor for a rim estimate"
            )
            return

        # --- RIM TOP RING ---
        # The rim is the TOP edge of the basket. Selecting points purely by horizontal
        # distance picks the whole near wall (top..floor), whose median Z lands near the
        # wall middle -> the grasp would descend far below the rim. Anchor to the top:
        # take the rim-top height as a high Z percentile and keep only the top ring.
        top_z = np.percentile(rim_points[:, 2], BASKET_TOP_PERCENTILE)
        top_ring = rim_points[rim_points[:, 2] > top_z - RIM_TOP_BAND]
        if len(top_ring) < MIN_POINTS_FOR_PCA:
            top_ring = rim_points

        # --- NEAR RIM POINT (closest to base, along the top ring) ---
        horiz_dist = np.sqrt(top_ring[:, 0] ** 2 + top_ring[:, 1] ** 2)
        k = max(MIN_POINTS_FOR_PCA, int(BASKET_NEAR_FRACTION * len(horiz_dist)))
        k = min(k, len(horiz_dist))
        near_idx = np.argsort(horiz_dist)[:k]
        rim_point = np.median(top_ring[near_idx], axis=0)
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

        # Gripper local Y is the finger-closing axis (same convention as the flat path).
        Y_grasp = radial
        X_grasp = np.cross(Y_grasp, Z_grasp)
        X_grasp = X_grasp / np.linalg.norm(X_grasp)
        Y_grasp = np.cross(Z_grasp, X_grasp)

        new_rot_mat = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        new_quat = Rotation.from_matrix(new_rot_mat).as_quat()

        # --- PUBLISH ---
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.target_frame
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        grasp_pose.pose.position.x = rim_x
        grasp_pose.pose.position.y = rim_y
        grasp_pose.pose.position.z = rim_z

        grasp_pose.pose.orientation.x = new_quat[0]
        grasp_pose.pose.orientation.y = new_quat[1]
        grasp_pose.pose.orientation.z = new_quat[2]
        grasp_pose.pose.orientation.w = new_quat[3]

        self.basket_pose_pub.publish(grasp_pose)
        self.get_logger().info(
            f"Basket rim grasp (link_base): X={rim_x:.3f}, Y={rim_y:.3f}, "
            f"Z={rim_z:.4f} (floor_z={floor_z:.4f}, top_z={top_z:.4f}, "
            f"rim_pts={len(rim_points)}, top_ring={len(top_ring)}, near_k={k})"
        )

        # --- CLOTHES POSE (bbox centroid, for picking clothes INSIDE the basket) ---
        # The clothes are located from this same basket detection: deproject the bbox
        # center pixel (≈ the top of the clothes pile in the middle of the basket).
        self.publish_clothes_pose(xmin, xmax, ymin, ymax, fx, fy, cx, cy, T_mat)

    def publish_clothes_pose(self, xmin, xmax, ymin, ymax, fx, fy, cx, cy, T_mat):
        """Deproject the basket bbox center and publish a top-down clothes grasp pose.

        Reuses the intrinsics and base transform already computed for the rim. The
        center depth is a small-window median (robust to holes / dark fabric). The
        orientation matches the rim grasp (top-down, wrist radial toward the base).
        """
        cu = int((xmin + xmax) / 2)
        cv = int((ymin + ymax) / 2)

        # Median depth over a small window around the center (robust to NaN/holes).
        win = CLOTHES_CENTER_WINDOW
        w_lo = max(xmin, cu - win)
        w_hi = min(xmax, cu + win + 1)
        h_lo = max(ymin, cv - win)
        h_hi = min(ymax, cv + win + 1)
        center_patch = self.latest_depth[h_lo:h_hi, w_lo:w_hi]
        valid = center_patch[(center_patch > 0) & (~np.isnan(center_patch))]
        if len(valid) == 0:
            return
        z = float(np.median(valid))

        x_cam = (cu - cx) * z / fx
        y_cam = (cv - cy) * z / fy
        point_cam = np.array([x_cam, y_cam, z, 1.0])
        point_base = T_mat @ point_cam
        if not np.isfinite(point_base[:3]).all():
            return
        clothes_x, clothes_y, clothes_z = (
            float(point_base[0]),
            float(point_base[1]),
            float(point_base[2]),
        )

        # --- TOP-DOWN ORIENTATION (radial wrist, same convention as the rim grasp) ---
        Z_grasp = np.array([0.0, 0.0, -1.0])
        radial = np.array([clothes_x, clothes_y, 0.0])
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

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.target_frame
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        grasp_pose.pose.position.x = clothes_x
        grasp_pose.pose.position.y = clothes_y
        grasp_pose.pose.position.z = clothes_z

        grasp_pose.pose.orientation.x = new_quat[0]
        grasp_pose.pose.orientation.y = new_quat[1]
        grasp_pose.pose.orientation.z = new_quat[2]
        grasp_pose.pose.orientation.w = new_quat[3]

        self.clothes_pose_pub.publish(grasp_pose)
        self.get_logger().info(
            f"Clothes grasp (link_base): X={clothes_x:.3f}, Y={clothes_y:.3f}, "
            f"Z={clothes_z:.4f} (center_px=({cu},{cv}), patch_pts={len(valid)})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FlatGraspEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

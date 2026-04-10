#!/usr/bin/env python3
"""
Refactored Basket Handle Grasp Estimator (v2.0)

High-precision rim detection using:
1. Z-slicing (top 4cm)
2. Downsampling to 300-500 points
3. Local PCA linearity scoring at each point
4. Selection of highest linearity for handle direction
5. Generation of 4 alternative grasp poses

Publishes ONLY grasping alternatives (no lift pose).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.vision_constants import (
    ZERO_SHOT_DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray


# ============================================================================
# Configuration Parameters
# ============================================================================

# Z-slicing: keep only points within this distance from max Z
Z_SLICE_DISTANCE = 0.04  # meters (4 cm)

# Downsampling target
DOWNSAMPLE_TARGET = 400  # aim for ~300-500 points

# Local neighborhood for PCA
NEIGHBOR_RADIUS = 0.025  # meters (2.5 cm)

# Grasp surface offset
GRASP_SURFACE_OFFSET = 0.02  # meters

# Handle offset distance for alternatives
HANDLE_OFFSET_DIST = 0.03  # meters

# Minimum points for valid PCA
MIN_POINTS_FOR_PCA = 20


class BasketHandleGraspEstimator(Node):
    def __init__(self):
        super().__init__("basket_handle_grasp_estimator_v2")
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = "link_base"
        self.depth_frame_id = "zed_left_camera_optical_frame"

        self.latest_depth = None
        self.intrinsics = None

        # Subscriptions
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1
        )
        self.create_subscription(
            ObjectDetectionArray,
            ZERO_SHOT_DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        # Publisher: only grasp poses (no lift pose)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/manipulation/basket_grasp_pose", 10
        )

        self.get_logger().info("Basket Handle Grasp Estimator v2.0 initialized")

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.depth_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                "fx": msg.k[0],
                "fy": msg.k[4],
                "cx": msg.k[2],
                "cy": msg.k[5],
            }

    def detections_callback(self, msg: ObjectDetectionArray):
        if self.latest_depth is None or self.intrinsics is None:
            return

        for det in msg.detections:
            label = det.label_text.lower() if hasattr(det, "label_text") else ""
            if (
                "basket" in label
                or "casket" in label
                or "basket_handle" in label
            ):
                try:
                    self.process_basket_detection(det)
                except Exception as e:
                    self.get_logger().error(f"Error processing detection: {e}")

    def process_basket_detection(self, detection):
        """
        Main pipeline:
        1. Extract ROI from depth
        2. Transform to base frame
        3. Z-slice (top 4cm)
        4. Downsample
        5. Find rim direction via local PCA linearity
        6. Generate 4 poses
        7. Publish
        """
        h_img, w_img = self.latest_depth.shape

        # ======================================================================
        # STEP 1: Extract ROI and convert to 3D points
        # ======================================================================
        if detection.xmax <= 1.0 and detection.ymax <= 1.0:
            # Normalized coords
            xmin = int(max(0, detection.xmin * w_img))
            ymin = int(max(0, detection.ymin * h_img))
            xmax = int(min(w_img, detection.xmax * w_img))
            ymax = int(min(h_img, detection.ymax * h_img))
        else:
            # Pixel coords
            xmin = int(max(0, detection.xmin))
            ymin = int(max(0, detection.ymin))
            xmax = int(min(w_img, detection.xmax))
            ymax = int(min(h_img, detection.ymax))

        if xmax <= xmin or ymax <= ymin:
            self.get_logger().warn("Invalid ROI bounds")
            return

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        if not np.any(valid_mask):
            self.get_logger().warn("No valid depth in ROI")
            return

        # Extract valid points in camera frame
        v_local, u_local = np.where(valid_mask)
        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx, cy = self.intrinsics["cx"], self.intrinsics["cy"]

        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_3d_cam = np.vstack((x, y, z_vals)).T

        # ======================================================================
        # STEP 2: Transform to base frame
        # ======================================================================
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
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

        points_3d_hom = np.hstack(
            (points_3d_cam, np.ones((points_3d_cam.shape[0], 1)))
        )
        points_base = (T_mat @ points_3d_hom.T).T[:, :3]

        # ======================================================================
        # STEP 3: Z-slice (keep top 4cm)
        # ======================================================================
        z_max = np.max(points_base[:, 2])
        z_min_threshold = z_max - Z_SLICE_DISTANCE
        rim_mask = points_base[:, 2] >= z_min_threshold
        rim_points = points_base[rim_mask]

        if rim_points.shape[0] < MIN_POINTS_FOR_PCA:
            self.get_logger().warn(
                f"Not enough rim points after Z-slice: {rim_points.shape[0]}"
            )
            return

        self.get_logger().info(
            f"Z-sliced rim: {rim_points.shape[0]} points (z ∈ [{z_min_threshold:.3f}, {z_max:.3f}])"
        )

        # ======================================================================
        # STEP 4: Downsample
        # ======================================================================
        if rim_points.shape[0] > DOWNSAMPLE_TARGET:
            downsample_ratio = DOWNSAMPLE_TARGET / rim_points.shape[0]
            indices = np.random.choice(
                rim_points.shape[0],
                size=DOWNSAMPLE_TARGET,
                replace=False,
            )
            rim_points = rim_points[indices]

        self.get_logger().info(f"Downsampled to {rim_points.shape[0]} points")

        # ======================================================================
        # STEP 5: Local PCA Linearity Scoring
        # ======================================================================
        # Build KDTree for neighbor search
        kdtree = cKDTree(rim_points)

        linearity_scores = np.zeros(rim_points.shape[0])

        for i, point in enumerate(rim_points):
            # Find neighbors within radius
            neighbor_indices = kdtree.query_ball_point(point, NEIGHBOR_RADIUS)

            if len(neighbor_indices) < 3:
                # Not enough neighbors for meaningful PCA
                linearity_scores[i] = 0.0
                continue

            neighbor_points = rim_points[neighbor_indices]
            centroid_local = np.mean(neighbor_points, axis=0)
            pts_centered = neighbor_points - centroid_local

            # Covariance
            cov = np.cov(pts_centered.T)
            eigvals = np.linalg.eigvalsh(cov)
            eigvals = np.sort(eigvals)[::-1]  # descending order

            e1, e2, e3 = eigvals[0], eigvals[1], eigvals[2]

            # Linearity score
            linearity_scores[i] = e1 / (e2 + e3 + 1e-6)

        # ======================================================================
        # STEP 6: Select point with highest linearity
        # ======================================================================
        best_idx = np.argmax(linearity_scores)
        best_point = rim_points[best_idx]
        best_score = linearity_scores[best_idx]

        self.get_logger().info(
            f"Best linearity point: idx={best_idx}, score={best_score:.4f}, "
            f"pos=({best_point[0]:.3f}, {best_point[1]:.3f}, {best_point[2]:.3f})"
        )

        # Extract handle direction from local PCA at best point
        neighbor_indices = kdtree.query_ball_point(best_point, NEIGHBOR_RADIUS)
        neighbor_points = rim_points[neighbor_indices]
        centroid_local = np.mean(neighbor_points, axis=0)
        pts_centered = neighbor_points - centroid_local

        cov = np.cov(pts_centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        # eigvecs are columns; last column (largest eigenvalue)
        principal_eigenvector = eigvecs[:, -1]

        # Project to XY plane
        handle_dir = np.array([principal_eigenvector[0], principal_eigenvector[1], 0.0])
        if np.linalg.norm(handle_dir) < 1e-6:
            self.get_logger().warn("Degenerate handle direction")
            return

        handle_dir = handle_dir / np.linalg.norm(handle_dir)

        self.get_logger().info(
            f"Handle direction: ({handle_dir[0]:.3f}, {handle_dir[1]:.3f}, {handle_dir[2]:.3f})"
        )

        # ======================================================================
        # STEP 7: Generate 4 alternative poses
        # ======================================================================
        # End-effector frame: Z down, X along handle
        Z_grasp = np.array([0.0, 0.0, -1.0])
        X_grasp = np.array([handle_dir[0], handle_dir[1], 0.0])
        X_grasp = X_grasp / np.linalg.norm(X_grasp)
        Y_grasp = np.cross(Z_grasp, X_grasp)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)

        # Normal rotation
        rot_normal = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        quat_normal = Rotation.from_matrix(rot_normal).as_quat()

        # Reversed (180° in Z)
        rot_reversed = np.column_stack((-X_grasp, -Y_grasp, Z_grasp))
        quat_reversed = Rotation.from_matrix(rot_reversed).as_quat()

        # Helper to create pose
        def create_pose(position_offset, quat):
            p = PoseStamped()
            p.header.frame_id = self.target_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(best_point[0] + position_offset[0])
            p.pose.position.y = float(best_point[1] + position_offset[1])
            p.pose.position.z = float(best_point[2] + GRASP_SURFACE_OFFSET)
            p.pose.orientation.x = float(quat[0])
            p.pose.orientation.y = float(quat[1])
            p.pose.orientation.z = float(quat[2])
            p.pose.orientation.w = float(quat[3])
            return p

        poses = [
            create_pose([0.0, 0.0], quat_normal),  # Center normal
            create_pose([0.0, 0.0], quat_reversed),  # Center reversed
            create_pose(X_grasp * HANDLE_OFFSET_DIST, quat_normal),  # Offset +X
            create_pose(-X_grasp * HANDLE_OFFSET_DIST, quat_normal),  # Offset -X
        ]

        # ======================================================================
        # STEP 8: Publish grasp alternatives
        # ======================================================================
        self.get_logger().info("Publishing 4 basket grasp alternatives...")
        for i, pose in enumerate(poses):
            self.pose_pub.publish(pose)
            self.get_logger().debug(
                f"  Pose {i}: pos=({pose.pose.position.x:.3f}, "
                f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BasketHandleGraspEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

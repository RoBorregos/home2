#!/usr/bin/env python3
"""
Simple basket-handle grasp estimator.

Publishes two PoseStamped topics when a basket (or similar) is detected:
- /manipulation/basket_grasp_pose : pose to perform the grasp (end-effector pointing down and parallel to handle)
- /manipulation/basket_lift_pose  : safe lift pose after grasp (raise in world Z to avoid spilling)

Approach: take depth ROI from detection bbox, transform points to base frame,
compute PCA to find principal axis (handle direction), build a rotation where
Z points downward and X is aligned with the handle. The node avoids adding
collision objects itself (leave collision management to pick flow).

This is intentionally lightweight and mirrors the style of `flat_grasp_estimator.py`.
"""

import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.vision_constants import (
    ZERO_SHOT_DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)

from frida_interfaces.msg import ObjectDetectionArray 


# Tunable parameters
GRASP_SURFACE_OFFSET = 0.02  # meters: how far above the detected handle surface to place the gripper contact
MIN_POINTS_FOR_PCA = 20
LIFT_HEIGHT = 0.20  # meters to lift after grasp to avoid spilling
APPROACH_DISTANCE = 0.12  # meters offset from grasp along approach axis


class BasketHandleGraspEstimator(Node):
    def __init__(self):
        super().__init__('basket_handle_grasp_estimator')
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'link_base'
        self.depth_frame_id = 'zed_left_camera_optical_frame'

        self.latest_depth = None
        self.intrinsics = None

        # Topics
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(ObjectDetectionArray, ZERO_SHOT_DETECTIONS_TOPIC, self.detections_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/manipulation/basket_grasp_pose', 10)
        self.lift_pub = self.create_publisher(PoseStamped, '/manipulation/basket_lift_pose', 10)

        self.get_logger().info('Basket Handle Grasp Estimator initialized')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5]}

    def detections_callback(self, msg: ObjectDetectionArray):
        # Guard clauses
        if self.latest_depth is None or self.intrinsics is None:
            return

        for det in msg.detections:
            label = det.label_text.lower() if hasattr(det, 'label_text') else ''
            # Consider common labels for baskets/containers; adjust as needed
            if 'basket' in label or 'casket' in label or 'basket_handle' in label:
                try:
                    self.process_basket_detection(det)
                except Exception as e:
                    self.get_logger().error(f'Error processing detection: {e}')

    def process_basket_detection(self, detection):
        # Convert bbox to pixel coordinates
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
        if not np.any(valid_mask):
            return

        v_local, u_local = np.where(valid_mask)
        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        cx, cy = self.intrinsics['cx'], self.intrinsics['cy']

        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_3d_cam = np.vstack((x, y, z_vals)).T

        # Transform to base frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f'Waiting for TF: {e}')
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

        if points_base.shape[0] < MIN_POINTS_FOR_PCA:
            self.get_logger().warn('Not enough points for PCA on basket handle')
            return

        # Compute centroid (use median for robustness)
        centroid = np.median(points_base, axis=0)

        # PCA to find principal axis (handle direction)
        pts2 = points_base - centroid
        cov = np.cov(pts2.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        principal = eigvecs[:, np.argmax(eigvals)]

        # Project principal to XY plane (we want handle direction in XY)
        handle_dir = np.array([principal[0], principal[1], 0.0])
        if np.linalg.norm(handle_dir) < 1e-6:
            self.get_logger().warn('Degenerate principal direction')
            return
        handle_dir = handle_dir / np.linalg.norm(handle_dir)

        # Desired end-effector axes: Z downwards, X along handle_dir, Y = Z x X
        Z_grasp = np.array([0.0, 0.0, -1.0])
        X_grasp = np.array([handle_dir[0], handle_dir[1], 0.0])
        X_grasp = X_grasp / np.linalg.norm(X_grasp)
        Y_grasp = np.cross(Z_grasp, X_grasp)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)

        rot = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        quat = Rotation.from_matrix(rot).as_quat()  # [x, y, z, w]

        # Build grasp pose (slightly above handle surface)
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.target_frame
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position.x = float(centroid[0])
        grasp_pose.pose.position.y = float(centroid[1])
        grasp_pose.pose.position.z = float(centroid[2] + GRASP_SURFACE_OFFSET)
        grasp_pose.pose.orientation.x = float(quat[0])
        grasp_pose.pose.orientation.y = float(quat[1])
        grasp_pose.pose.orientation.z = float(quat[2])
        grasp_pose.pose.orientation.w = float(quat[3])

        # Build an approach pose offset along -Y_grasp (so gripper comes in from outside)
        approach_pose = PoseStamped()
        approach_pose.header = grasp_pose.header
        approach_pos = np.array([grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z])
        approach_pos = approach_pos - Y_grasp * APPROACH_DISTANCE
        approach_pose.pose.position.x = float(approach_pos[0])
        approach_pose.pose.position.y = float(approach_pos[1])
        approach_pose.pose.position.z = float(approach_pos[2])
        approach_pose.pose.orientation = grasp_pose.pose.orientation

        # Lift pose after grasp (raise in world Z to avoid contents spilling)
        lift_pose = PoseStamped()
        lift_pose.header = grasp_pose.header
        lift_pose.pose.position.x = grasp_pose.pose.position.x
        lift_pose.pose.position.y = grasp_pose.pose.position.y
        lift_pose.pose.position.z = grasp_pose.pose.position.z + LIFT_HEIGHT
        lift_pose.pose.orientation = grasp_pose.pose.orientation

        # Publish approach (as grasp_pose we publish the approach pose first so pick server can move to pregrasp)
        # We publish both: the approach/pregrasp and the actual grasp contact pose on the same topic in sequence
        self.get_logger().info(f'Publishing basket grasp pose at ({grasp_pose.pose.position.x:.3f}, {grasp_pose.pose.position.y:.3f}, {grasp_pose.pose.position.z:.3f})')
        # First publish approach (pre-grasp)
        self.pose_pub.publish(approach_pose)
        # Small delay to ensure subscriber receives in order
        time.sleep(0.05)
        # Then publish the contact grasp pose
        self.pose_pub.publish(grasp_pose)

        # And publish the lift pose separately
        self.lift_pub.publish(lift_pose)


def main(args=None):
    rclpy.init(args=args)
    node = BasketHandleGraspEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

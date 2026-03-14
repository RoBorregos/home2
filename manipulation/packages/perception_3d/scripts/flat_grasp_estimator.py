#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frida_constants.vision_constants import (
    ZERO_SHOT_DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC
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


class FlatGraspEstimator(Node):
    def __init__(self):
        super().__init__('flat_grasp_estimator')
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'link_base'

        self.latest_depth = None
        self.intrinsics = None
        self.depth_frame_id = "zed_left_camera_optical_frame"

        self.target_classes = ['spoon', 'fork', 'knife']

        # Rolling buffer for table height stabilization
        self.table_height_buffer = deque(maxlen=TABLE_HEIGHT_BUFFER_SIZE)

        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(ObjectDetectionArray, ZERO_SHOT_DETECTIONS_TOPIC, self.detections_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/manipulation/flat_grasp_pose', 10)

        self.get_logger().info(f"Flat Grasp Estimator initialized. Mapping to: {self.target_frame}")

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_frame_id = msg.header.frame_id

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5]}

    def detections_callback(self, msg):
        if self.latest_depth is None or self.intrinsics is None:
            return
        for det in msg.detections:
            if det.label_text.lower() in self.target_classes:
                self.process_flat_object(det)

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
        object_mask_roi = valid_mask & (roi_depth < table_z_cam - 0.001) & (roi_depth > table_z_cam - 0.05)
        v_local, u_local = np.where(object_mask_roi)

        use_full_roi = len(v_local) < MIN_POINTS_FOR_PCA
        if use_full_roi:
            v_local, u_local = np.where(valid_mask)

        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        cx, cy = self.intrinsics['cx'], self.intrinsics['cy']
        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_3d_cam = np.vstack((x, y, z_vals)).T

        # --- TRANSFORM TO BASE FRAME ---
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF tree from camera to base... {e}")
            return

        q = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z, trans.transform.rotation.w]
        t = [trans.transform.translation.x, trans.transform.translation.y,
             trans.transform.translation.z]
        rot_mat = Rotation.from_quat(q).as_matrix()

        T_mat = np.eye(4)
        T_mat[:3, :3] = rot_mat
        T_mat[:3, 3] = t

        points_3d_hom = np.hstack((points_3d_cam, np.ones((points_3d_cam.shape[0], 1))))
        points_base = (T_mat @ points_3d_hom.T).T[:, :3]

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


def main(args=None):
    rclpy.init(args=args)
    node = FlatGraspEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

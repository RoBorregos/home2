#!/usr/bin/env python3
"""
Basket-handle grasp estimator with rim boundary extraction.

Extracts the 3D rim curve Gamma from the depth ROI using Open3D's
compute_boundary_points (angular-gap criterion on surface normals),
then derives grasp poses along the rim.

Publishes:
- /manipulation/basket_grasp_pose : grasp pose alternatives (end-effector down)
- /manipulation/basket_lift_pose  : safe lift pose after grasp

Debug topics (enabled when `publish_debug_markers` parameter is True):
- /manipulation/basket_rim_markers : MarkerArray with rim points, centroid, handle axis
- /manipulation/basket_rim_cloud   : PointCloud2 of the extracted rim points
- /manipulation/basket_roi_cloud   : PointCloud2 of the raw ROI cloud (pre-boundary)

Parameters:
- publish_debug_markers (bool, default True)   : publish RViz visualization
- save_debug_pcd       (bool, default False)   : dump per-frame .pcd to disk
- debug_pcd_dir        (str, default /tmp/basket_debug)
- use_rim_boundary     (bool, default True)    : when False, runs old PCA-on-ROI path
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped, Point as PointMsg
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import open3d as o3d

from frida_constants.vision_constants import (
    ZERO_SHOT_DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray


# Grasp geometry
GRASP_SURFACE_OFFSET = 0.02   # meters above rim
LIFT_HEIGHT = 0.20            # meters
HANDLE_OFFSET_DIST = 0.03     # meters: side-shift alternatives

# Boundary extraction
VOXEL_SIZE = 0.003                  # m -- downsample before normal/boundary
NORMAL_SEARCH_RADIUS = 0.015        # m
NORMAL_MAX_NN = 30
BOUNDARY_SEARCH_RADIUS = 0.020      # m
BOUNDARY_MAX_NN = 30
BOUNDARY_ANGLE_THRESHOLD = 90.0     # degrees -- angular gap criterion

# A rim sits at the top of the basket (in robot Z). Restrict boundary search
# to the upper band to reject inside-floor edges, holes, and bottom rims.
RIM_TOP_BAND = 0.04                 # m -- keep points within this band below max Z

MIN_POINTS_FOR_BOUNDARY = 200
MIN_RIM_POINTS = 20


class BasketHandleGraspEstimator(Node):
    def __init__(self):
        super().__init__('basket_handle_grasp_estimator')

        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('save_debug_pcd', False)
        self.declare_parameter('debug_pcd_dir', '/tmp/basket_debug')
        self.declare_parameter('use_rim_boundary', True)
        self.publish_debug = self.get_parameter('publish_debug_markers').value
        self.save_pcd = self.get_parameter('save_debug_pcd').value
        self.debug_dir = self.get_parameter('debug_pcd_dir').value
        self.use_rim_boundary = self.get_parameter('use_rim_boundary').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'link_base'
        self.depth_frame_id = 'zed_left_camera_optical_frame'

        self.latest_depth = None
        self.intrinsics = None

        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(ObjectDetectionArray, ZERO_SHOT_DETECTIONS_TOPIC, self.detections_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/manipulation/basket_grasp_pose', 10)
        self.lift_pub = self.create_publisher(PoseStamped, '/manipulation/basket_lift_pose', 10)

        self.rim_marker_pub = self.create_publisher(MarkerArray, '/manipulation/basket_rim_markers', 5)
        self.rim_cloud_pub = self.create_publisher(PointCloud2, '/manipulation/basket_rim_cloud', 5)
        self.roi_cloud_pub = self.create_publisher(PointCloud2, '/manipulation/basket_roi_cloud', 5)

        self._frame_count = 0
        if self.save_pcd:
            os.makedirs(self.debug_dir, exist_ok=True)

        self.get_logger().info(
            f'Basket Handle Grasp Estimator initialized '
            f'(rim_boundary={self.use_rim_boundary}, debug={self.publish_debug}, '
            f'save_pcd={self.save_pcd})'
        )

    # ---------- subscribers ----------

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4],
                               'cx': msg.k[2], 'cy': msg.k[5]}

    def detections_callback(self, msg: ObjectDetectionArray):
        if self.latest_depth is None or self.intrinsics is None:
            return
        for det in msg.detections:
            label = det.label_text.lower() if hasattr(det, 'label_text') else ''
            if 'basket' in label or 'casket' in label or 'basket_handle' in label:
                try:
                    self.process_basket_detection(det)
                except Exception as e:
                    self.get_logger().error(f'Error processing detection: {e}')

    # ---------- ROI -> base-frame points ----------

    def _roi_pixels_to_base(self, detection):
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

        roi_depth = self.latest_depth[ymin:ymax, xmin:xmax]
        valid_mask = (roi_depth > 0) & (~np.isnan(roi_depth))
        if not np.any(valid_mask):
            return None

        v_local, u_local = np.where(valid_mask)
        u_global = u_local + xmin
        v_global = v_local + ymin
        z_vals = roi_depth[v_local, u_local]

        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        cx, cy = self.intrinsics['cx'], self.intrinsics['cy']
        x = (u_global - cx) * z_vals / fx
        y = (v_global - cy) * z_vals / fy
        points_cam = np.vstack((x, y, z_vals)).T

        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f'Waiting for TF: {e}')
            return None

        q = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z, trans.transform.rotation.w]
        t = [trans.transform.translation.x, trans.transform.translation.y,
             trans.transform.translation.z]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = t
        homog = np.hstack((points_cam, np.ones((points_cam.shape[0], 1))))
        return (T @ homog.T).T[:, :3]

    # ---------- rim extraction ----------

    def _extract_rim_boundary(self, points_base):
        """Open3D Tensor-API boundary extraction. Returns (rim_pts, centroid) or
        (None, None) if extraction is not reliable."""
        if points_base.shape[0] < MIN_POINTS_FOR_BOUNDARY:
            self.get_logger().warn(
                f'Not enough ROI points for boundary extraction: {points_base.shape[0]}'
            )
            return None, None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_base)
        pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=NORMAL_SEARCH_RADIUS, max_nn=NORMAL_MAX_NN,
            )
        )
        # Rim normals point roughly up in base frame; orient consistently so
        # the angular-gap test sees coherent neighborhoods.
        pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

        try:
            t_pcd = o3d.t.geometry.PointCloud.from_legacy(pcd)
            t_boundary, _mask = t_pcd.compute_boundary_points(
                radius=BOUNDARY_SEARCH_RADIUS,
                max_nn=BOUNDARY_MAX_NN,
                angle_threshold=BOUNDARY_ANGLE_THRESHOLD,
            )
            boundary_pts = t_boundary.point.positions.numpy()
        except Exception as e:
            self.get_logger().warn(f'compute_boundary_points failed: {e}')
            return None, None

        if boundary_pts.shape[0] < MIN_RIM_POINTS:
            self.get_logger().warn(
                f'Boundary extractor returned too few points: {boundary_pts.shape[0]}'
            )
            return None, None

        z_max = float(np.max(boundary_pts[:, 2]))
        rim_mask = boundary_pts[:, 2] >= (z_max - RIM_TOP_BAND)
        rim_pts = boundary_pts[rim_mask]

        if rim_pts.shape[0] < MIN_RIM_POINTS:
            self.get_logger().warn(
                f'Top-band rim filter left too few points: {rim_pts.shape[0]}'
            )
            return None, None

        centroid = np.median(rim_pts, axis=0)

        if self.save_pcd:
            try:
                fn_in = os.path.join(self.debug_dir, f'roi_{self._frame_count:04d}.pcd')
                fn_rim = os.path.join(self.debug_dir, f'rim_{self._frame_count:04d}.pcd')
                o3d.io.write_point_cloud(fn_in, pcd)
                rim_pcd = o3d.geometry.PointCloud()
                rim_pcd.points = o3d.utility.Vector3dVector(rim_pts)
                o3d.io.write_point_cloud(fn_rim, rim_pcd)
                self.get_logger().info(f'Saved debug clouds: {fn_in}, {fn_rim}')
                self._frame_count += 1
            except Exception as e:
                self.get_logger().warn(f'Save pcd failed: {e}')

        return rim_pts, centroid

    def _principal_axis_xy(self, pts, centroid):
        """In-plane PCA on (pts - centroid). Returns unit vector in XY or None."""
        pts2 = pts[:, :2] - centroid[:2]
        if pts2.shape[0] < 3:
            return None
        cov = np.cov(pts2.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        principal = eigvecs[:, np.argmax(eigvals)]
        handle_dir = np.array([float(principal[0]), float(principal[1]), 0.0])
        n = np.linalg.norm(handle_dir)
        if n < 1e-6:
            return None
        return handle_dir / n

    # ---------- pose publication ----------

    def _publish_grasps(self, centroid, handle_dir):
        Z_grasp = np.array([0.0, 0.0, -1.0])
        X_grasp = np.array([handle_dir[0], handle_dir[1], 0.0])
        X_grasp = X_grasp / np.linalg.norm(X_grasp)
        Y_grasp = np.cross(Z_grasp, X_grasp)
        Y_grasp = Y_grasp / np.linalg.norm(Y_grasp)

        rot_normal = np.column_stack((X_grasp, Y_grasp, Z_grasp))
        quat_normal = Rotation.from_matrix(rot_normal).as_quat()
        rot_reversed = np.column_stack((-X_grasp, -Y_grasp, Z_grasp))
        quat_reversed = Rotation.from_matrix(rot_reversed).as_quat()

        def create_pose(position_offset, quat):
            p = PoseStamped()
            p.header.frame_id = self.target_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = float(centroid[0] + position_offset[0])
            p.pose.position.y = float(centroid[1] + position_offset[1])
            p.pose.position.z = float(centroid[2] + GRASP_SURFACE_OFFSET)
            p.pose.orientation.x = float(quat[0])
            p.pose.orientation.y = float(quat[1])
            p.pose.orientation.z = float(quat[2])
            p.pose.orientation.w = float(quat[3])
            return p

        pose_center = create_pose([0.0, 0.0], quat_normal)
        pose_center_rev = create_pose([0.0, 0.0], quat_reversed)
        pose_offset_pos = create_pose(X_grasp * HANDLE_OFFSET_DIST, quat_normal)
        pose_offset_neg = create_pose(-X_grasp * HANDLE_OFFSET_DIST, quat_normal)

        self.get_logger().info('Publishing basket grasp alternatives...')
        self.pose_pub.publish(pose_center)
        self.pose_pub.publish(pose_center_rev)
        self.pose_pub.publish(pose_offset_pos)
        self.pose_pub.publish(pose_offset_neg)

        lift_pose = PoseStamped()
        lift_pose.header = pose_center.header
        lift_pose.pose.position.x = pose_center.pose.position.x
        lift_pose.pose.position.y = pose_center.pose.position.y
        lift_pose.pose.position.z = pose_center.pose.position.z + LIFT_HEIGHT
        lift_pose.pose.orientation = pose_center.pose.orientation
        self.lift_pub.publish(lift_pose)

    # ---------- debug ----------

    def _xyz_to_cloud(self, pts, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, pts.astype(np.float32))

    def _publish_debug(self, roi_pts, rim_pts, centroid, handle_dir, source: str):
        stamp = self.get_clock().now().to_msg()
        try:
            self.roi_cloud_pub.publish(self._xyz_to_cloud(roi_pts, stamp))
            self.rim_cloud_pub.publish(self._xyz_to_cloud(rim_pts, stamp))
        except Exception as e:
            self.get_logger().warn(f'Debug cloud publish failed: {e}')

        markers = MarkerArray()

        rim_marker = Marker()
        rim_marker.header.frame_id = self.target_frame
        rim_marker.header.stamp = stamp
        rim_marker.ns = 'basket_rim'
        rim_marker.id = 0
        rim_marker.type = Marker.SPHERE_LIST
        rim_marker.action = Marker.ADD
        rim_marker.pose.orientation.w = 1.0
        rim_marker.scale.x = rim_marker.scale.y = rim_marker.scale.z = 0.008
        # Green = boundary extractor, yellow = fallback PCA
        if source == 'rim_boundary':
            rim_marker.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
        else:
            rim_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.9)
        for p in rim_pts:
            rim_marker.points.append(PointMsg(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        markers.markers.append(rim_marker)

        c_marker = Marker()
        c_marker.header.frame_id = self.target_frame
        c_marker.header.stamp = stamp
        c_marker.ns = 'basket_rim'
        c_marker.id = 1
        c_marker.type = Marker.SPHERE
        c_marker.action = Marker.ADD
        c_marker.pose.position.x = float(centroid[0])
        c_marker.pose.position.y = float(centroid[1])
        c_marker.pose.position.z = float(centroid[2])
        c_marker.pose.orientation.w = 1.0
        c_marker.scale.x = c_marker.scale.y = c_marker.scale.z = 0.035
        c_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.95)
        markers.markers.append(c_marker)

        axis_marker = Marker()
        axis_marker.header.frame_id = self.target_frame
        axis_marker.header.stamp = stamp
        axis_marker.ns = 'basket_rim'
        axis_marker.id = 2
        axis_marker.type = Marker.ARROW
        axis_marker.action = Marker.ADD
        axis_marker.pose.orientation.w = 1.0
        axis_marker.scale.x = 0.005   # shaft diameter
        axis_marker.scale.y = 0.012   # head diameter
        axis_marker.scale.z = 0.012   # head length
        axis_marker.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.95)
        L = 0.08
        axis_marker.points.append(PointMsg(
            x=float(centroid[0] - handle_dir[0] * L),
            y=float(centroid[1] - handle_dir[1] * L),
            z=float(centroid[2]),
        ))
        axis_marker.points.append(PointMsg(
            x=float(centroid[0] + handle_dir[0] * L),
            y=float(centroid[1] + handle_dir[1] * L),
            z=float(centroid[2]),
        ))
        markers.markers.append(axis_marker)

        text_marker = Marker()
        text_marker.header.frame_id = self.target_frame
        text_marker.header.stamp = stamp
        text_marker.ns = 'basket_rim'
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(centroid[0])
        text_marker.pose.position.y = float(centroid[1])
        text_marker.pose.position.z = float(centroid[2] + 0.08)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.025
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f'{source} | rim_pts={len(rim_pts)}'
        markers.markers.append(text_marker)

        self.rim_marker_pub.publish(markers)

    # ---------- main pipeline ----------

    def process_basket_detection(self, detection):
        points_base = self._roi_pixels_to_base(detection)
        if points_base is None or points_base.shape[0] < 3:
            return

        rim_pts, centroid, source = None, None, None

        if self.use_rim_boundary:
            rim_pts, centroid = self._extract_rim_boundary(points_base)
            if rim_pts is not None:
                source = 'rim_boundary'

        if rim_pts is None:
            # Fallback: original visible-point centroid + ROI PCA
            self.get_logger().warn('Falling back to PCA-on-ROI (legacy path)')
            centroid = np.median(points_base, axis=0)
            rim_pts = points_base
            source = 'pca_fallback'

        handle_dir = self._principal_axis_xy(rim_pts, centroid)
        if handle_dir is None:
            self.get_logger().warn('Could not derive handle direction')
            return

        self._publish_grasps(centroid, handle_dir)

        if self.publish_debug:
            try:
                self._publish_debug(points_base, rim_pts, centroid, handle_dir, source)
            except Exception as e:
                self.get_logger().warn(f'Debug publish failed: {e}')


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

#!/usr/bin/env python3
"""
GraspNet-baseline grasp detection service for ROS2.
Uses the open-source graspnet-baseline: https://github.com/graspnet/graspnet-baseline
Replaces GPD (C++) with a persistent Python node — no restart needed between calls.
"""

import numpy as np
import rclpy
import tf2_ros
import torch
from frida_constants.manipulation_constants import GRASP_DETECTION_SERVICE
from frida_interfaces.srv import GraspDetection
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

GRASP_POINTCLOUD_TOPIC = "/manipulation/grasp_pcl"
GRASP_MARKER_TOPIC = "/manipulation/grasp_markers"

# Number of points GraspNet expects as input
NUM_POINT = 20000


class GraspNetService(Node):
    def __init__(self):
        super().__init__("grasp_detection_service")

        self.declare_parameter("target_frame", "link_base")
        self.declare_parameter("transform_timeout", 1.0)
        self.declare_parameter(
            "checkpoint_path",
            "/workspace/src/manipulation/packages/arm_pkg/graspnet_checkpoint/checkpoint-rs.tar",
        )
        self.declare_parameter("num_view", 300)
        self.declare_parameter("collision_thresh", 0.01)
        self.declare_parameter("voxel_size", 0.01)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pcd_pub = self.create_publisher(PointCloud2, GRASP_POINTCLOUD_TOPIC, 10)
        self.marker_pub = self.create_publisher(MarkerArray, GRASP_MARKER_TOPIC, 10)

        self._load_graspnet()

        self.srv = self.create_service(
            GraspDetection, GRASP_DETECTION_SERVICE, self.handle_service
        )
        self.get_logger().info("GraspNet service ready")

    def _load_graspnet(self):
        # Import from graspnet-baseline (must be on PYTHONPATH)
        from graspnet import GraspNet, pred_decode

        self._pred_decode = pred_decode

        num_view = self.get_parameter("num_view").get_parameter_value().integer_value
        checkpoint_path = (
            self.get_parameter("checkpoint_path").get_parameter_value().string_value
        )

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.net = GraspNet(
            input_feature_dim=0,
            num_view=num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        self.net.to(self.device)

        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.net.load_state_dict(checkpoint["model_state_dict"])
        self.net.eval()
        self.get_logger().info(f"GraspNet model loaded from {checkpoint_path}")

    def _get_transform_matrix(self, source_frame, target_frame, stamp):
        timeout = (
            self.get_parameter("transform_timeout").get_parameter_value().double_value
        )
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                rclpy.duration.Duration(seconds=timeout),
            )
            trans = t.transform.translation
            rot = t.transform.rotation
            R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            T = np.eye(4, dtype=np.float32)
            T[:3, :3] = R
            T[:3, 3] = [trans.x, trans.y, trans.z]
            return T
        except TransformException as ex:
            self.get_logger().error(f"TF Error: {ex}")
            return None

    def _parse_pointcloud(self, cloud_msg):
        """Parse PointCloud2 → (N,3) float32 points and (N,3) float32 colors [0,1]."""
        field_names = [f.name for f in cloud_msg.fields]

        if "rgb" in field_names:
            gen = pc2.read_points(
                cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
            )
            data = list(gen)
            if not data:
                return None, None
            pts = np.array([[p[0], p[1], p[2]] for p in data], dtype=np.float32)
            rgb_raw = np.array([p[3] for p in data], dtype=np.float32)
            rgb_bytes = rgb_raw.view(np.uint8).reshape(-1, 4)
            colors = rgb_bytes[:, [2, 1, 0]].astype(np.float32) / 255.0
        elif "r" in field_names and "g" in field_names and "b" in field_names:
            gen = pc2.read_points(
                cloud_msg, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True
            )
            data = list(gen)
            if not data:
                return None, None
            arr = np.array(data, dtype=np.float32)
            pts = arr[:, :3]
            colors = arr[:, 3:6] / 255.0
        else:
            gen = pc2.read_points(
                cloud_msg, field_names=("x", "y", "z"), skip_nans=True
            )
            data = list(gen)
            if not data:
                return None, None
            pts = np.array(data, dtype=np.float32)
            colors = np.full((len(pts), 3), 0.5, dtype=np.float32)

        return pts, colors

    def _sample_points(self, pts, colors):
        """Sample or upsample to NUM_POINT points as GraspNet expects."""
        n = len(pts)
        if n >= NUM_POINT:
            idxs = np.random.choice(n, NUM_POINT, replace=False)
        else:
            idxs1 = np.arange(n)
            idxs2 = np.random.choice(n, NUM_POINT - n, replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        return pts[idxs], colors[idxs]

    def handle_service(self, req, res):
        target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        try:
            if not req.input_cloud.data:
                res.success = False
                res.message = "No input cloud provided"
                return res

            cloud_msg = req.input_cloud
            source_frame = cloud_msg.header.frame_id
            stamp = cloud_msg.header.stamp

            pts, colors = self._parse_pointcloud(cloud_msg)
            if pts is None or len(pts) == 0:
                res.success = False
                res.message = "Empty point cloud"
                return res

            # Transform to target frame
            if source_frame != target_frame:
                T = self._get_transform_matrix(source_frame, target_frame, stamp)
                if T is None:
                    res.success = False
                    res.message = "TF transform failed"
                    return res
                pts_h = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])
                pts = (T @ pts_h.T).T[:, :3]

            # Publish transformed cloud for visualization
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = target_frame
            self.pcd_pub.publish(pc2.create_cloud_xyz32(header, pts.tolist()))

            self.get_logger().info(f"Running GraspNet on {len(pts)} points")

            pts_sampled, colors_sampled = self._sample_points(pts, colors)

            # Build input tensors
            cloud_tensor = torch.from_numpy(pts_sampled[np.newaxis]).to(self.device)
            color_tensor = torch.from_numpy(colors_sampled[np.newaxis]).to(self.device)

            end_points = {
                "point_clouds": cloud_tensor,
                "cloud_colors": color_tensor,
            }

            with torch.no_grad():
                end_points = self.net(end_points)
                grasp_preds = self._pred_decode(end_points)

            gg_array = grasp_preds[0].detach().cpu().numpy()

            from graspnetAPI import GraspGroup

            gg = GraspGroup(gg_array)

            # Optional collision filtering against the full object cloud
            collision_thresh = (
                self.get_parameter("collision_thresh")
                .get_parameter_value()
                .double_value
            )
            voxel_size = (
                self.get_parameter("voxel_size").get_parameter_value().double_value
            )
            if collision_thresh > 0:
                try:
                    from collision_detector import ModelFreeCollisionDetector

                    mfcdetector = ModelFreeCollisionDetector(pts, voxel_size=voxel_size)
                    collision_mask = mfcdetector.detect(
                        gg, approach_dist=0.05, collision_thresh=collision_thresh
                    )
                    gg = gg[~collision_mask]
                except ImportError:
                    self.get_logger().warn(
                        "collision_detector not found, skipping collision filtering"
                    )

            if len(gg) == 0:
                res.success = False
                res.message = "No grasps detected"
                return res

            gg = gg.nms().sort_by_score()
            self.get_logger().info(f"Detected {len(gg)} grasps after NMS")

            # Rotation correction: GraspNet approach direction is along X-axis,
            # the xarm EE expects approach along Z-axis → rotate 90° around Y
            correction = Rotation.from_euler("y", np.pi / 2)

            for i in range(len(gg)):
                grasp = gg[i]
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = target_frame

                t = grasp.translation
                pose.pose.position.x = float(t[0])
                pose.pose.position.y = float(t[1])
                pose.pose.position.z = float(t[2])

                rot = Rotation.from_matrix(grasp.rotation_matrix)
                q = (rot * correction).as_quat()  # [x, y, z, w]
                pose.pose.orientation.x = float(q[0])
                pose.pose.orientation.y = float(q[1])
                pose.pose.orientation.z = float(q[2])
                pose.pose.orientation.w = float(q[3])

                res.grasp_poses.append(pose)
                res.grasp_scores.append(float(grasp.score))

            self.marker_pub.publish(
                self._create_markers(res.grasp_poses, res.grasp_scores)
            )
            res.success = True
            return res

        except Exception as e:
            res.success = False
            res.message = f"Error: {e}"
            self.get_logger().error(f"Service failed: {e}")
            import traceback

            self.get_logger().error(traceback.format_exc())
            return res

    def _create_markers(self, poses, scores):
        markers = MarkerArray()
        max_score = max(scores) if scores else 1.0
        for i, (pose, score) in enumerate(zip(poses, scores)):
            norm = score / max_score if max_score > 0 else 0.5
            m = Marker()
            m.header = pose.header
            m.ns = f"grasp_{i}"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = pose.pose
            m.scale.x = 0.1
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.color.r = float(1.0 - norm)
            m.color.g = float(norm)
            m.color.b = 0.1
            m.color.a = 0.8
            m.lifetime.sec = 10
            markers.markers.append(m)
        return markers


def main():
    rclpy.init()
    node = GraspNetService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

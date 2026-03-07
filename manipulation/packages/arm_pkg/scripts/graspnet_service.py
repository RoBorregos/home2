#!/usr/bin/env python3
"""
Contact-GraspNet service for ROS2 — drop-in replacement for gpd_service.cpp.

Key improvements over GPD:
  - Python (no C++ compilation / GPD library needed)
  - Stays alive between calls (no respawn needed)
  - GPU accelerated via TensorFlow
  - No TF-frame hacks: grasps are returned in whatever frame the cloud is in

Setup (run once):
  git clone https://github.com/NVlabs/contact_graspnet \
      manipulation/packages/contact_graspnet
  pip install tensorflow trimesh scipy

  # Download checkpoints per Contact-GraspNet README, place them at:
  #   manipulation/packages/contact_graspnet/checkpoints/
  #       scene_test_2048_bs3_hor_sigma_001/

Parameters (ROS2):
  target_frame      (str)   Frame to transform cloud into before inference.
                            Default: "link_base"
  checkpoint_dir    (str)   Absolute path to Contact-GraspNet checkpoint dir.
  num_points        (int)   Cloud is subsampled to this many points. Default: 20000
  num_grasps        (int)   Max grasps returned per call. Default: 10
  forward_passes    (int)   GraspNet forward passes (more = slower, more grasps). Default: 1
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_utils
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from frida_interfaces.srv import GraspDetection
from scipy.spatial.transform import Rotation as SciRot

_DEFAULT_GRASPNET_SRC = "/workspace/src/manipulation/packages/contact_graspnet"
_DEFAULT_CHECKPOINT = os.path.join(
    _DEFAULT_GRASPNET_SRC,
    "checkpoints",
    "scene_test_2048_bs3_hor_sigma_001",
)

GRASP_DETECTION_SERVICE = "detect_grasps"
GRASP_MARKER_TOPIC = "/grasp_markers"
GRASP_POINTCLOUD_TOPIC = "/grasp_pointcloud"


class GraspNetService(Node):
    def __init__(self):
        super().__init__("graspnet_service")

        self.declare_parameter("target_frame", "link_base")
        self.declare_parameter("checkpoint_dir", _DEFAULT_CHECKPOINT)
        self.declare_parameter("num_points", 20000)
        self.declare_parameter("num_grasps", 10)
        self.declare_parameter("forward_passes", 1)

        self._target_frame = self.get_parameter("target_frame").value
        self._checkpoint_dir = self.get_parameter("checkpoint_dir").value
        self._num_points = self.get_parameter("num_points").value
        self._num_grasps = self.get_parameter("num_grasps").value
        self._forward_passes = self.get_parameter("forward_passes").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, GRASP_MARKER_TOPIC, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, GRASP_POINTCLOUD_TOPIC, 10)

        self._session = None
        self._grasp_estimator = None
        self._load_model()

        self._service = self.create_service(
            GraspDetection, GRASP_DETECTION_SERVICE, self._handle_service
        )

        self.get_logger().info(
            f"GraspNet service ready on /{GRASP_DETECTION_SERVICE} "
            f"(target_frame={self._target_frame}, num_grasps={self._num_grasps})"
        )

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_model(self):
        """Load Contact-GraspNet once at startup — stays in memory for all calls."""
        graspnet_src = os.path.dirname(
            self._checkpoint_dir.rstrip("/").rsplit("/checkpoints", 1)[0]
            + "/checkpoints"
        )
        # Fallback to default src path if checkpoint_dir is inside the default location
        graspnet_src = _DEFAULT_GRASPNET_SRC

        try:
            if graspnet_src not in sys.path:
                sys.path.insert(0, graspnet_src)

            import tensorflow.compat.v1 as tf  # Contact-GraspNet uses TF1 API

            tf.disable_eager_execution()

            from contact_graspnet.contact_grasp_estimator import GraspEstimator
            from contact_graspnet import config_utils

            global_config = config_utils.load_config(
                self._checkpoint_dir, batch_size=1, arg_configs=[]
            )
            self._grasp_estimator = GraspEstimator(global_config)
            self._grasp_estimator.build_network()

            saver = tf.train.Saver(save_relative_paths=True)
            self._session = tf.Session()
            self._grasp_estimator.load_weights(
                self._session, saver, self._checkpoint_dir
            )

            self.get_logger().info(
                f"Contact-GraspNet loaded from: {self._checkpoint_dir}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to load GraspNet model: {e}\n"
                "Make sure contact_graspnet is cloned and checkpoints are downloaded."
            )
            self._grasp_estimator = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transform_cloud(self, cloud_msg: PointCloud2) -> PointCloud2 | None:
        """Transform PointCloud2 to self._target_frame. Returns None on failure."""
        if cloud_msg.header.frame_id == self._target_frame:
            return cloud_msg
        try:
            from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return do_transform_cloud(cloud_msg, transform)
        except TransformException as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def _cloud_to_numpy(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to (N, 3) float32 array, dropping NaNs."""
        pts = list(
            pc2_utils.read_points(
                cloud_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        if not pts:
            return np.zeros((0, 3), dtype=np.float32)
        return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)

    def _subsample(self, points: np.ndarray, n: int) -> np.ndarray:
        if len(points) <= n:
            return points
        idx = np.random.choice(len(points), n, replace=False)
        return points[idx]

    def _matrix_to_pose(self, mat: np.ndarray, frame_id: str, stamp) -> PoseStamped:
        """Convert 4×4 homogeneous grasp matrix → PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(mat[0, 3])
        pose.pose.position.y = float(mat[1, 3])
        pose.pose.position.z = float(mat[2, 3])
        q = SciRot.from_matrix(mat[:3, :3]).as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        return pose

    def _publish_markers(self, poses, scores, frame_id):
        markers = MarkerArray()
        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (pose_stamped, score) in enumerate(zip(poses, scores)):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "graspnet"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = pose_stamped.pose
            m.scale.x = 0.1
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.color.a = 0.8
            # Green → red based on score rank
            m.color.r = float(1.0 - score)
            m.color.g = float(score)
            m.color.b = 0.0
            m.lifetime.sec = 10
            markers.markers.append(m)

        self._marker_pub.publish(markers)

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def _handle_service(
        self,
        request: GraspDetection.Request,
        response: GraspDetection.Response,
    ) -> GraspDetection.Response:
        if self._grasp_estimator is None:
            response.success = False
            response.message = "GraspNet model not loaded — check logs for details."
            return response

        if not request.input_cloud.data:
            response.success = False
            response.message = "Empty input cloud."
            return response

        # 1. Transform to robot frame
        cloud_msg = self._transform_cloud(request.input_cloud)
        if cloud_msg is None:
            response.success = False
            response.message = "TF transform failed."
            return response

        self._cloud_pub.publish(cloud_msg)

        # 2. Convert to numpy
        points = self._cloud_to_numpy(cloud_msg)
        if len(points) < 10:
            response.success = False
            response.message = f"Cloud too small: {len(points)} points."
            return response

        # 3. Subsample
        points = self._subsample(points, self._num_points)
        self.get_logger().info(
            f"Running Contact-GraspNet on {len(points)} points "
            f"(forward_passes={self._forward_passes})..."
        )

        # 4. Inference
        try:
            pred_grasps, scores, _, _ = self._grasp_estimator.predict_scene_grasps(
                self._session,
                points,
                pc_segments={0: np.arange(len(points))},
                local_regions=True,
                filter_grasps=True,
                forward_passes=self._forward_passes,
            )
        except Exception as e:
            self.get_logger().error(f"GraspNet inference error: {e}")
            response.success = False
            response.message = f"Inference error: {e}"
            return response

        # 5. Flatten across segments and sort by score
        all_grasps = []
        all_scores = []
        for seg_id in pred_grasps:
            for mat, score in zip(pred_grasps[seg_id], scores[seg_id]):
                all_grasps.append(mat)
                all_scores.append(float(score))

        if not all_grasps:
            response.success = False
            response.message = "No grasps found."
            return response

        # Sort descending by score, take top N
        sorted_pairs = sorted(
            zip(all_scores, all_grasps), key=lambda x: x[0], reverse=True
        )[: self._num_grasps]

        stamp = cloud_msg.header.stamp
        for score, mat in sorted_pairs:
            response.grasp_poses.append(
                self._matrix_to_pose(mat, self._target_frame, stamp)
            )
            response.grasp_scores.append(score)

        self._publish_markers(
            response.grasp_poses, response.grasp_scores, self._target_frame
        )

        self.get_logger().info(
            f"Returning {len(response.grasp_poses)} grasps "
            f"(top score: {response.grasp_scores[0]:.3f})."
        )
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetService()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

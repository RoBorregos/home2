#!/usr/bin/env python3
"""
Setup: run `docker/manipulation/setup_giga.sh` first (submodule init, `vgn`
python package install, ConvONets extension build, and manual pretrained-
checkpoint download instructions
"""

import pathlib
import types

import numpy as np
import rclpy
from frida_constants.manipulation_constants import (
    GRASP_DETECTION_SERVICE,
    GRASP_MARKER_TOPIC,
    GRASP_POINTCLOUD_TOPIC,
)
from frida_interfaces.srv import GraspDetection
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from giga_grasp_detection.tsdf_utils import (
    build_gripper_markers,
    giga_grasp_to_pose_stamped,
    points_to_tsdf_grid,
)

_DEFAULT_MODEL_PATH = (
    "/workspace/src/manipulation/packages/giga_grasp_detection/models/vgn_conv.pth"
)

_DEFAULT_QUAL_TH = 0.9  # VGNImplicit's own default quality threshold
_DEFAULT_WORKSPACE_SIZE = 0.3  # meters; GIGA/VGN training workspace cube edge
_DEFAULT_RESOLUTION = 40  # voxel grid side; fixed by the checkpoint architecture
_QOS_DEPTH = 10


class GigaGraspDetectionService(Node):
    def __init__(self):
        super().__init__("giga_grasp_detection_service")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("pcd_default_frame", "base_link")
        self.declare_parameter("transform_timeout", 1.0)
        self.declare_parameter("model_path", _DEFAULT_MODEL_PATH)
        self.declare_parameter("model_type", "vgn")
        self.declare_parameter("qual_th", _DEFAULT_QUAL_TH)
        self.declare_parameter("workspace_size", _DEFAULT_WORKSPACE_SIZE)
        self.declare_parameter("resolution", _DEFAULT_RESOLUTION)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pcd_pub = self.create_publisher(
            PointCloud2, GRASP_POINTCLOUD_TOPIC, _QOS_DEPTH
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, GRASP_MARKER_TOPIC, _QOS_DEPTH
        )

        self.detector = None
        self.model_load_error = None
        self._load_model()

        self.srv = self.create_service(
            GraspDetection, GRASP_DETECTION_SERVICE, self.handle_service
        )
        self.get_logger().info("GIGA grasp detection service ready")

    def _load_model(self):
        model_path = pathlib.Path(
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        model_type = self.get_parameter("model_type").get_parameter_value().string_value
        qual_th = self.get_parameter("qual_th").get_parameter_value().double_value
        resolution = (
            self.get_parameter("resolution").get_parameter_value().integer_value
        )

        if not model_path.exists():
            self.model_load_error = (
                f"GIGA checkpoint not found at {model_path}. Run "
                "docker/manipulation/setup_giga.sh and follow its printed manual "
                "checkpoint-download instructions, then set the 'model_path' "
                "param to the actual downloaded filename."
            )
            self.get_logger().error(self.model_load_error)
            return

        try:
            # Imported lazily so a missing `vgn` install fails per-request with
            # a clear service error, not by crashing the whole node at launch.
            from vgn.detection_implicit import VGNImplicit

            self.detector = VGNImplicit(
                model_path,
                model_type,
                qual_th=qual_th,
                resolution=resolution,
                best=True,
            )
        except Exception as exc:  # noqa: BLE001 -- surface any load failure as a clear service error, not a crash
            self.model_load_error = f"Failed to load GIGA model: {exc}"
            self.get_logger().error(self.model_load_error)

    def _load_cloud_points(self, req) -> "tuple[np.ndarray, str, object]":
        """Return (points_Nx3, source_frame, stamp) from either input_cloud or
        pcd_path, or raise ValueError with a user-facing message."""
        if req.input_cloud.data:
            points = point_cloud2.read_points_numpy(
                req.input_cloud, field_names=("x", "y", "z"), skip_nans=True
            )
            return (
                np.asarray(points, dtype=np.float64).reshape(-1, 3),
                req.input_cloud.header.frame_id,
                req.input_cloud.header.stamp,
            )

        if req.pcd_path:
            try:
                import open3d as o3d
            except ImportError as exc:
                raise ValueError(
                    "pcd_path given but open3d isn't installed -- run "
                    "docker/manipulation/setup_giga.sh"
                ) from exc
            cloud = o3d.io.read_point_cloud(req.pcd_path)
            points = np.asarray(cloud.points, dtype=np.float64)
            if points.size == 0:
                raise ValueError(f"PCD load failed or empty: {req.pcd_path}")
            pcd_default_frame = (
                self.get_parameter("pcd_default_frame")
                .get_parameter_value()
                .string_value
            )
            return points, pcd_default_frame, self.get_clock().now().to_msg()

        raise ValueError("No input")

    def _transform_points(
        self, points: np.ndarray, source_frame: str, stamp, target_frame: str
    ) -> np.ndarray:
        if source_frame == target_frame:
            return points
        timeout = (
            self.get_parameter("transform_timeout").get_parameter_value().double_value
        )
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp) if hasattr(stamp, "sec") else Time(),
            rclpy.duration.Duration(seconds=timeout),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        return rot.apply(points) + np.array([t.x, t.y, t.z])

    def handle_service(self, req, res):
        target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        if req.cfg_path:
            # Baseline: unlike the old GPD service (whose cfg_path pointed at a
            # gflags-style .cfg with hand geometry / network weights params),
            # this node's equivalent settings are ROS params (model_path,
            # model_type, qual_th, workspace_size, resolution) set at launch,
            # not per-request. cfg_path is accepted for interface compatibility
            # and logged, not parsed -- deliberately deferred rather than
            # half-implemented for this baseline.
            self.get_logger().info(
                f"cfg_path='{req.cfg_path}' received but ignored by this "
                "baseline -- see module docstring."
            )

        if self.detector is None:
            res.success = False
            res.message = self.model_load_error or "GIGA model not loaded"
            return res

        try:
            points, source_frame, stamp = self._load_cloud_points(req)
        except ValueError as exc:
            res.success = False
            res.message = str(exc)
            return res

        try:
            points = self._transform_points(points, source_frame, stamp, target_frame)
        except TransformException as exc:
            res.success = False
            res.message = f"Transform failed: {exc}"
            return res

        header = Header()
        header.frame_id = target_frame
        header.stamp = self.get_clock().now().to_msg()
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        self.pcd_pub.publish(cloud_msg)

        workspace_size = (
            self.get_parameter("workspace_size").get_parameter_value().double_value
        )
        resolution = (
            self.get_parameter("resolution").get_parameter_value().integer_value
        )
        grid, origin = points_to_tsdf_grid(
            points, size=workspace_size, resolution=resolution
        )

        state = types.SimpleNamespace(tsdf=grid)

        try:
            grasps, scores, _elapsed = self.detector(state)
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = f"GIGA inference failed: {exc}"
            self.get_logger().error(res.message)
            return res

        order = np.argsort(scores)[::-1] if len(scores) else []
        poses = []
        widths = []
        final_scores = []
        for i in order:
            grasp = grasps[i]
            pose = giga_grasp_to_pose_stamped(
                grasp, origin, target_frame, cloud_msg.header.stamp
            )
            poses.append(pose)
            widths.append(grasp.width)
            final_scores.append(float(scores[i]))

        res.success = True
        res.message = f"{len(poses)} grasp(s) detected"
        res.grasp_poses = poses
        res.grasp_scores = final_scores

        if poses:
            self.marker_pub.publish(build_gripper_markers(poses, final_scores, widths))

        return res


def main(args=None):
    rclpy.init(args=args)
    node = GigaGraspDetectionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

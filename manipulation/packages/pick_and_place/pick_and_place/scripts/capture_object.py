#!/usr/bin/env python3
"""Capture an object's point cloud for the grasp database.

Usage:
    ros2 run pick_and_place capture_object --ros-args -p name:=orange -p mode:=live
    ros2 run pick_and_place capture_object --ros-args -p name:=orange -p mode:=file -p input:=/path/to/scan.ply
"""

import os
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import rclpy.duration

from pick_and_place.utils.dense_cloud import make_dense_cluster
from pick_and_place.utils.perception_utils import get_object_detection
from frida_constants.vision_constants import DEPTH_IMAGE_TOPIC, CAMERA_INFO_TOPIC
from frida_constants.manipulation_constants import DETECTION_HANDLER_TOPIC_SRV

DB_BASE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "grasp_database",
)


class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture_object")

        self.declare_parameter("name", "")
        self.declare_parameter("mode", "live")  # "live" or "file"
        self.declare_parameter("input", "")  # path for file mode

        self.object_name = self.get_parameter("name").value
        self.mode = self.get_parameter("mode").value
        self.input_path = self.get_parameter("input").value

        if not self.object_name:
            self.get_logger().error("Parameter 'name' is required")
            raise SystemExit(1)

        self._latest_depth = None
        self._camera_info = None

        depth_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self._depth_cb, depth_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, depth_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        from frida_interfaces.srv import DetectionHandler
        self.detection_client = self.create_client(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV
        )

    def _depth_cb(self, msg):
        self._latest_depth = msg

    def _info_cb(self, msg):
        self._camera_info = msg

    def get_tf_matrix(self, source, target):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            mat = np.eye(4)
            mat[:3, :3] = rot
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def get_viewpoint(self):
        """Get camera position in base_link via TF."""
        mat = self.get_tf_matrix("zed_left_camera_optical_frame", "base_link")
        if mat is not None:
            return mat[:3, 3].tolist()
        return [0.0, 0.06, 1.17]  # fallback

    def capture_live(self):
        self.get_logger().info("=== Live Capture Mode ===")
        self.get_logger().info(f"Object: {self.object_name}")
        self.get_logger().info("Place the object in front of the robot and press Enter...")

        input(">> Press Enter to capture...")

        # Wait for depth + camera_info
        self.get_logger().info("Waiting for depth image...")
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_depth is not None and self._camera_info is not None:
                break

        if self._latest_depth is None or self._camera_info is None:
            self.get_logger().error("No depth/camera_info received")
            return None, None

        # Get detection bbox
        self.get_logger().info(f"Getting detection for '{self.object_name}'...")
        detection = get_object_detection(self.object_name, self.detection_client)
        if detection is None:
            self.get_logger().error(f"Object '{self.object_name}' not detected")
            return None, None

        # TF for deprojection
        tf_matrix = self.get_tf_matrix("zed_left_camera_optical_frame", "base_link")
        if tf_matrix is None:
            return None, None

        # Build dense cloud
        cloud_msg = make_dense_cluster(
            self._latest_depth,
            self._camera_info,
            detection.xmin, detection.ymin,
            detection.xmax, detection.ymax,
            tf_matrix=tf_matrix,
        )
        if cloud_msg is None:
            self.get_logger().error("Failed to create dense cloud")
            return None, None

        # Read points
        gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array([list(p) for p in gen], dtype=np.float64)
        if len(points) == 0:
            self.get_logger().error("Empty cloud")
            return None, None

        viewpoint = self.get_viewpoint()
        centroid = points.mean(axis=0).tolist()

        self.get_logger().info(f"Captured {len(points)} points, centroid={centroid}")
        return points, {"centroid": centroid, "viewpoint": viewpoint}

    def capture_file(self):
        self.get_logger().info("=== File Capture Mode ===")
        self.get_logger().info(f"Object: {self.object_name}")
        self.get_logger().info(f"Input: {self.input_path}")

        if not os.path.isfile(self.input_path):
            self.get_logger().error(f"File not found: {self.input_path}")
            return None, None

        ext = os.path.splitext(self.input_path)[1].lower()

        try:
            import open3d as o3d
            if ext in (".ply", ".pcd", ".xyz", ".obj", ".stl"):
                pcd = o3d.io.read_point_cloud(self.input_path)
                points = np.asarray(pcd.points)
            else:
                self.get_logger().error(f"Unsupported format: {ext}")
                return None, None
        except ImportError:
            # Fallback: try PCL via numpy for PCD files
            if ext == ".pcd":
                points = self._read_pcd_numpy(self.input_path)
            else:
                self.get_logger().error("Open3D not available and file is not PCD")
                return None, None

        if points is None or len(points) == 0:
            self.get_logger().error("No points loaded")
            return None, None

        centroid = points.mean(axis=0).tolist()
        viewpoint = [0.0, 0.06, 1.17]  # default

        self.get_logger().info(f"Loaded {len(points)} points from file")
        return points, {"centroid": centroid, "viewpoint": viewpoint}

    def _read_pcd_numpy(self, path):
        """Simple ASCII PCD reader."""
        with open(path, "r") as f:
            lines = f.readlines()
        data_start = None
        for i, line in enumerate(lines):
            if line.strip().startswith("DATA"):
                data_start = i + 1
                break
        if data_start is None:
            return None
        points = []
        for line in lines[data_start:]:
            parts = line.strip().split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        return np.array(points, dtype=np.float64) if points else None

    def save(self, points, metadata):
        obj_dir = os.path.join(DB_BASE, "objects", self.object_name)
        os.makedirs(obj_dir, exist_ok=True)

        # Save PCD (simple ASCII format)
        pcd_path = os.path.join(obj_dir, "model.pcd")
        with open(pcd_path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        # Save metadata
        meta_path = os.path.join(obj_dir, "metadata.yaml")
        metadata["object_name"] = self.object_name
        metadata["capture_date"] = time.strftime("%Y-%m-%d")
        metadata["num_points"] = len(points)
        with open(meta_path, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False)

        # Update database index
        db_file = os.path.join(DB_BASE, "database.yaml")
        if os.path.isfile(db_file):
            with open(db_file, "r") as f:
                db = yaml.safe_load(f) or {}
        else:
            db = {}
        objects = db.get("objects", [])
        if self.object_name not in objects:
            objects.append(self.object_name)
        db["objects"] = objects
        with open(db_file, "w") as f:
            yaml.dump(db, f, default_flow_style=False)

        self.get_logger().info(f"Saved to {obj_dir}")
        self.get_logger().info(f"  model.pcd: {len(points)} points")
        self.get_logger().info(f"  metadata.yaml: centroid={metadata['centroid']}")

    def run(self):
        if self.mode == "live":
            points, metadata = self.capture_live()
        elif self.mode == "file":
            points, metadata = self.capture_file()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        if points is not None and metadata is not None:
            self.save(points, metadata)
            self.get_logger().info("Capture complete!")
        else:
            self.get_logger().error("Capture failed")


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

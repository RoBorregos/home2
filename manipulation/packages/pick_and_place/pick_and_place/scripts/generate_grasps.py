#!/usr/bin/env python3
"""Generate grasps offline for an object in the grasp database.

Calls the GPD service with the stored PCD file and saves the resulting
grasps as a YAML file.  Handles GPD service restart (it kills itself
after each call).

Usage:
    ros2 run pick_and_place generate_grasps --ros-args -p name:=orange [-p num_samples:=2000]
"""

import os
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import GraspDetection
from frida_constants.manipulation_constants import GRASP_DETECTION_SERVICE

DB_BASE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "grasp_database",
)

# Default GPD config paths (inside the container workspace)
CFG_PATHS = [
    "/workspace/src/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg",
    "/workspace/src/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg",
]


class GenerateGraspsNode(Node):
    def __init__(self):
        super().__init__("generate_grasps")

        self.declare_parameter("name", "")
        self.declare_parameter("num_samples", 2000)

        self.object_name = self.get_parameter("name").value
        self.num_samples = self.get_parameter("num_samples").value

        if not self.object_name:
            self.get_logger().error("Parameter 'name' is required")
            raise SystemExit(1)

        self.grasp_client = self.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE
        )

    def wait_for_gpd(self, timeout=30.0):
        """Wait for GPD service to be available (it respawns after each call)."""
        self.get_logger().info("Waiting for GPD service...")
        available = self.grasp_client.wait_for_service(timeout_sec=timeout)
        if not available:
            self.get_logger().error(f"GPD service not available after {timeout}s")
            return False
        self.get_logger().info("GPD service ready")
        return True

    def call_gpd(self, pcd_path, cfg_path):
        """Call GPD service with a PCD file."""
        request = GraspDetection.Request()
        request.pcd_path = pcd_path
        request.cfg_path = cfg_path

        self.get_logger().info(f"Calling GPD with {pcd_path}...")
        future = self.grasp_client.call_async(request)

        # Spin until done (GPD takes a few seconds)
        timeout = 60.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not future.done():
            self.get_logger().error("GPD call timed out")
            return [], []

        result = future.result()
        if result is None:
            self.get_logger().error("GPD returned None")
            return [], []

        return result.grasp_poses, result.grasp_scores

    def run(self):
        obj_dir = os.path.join(DB_BASE, "objects", self.object_name)
        pcd_path = os.path.join(obj_dir, "model.pcd")
        meta_path = os.path.join(obj_dir, "metadata.yaml")

        if not os.path.isfile(pcd_path):
            self.get_logger().error(
                f"model.pcd not found for '{self.object_name}'. "
                f"Run capture_object first."
            )
            return

        # Load metadata for centroid
        if os.path.isfile(meta_path):
            with open(meta_path, "r") as f:
                metadata = yaml.safe_load(f)
            centroid = np.array(metadata.get("centroid", [0, 0, 0]))
            viewpoint = metadata.get("viewpoint", [0, 0.06, 1.17])
        else:
            self.get_logger().warn("No metadata.yaml found, using defaults")
            centroid = np.array([0, 0, 0])
            viewpoint = [0, 0.06, 1.17]

        self.get_logger().info(f"Object: {self.object_name}")
        self.get_logger().info(f"PCD: {pcd_path}")
        self.get_logger().info(f"Centroid: {centroid.tolist()}")
        self.get_logger().info(f"Viewpoint: {viewpoint}")

        all_grasps = []

        for cfg_path in CFG_PATHS:
            if not self.wait_for_gpd():
                self.get_logger().error("Cannot reach GPD service")
                return

            self.get_logger().info(f"Using config: {os.path.basename(cfg_path)}")
            poses, scores = self.call_gpd(pcd_path, cfg_path)

            self.get_logger().info(f"  Got {len(poses)} grasps")

            for pose, score in zip(poses, scores):
                # Convert absolute pose to relative (subtract centroid)
                abs_pos = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ])
                rel_pos = abs_pos - centroid

                orient = [
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ]

                # Estimate approach direction from orientation
                # The approach is the Z-axis of the grasp frame
                from scipy.spatial.transform import Rotation as R
                rot = R.from_quat(orient)
                approach = rot.as_matrix()[:, 2].tolist()

                all_grasps.append({
                    "position": rel_pos.tolist(),
                    "orientation": orient,
                    "score": float(score),
                    "approach": approach,
                })

            # GPD kills itself after each call — wait for respawn
            self.get_logger().info("Waiting for GPD respawn...")
            time.sleep(3)

        if not all_grasps:
            self.get_logger().error("No grasps generated!")
            return

        # Sort by score and deduplicate nearby grasps
        all_grasps.sort(key=lambda g: -g["score"])

        # Save
        grasps_file = os.path.join(obj_dir, "grasps.yaml")
        data = {
            "object_name": self.object_name,
            "capture_date": time.strftime("%Y-%m-%d"),
            "centroid": centroid.tolist(),
            "viewpoint": viewpoint,
            "num_grasps": len(all_grasps),
            "grasps": all_grasps,
        }
        with open(grasps_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.get_logger().info(f"Saved {len(all_grasps)} grasps to {grasps_file}")
        self.get_logger().info(f"Score range: {all_grasps[-1]['score']:.3f} - {all_grasps[0]['score']:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = GenerateGraspsNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

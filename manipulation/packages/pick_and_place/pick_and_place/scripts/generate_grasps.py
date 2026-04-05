#!/usr/bin/env python3
"""Generate grasps offline for an object in the grasp database.

Runs GPD multiple times with different approach direction configs to
generate grasps from all angles.  Each run produces ~15-20 grasps;
5 runs combined yield ~75-100 unique grasps in ~8-10 minutes.

Usage:
    ros2 run pick_and_place generate_grasps --ros-args -p name:=orange
"""

import os
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import GraspDetection
from scipy.spatial.transform import Rotation as R

DB_BASE = "/workspace/src/manipulation/packages/pick_and_place/grasp_database"

# Base GPD config (will be copied and modified per run)
BASE_CFG = "/workspace/src/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg"

# Multiple runs without approach filter — the PCD is in camera frame
# so direction-based filtering doesn't work.  Each run with different
# num_samples seeds GPD differently, producing diverse grasps.
APPROACH_CONFIGS = [
    {"name": "run1", "filter": False, "samples": 3000},
    {"name": "run2", "filter": False, "samples": 5000},
    {"name": "run3", "filter": False, "samples": 8000},
]

GRASP_SERVICE = "/manipulation/detect_grasps"
DEDUP_DISTANCE = 0.005  # 5mm — remove near-duplicate grasps


class GenerateGraspsNode(Node):
    def __init__(self):
        super().__init__("generate_grasps")
        self.declare_parameter("name", "")
        self.object_name = self.get_parameter("name").value

        if not self.object_name:
            self.get_logger().error("Parameter 'name' is required")
            raise SystemExit(1)

        self.grasp_client = self.create_client(GraspDetection, GRASP_SERVICE)

    def create_config(self, approach_cfg):
        """Create a temporary GPD config with specific approach settings."""
        with open(BASE_CFG, "r") as f:
            content = f.read()

        import re
        content = re.sub(
            r"^num_samples = \d+", f"num_samples = {approach_cfg['samples']}",
            content, flags=re.MULTILINE,
        )
        content = re.sub(
            r"^num_selected = \d+", "num_selected = 100",
            content, flags=re.MULTILINE,
        )
        filter_val = 1 if approach_cfg.get("filter", False) else 0
        content = re.sub(
            r"^filter_approach_direction = \d+",
            f"filter_approach_direction = {filter_val}",
            content, flags=re.MULTILINE,
        )

        tmp_path = f"/tmp/gpd_offline_{approach_cfg['name']}.cfg"
        with open(tmp_path, "w") as f:
            f.write(content)
        return tmp_path

    def wait_for_gpd(self, timeout=30.0):
        """Wait for GPD service to respawn."""
        self.get_logger().info("  Waiting for GPD service...")
        return self.grasp_client.wait_for_service(timeout_sec=timeout)

    def call_gpd(self, pcd_path, cfg_path):
        """Call GPD and return poses + scores."""
        request = GraspDetection.Request()
        request.pcd_path = pcd_path
        request.cfg_path = cfg_path

        future = self.grasp_client.call_async(request)
        timeout = 600.0  # 10 min max
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not future.done():
            self.get_logger().error("  GPD call timed out")
            return [], []

        result = future.result()
        if result is None:
            return [], []

        return result.grasp_poses, result.grasp_scores

    def poses_to_grasps(self, poses, scores, centroid):
        """Convert PoseStamped[] to grasp dicts relative to centroid."""
        grasps = []
        for pose, score in zip(poses, scores):
            abs_pos = np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ])
            rel_pos = (abs_pos - centroid).tolist()
            orient = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
            rot = R.from_quat(orient)
            approach = rot.as_matrix()[:, 2].tolist()

            grasps.append({
                "position": [round(x, 6) for x in rel_pos],
                "orientation": [round(x, 6) for x in orient],
                "score": round(float(score), 4),
                "approach": [round(x, 6) for x in approach],
            })
        return grasps

    def deduplicate(self, grasps):
        """Remove grasps within DEDUP_DISTANCE of each other."""
        if not grasps:
            return grasps
        filtered = [grasps[0]]
        for g in grasps[1:]:
            is_dup = False
            for f in filtered:
                dist = sum(
                    (a - b) ** 2 for a, b in zip(g["position"], f["position"])
                ) ** 0.5
                if dist < DEDUP_DISTANCE:
                    is_dup = True
                    break
            if not is_dup:
                filtered.append(g)
        return filtered

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

        # Load metadata
        centroid = np.array([0, 0, 0], dtype=np.float64)
        viewpoint = [0, 0.06, 1.17]
        if os.path.isfile(meta_path):
            with open(meta_path, "r") as f:
                metadata = yaml.safe_load(f)
            centroid = np.array(metadata.get("centroid", [0, 0, 0]))
            viewpoint = metadata.get("viewpoint", viewpoint)

        self.get_logger().info(f"=== Generating grasps for '{self.object_name}' ===")
        self.get_logger().info(f"PCD: {pcd_path}")
        self.get_logger().info(f"Centroid: {centroid.tolist()}")
        self.get_logger().info(f"Running {len(APPROACH_CONFIGS)} configs...")

        all_grasps = []
        total_start = time.time()

        for i, cfg in enumerate(APPROACH_CONFIGS):
            self.get_logger().info(
                f"\n[{i+1}/{len(APPROACH_CONFIGS)}] Config '{cfg['name']}': "
                f"samples={cfg['samples']}, filter={cfg.get('filter', False)}"
            )

            if not self.wait_for_gpd():
                self.get_logger().error("  GPD not available, skipping")
                continue

            tmp_cfg = self.create_config(cfg)
            t0 = time.time()
            poses, scores = self.call_gpd(pcd_path, tmp_cfg)
            elapsed = time.time() - t0

            grasps = self.poses_to_grasps(poses, scores, centroid)
            all_grasps.extend(grasps)

            self.get_logger().info(
                f"  Got {len(grasps)} grasps in {elapsed:.1f}s"
            )

            # GPD kills itself — wait for respawn before next run
            if i < len(APPROACH_CONFIGS) - 1:
                self.get_logger().info("  Waiting for GPD respawn...")
                time.sleep(3)

        if not all_grasps:
            self.get_logger().error("No grasps generated from any config!")
            return

        # Sort by score and deduplicate
        all_grasps.sort(key=lambda g: -g["score"])
        unique_grasps = self.deduplicate(all_grasps)

        total_time = time.time() - total_start

        # Save
        data = {
            "object_name": self.object_name,
            "capture_date": time.strftime("%Y-%m-%d"),
            "centroid": [round(x, 6) for x in centroid.tolist()],
            "viewpoint": viewpoint,
            "num_grasps": len(unique_grasps),
            "grasps": unique_grasps,
        }
        grasps_file = os.path.join(obj_dir, "grasps.yaml")
        with open(grasps_file, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

        self.get_logger().info(f"\n=== Done ===")
        self.get_logger().info(f"Total: {len(all_grasps)} raw → {len(unique_grasps)} unique")
        self.get_logger().info(f"Time: {total_time:.0f}s ({total_time/60:.1f} min)")
        self.get_logger().info(f"Saved: {grasps_file}")
        self.get_logger().info(
            f"Score range: {unique_grasps[-1]['score']:.3f} - {unique_grasps[0]['score']:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GenerateGraspsNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

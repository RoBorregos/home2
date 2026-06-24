#!/usr/bin/env python3
"""Calibrate the 3 cabinet shelf heights by clicking them in RViz Publish Point.
Run: ros2 run xarm_utils shelf_height_calibrator.py (writes config/shelf_levels.json)."""

import json
import os
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PointStamped for tf2 Buffer.transform
from ament_index_python.packages import get_package_share_directory

LEVEL_NAMES = ["BOTTOM (lowest)", "MIDDLE", "TOP (highest)"]


def levels_file():
    """Resolve the shelf levels path: FRIDA_SHELF_LEVELS_FILE env, else package config."""
    env = os.environ.get("FRIDA_SHELF_LEVELS_FILE")
    if env:
        return Path(env)
    try:
        share = Path(get_package_share_directory("xarm_utils"))
        return share / "config" / "shelf_levels.json"
    except Exception:
        return Path.home() / "frida_shelf_levels.json"


def levels_from_sorted_heights(heights):
    ordered = sorted(float(h) for h in heights)
    return {i + 1: h for i, h in enumerate(ordered)}


def save_shelf_levels(levels, path):
    payload = {
        "frame": "base_link",
        "levels": {str(k): float(v) for k, v in levels.items()},
    }
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2))
    return path


class ShelfHeightCalibrator(Node):
    def __init__(self):
        super().__init__("shelf_height_calibrator")
        self._clicks = []
        self._lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointStamped, "/clicked_point", self._on_click, 10)

    def _on_click(self, msg):
        z = self._to_base_link_z(msg)
        if z is None:
            return
        with self._lock:
            self._clicks.append(z)
            n = len(self._clicks)
        self.get_logger().info(f"click -> base_link z = {z:.3f} m ({n} this level)")

    def _to_base_link_z(self, msg):
        try:
            t = self.tf_buffer.transform(
                msg, "base_link", timeout=Duration(seconds=1.0)
            )
            return float(t.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF {msg.header.frame_id}->base_link failed: {e}")
            return None

    def take_level(self, name):
        """Wait for the operator to click a level, return the averaged base_link Z."""
        with self._lock:
            self._clicks = []
        input(
            f"\nClick the {name} shelf surface in RViz (1+ points), then press Enter..."
        )
        with self._lock:
            clicks = list(self._clicks)
        if not clicks:
            print("  No clicks captured. Try again.")
            return None
        avg = sum(clicks) / len(clicks)
        print(f"  {len(clicks)} click(s), base_link z = {avg:.3f} m")
        return avg


def main():
    rclpy.init()
    node = ShelfHeightCalibrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    path = levels_file()
    try:
        heights = []
        for name in LEVEL_NAMES:
            z = None
            while z is None:
                z = node.take_level(name)
            heights.append(z)
        levels = levels_from_sorted_heights(heights)
        save_shelf_levels(levels, path)
        print("\nCalibrated shelf levels (base_link Z):")
        for k in sorted(levels):
            print(f"  level {k}: {levels[k]:.3f} m")
        print(f"\nSaved to: {path}")
    except (KeyboardInterrupt, EOFError):
        print("\nAborted; nothing written.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

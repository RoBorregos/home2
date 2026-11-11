#!/usr/bin/env python3
"""Automatically detect shelf surface heights from an RGB-D view.

Place the robot in front of the shelf, point the depth camera at the complete
front, and run:

    ros2 run xarm_utils shelf_live_calibrator.py --arena 1

The file is only written after the detected horizontal surfaces are validated.
Use ``--dry-run`` to inspect the result without changing the file.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.duration
import rclpy.qos
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

from xarm_utils.shelf_levels import (
    DEFAULT_MIN_LEVEL_SEPARATION,
    MAX_SHELF_LEVELS,
    MIN_SHELF_LEVELS,
    ShelfCalibrationError,
    levels_file,
    load_shelf_calibration,
)

ARM_POSES_DEG = {
    "front_stare": [-90.0, -45.0, -90.0, 0.0, 0.0, 45.0],
}
MOVE_JOINTS_ACTION = "/manipulation/move_joints_action_server"
DOCK_TABLE_SERVICE = "/navigation/dock_table"
DEFAULT_DOCK_OFFSET = 0.30
DEFAULT_VALID_FRAMES = 12
DEFAULT_MAX_FRAME_SPREAD = 0.015


def _rotation_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def detect_shelf_levels(
    depth: np.ndarray,
    camera_info: CameraInfo,
    transform,
    min_levels: int = MIN_SHELF_LEVELS,
    max_levels: int = MAX_SHELF_LEVELS,
    min_level_separation: float = DEFAULT_MIN_LEVEL_SEPARATION,
) -> list[float]:
    """Estimate horizontal shelf heights in ``base_link`` from one depth image.

    Candidate bands are accepted only when they are wide in both horizontal
    directions and locally planar. This rejects most object tops and depth
    speckle before temporal fusion.
    """
    if depth.ndim != 2:
        raise ShelfCalibrationError(f"Expected a depth image, got shape {depth.shape}")

    height, width = depth.shape
    y0, y1 = int(height * 0.10), int(height * 0.95)
    x0, x1 = int(width * 0.10), int(width * 0.90)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    values = depth[y0:y1, x0:x1].astype(float)
    valid = np.isfinite(values) & (values > 0.15) & (values < 4.0)
    if int(valid.sum()) < 200:
        raise ShelfCalibrationError("Not enough valid depth points in the shelf view")

    z = values[valid]
    u = xx[valid]
    v = yy[valid]
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    if fx <= 0 or fy <= 0:
        raise ShelfCalibrationError("CameraInfo has invalid focal lengths")

    points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    rotation = _rotation_matrix(transform.transform.rotation)
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
    )
    base_points = points @ rotation.T + translation

    # Horizontal shelf surfaces produce dense, spatially wide, locally flat bands.
    bin_size = 0.01
    minimum_z = float(np.percentile(base_points[:, 2], 1))
    bins = np.floor((base_points[:, 2] - minimum_z) / bin_size).astype(int)
    candidates = []
    for bin_index in np.unique(bins):
        band = base_points[bins == bin_index]
        if len(band) < max(20, int(len(base_points) * 0.003)):
            continue
        x_span = np.percentile(band[:, 0], 95) - np.percentile(band[:, 0], 5)
        y_span = np.percentile(band[:, 1], 95) - np.percentile(band[:, 1], 5)
        z_residual = float(
            np.percentile(np.abs(band[:, 2] - np.median(band[:, 2])), 90)
        )
        if x_span < 0.20 or y_span < 0.08 or z_residual > 0.025:
            continue
        candidates.append(
            (
                len(band),
                float(
                    np.median(band[:, 2]),
                ),
            )
        )

    candidates.sort(reverse=True)
    if min_level_separation <= 0:
        raise ShelfCalibrationError("Minimum level separation must be positive")
    selected = []
    for _, candidate_z in candidates:
        if all(
            abs(candidate_z - selected_z) >= min_level_separation
            for selected_z in selected
        ):
            selected.append(candidate_z)
        if len(selected) == max_levels:
            break
    if len(selected) < min_levels:
        raise ShelfCalibrationError(
            f"Detected {len(selected)} shelf levels; expected at least {min_levels}. "
            "Ensure the complete shelf is visible and well illuminated."
        )
    return sorted(selected)


def save_detected_levels(arena: int, levels: list[float], path: Path) -> None:
    """Update one arena while preserving the other calibration entries."""
    data = json.loads(path.read_text()) if path.exists() else {}
    data["frame"] = "base_link"
    data.setdefault("compartment_height", 0.26)
    data.setdefault("arenas", {})[str(arena)] = [round(level, 4) for level in levels]
    path.write_text(json.dumps(data, indent=2) + "\n")


class ShelfLiveCalibrator:
    def __init__(
        self,
        node,
        depth_topic: str,
        camera_info_topic: str,
        min_level_separation: float,
    ):
        self.node = node
        self.bridge = CvBridge()
        self.depth = None
        self.camera_info = None
        self.depth_frame = None
        self.min_level_separation = min_level_separation
        qos = rclpy.qos.qos_profile_sensor_data
        node.create_subscription(Image, depth_topic, self._depth_callback, qos)
        node.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def _depth_callback(self, message):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1")
            self.depth_frame = message.header.frame_id
        except CvBridgeError as exc:
            self.node.get_logger().error(f"Depth conversion failed: {exc}")

    def _camera_info_callback(self, message):
        self.camera_info = message

    def wait_for_calibration(self, timeout: float) -> list[float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.depth is None or self.camera_info is None or not self.depth_frame:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    self.depth_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                return detect_shelf_levels(
                    self.depth,
                    self.camera_info,
                    transform,
                    min_level_separation=self.min_level_separation,
                )
            except (tf2_ros.TransformException, ShelfCalibrationError) as exc:
                self.node.get_logger().warn(f"Waiting for a valid shelf view: {exc}")
        raise ShelfCalibrationError(
            f"Timed out after {timeout:.0f}s waiting for shelf geometry"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arena", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--min-level-separation",
        type=float,
        default=DEFAULT_MIN_LEVEL_SEPARATION,
        help="Minimum distance between detected levels in meters.",
    )
    parser.add_argument("--depth-topic", default="/zed/zed_node/depth/depth_registered")
    parser.add_argument(
        "--camera-info-topic", default="/zed/zed_node/depth/camera_info"
    )
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("shelf_live_calibrator")
    calibrator = ShelfLiveCalibrator(
        node,
        args.depth_topic,
        args.camera_info_topic,
        args.min_level_separation,
    )
    try:
        levels = calibrator.wait_for_calibration(args.timeout)
        path = levels_file()
        print("Detected shelf levels in base_link:")
        for index, level in enumerate(levels, start=1):
            print(f"  level {index}: {level:.3f} m")
        if args.dry_run:
            print("Dry run: calibration file was not changed.")
        else:
            save_detected_levels(args.arena, levels, path)
            load_shelf_calibration(args.arena, path)
            print(f"Saved automatic calibration to {path}")
        return 0
    except ShelfCalibrationError as exc:
        print(f"Automatic shelf calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

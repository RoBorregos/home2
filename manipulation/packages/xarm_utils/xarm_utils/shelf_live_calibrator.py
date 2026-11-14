#!/usr/bin/env python3
"""Automatically detect shelf surface heights from an RGB-D view.

Place the robot in front of the shelf, point the depth camera at the complete
front, and run:

    ros2 run xarm_utils shelf_live_calibrator.py --arena 1

The file is only written after the detected horizontal surfaces are validated.
Use ``--dry-run`` to inspect the result without changing the file.
"""

import argparse
from collections import deque
import math
import sys
import time
from pathlib import Path
from threading import Thread

import rclpy
import rclpy.qos
from rclpy.executors import (
    ExternalShutdownException,
    ShutdownException,
    SingleThreadedExecutor,
)
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, JointState

from xarm_utils.shelf_detection import DEFAULT_MAX_TILT_DEG, detect_shelf_geometry
from xarm_utils.shelf_geometry import (
    compare_reference_heights,
    confirmed_geometry_report,
)
from xarm_utils.shelf_observations import ShelfLevelConsensus
from xarm_utils.shelf_levels import (
    DEFAULT_MIN_LEVEL_SEPARATION,
    ShelfCalibrationError,
    levels_file,
    save_detected_levels,
    write_json_atomic,
)

ARM_POSES_DEG = {
    "front_stare": [-90.0, -45.0, -90.0, 0.0, 0.0, 45.0],
}
MOVE_JOINTS_ACTION = "/manipulation/move_joints_action_server"
DOCK_TABLE_SERVICE = "/navigation/dock_table"
DEFAULT_DOCK_OFFSET = 0.30
DEFAULT_VALID_FRAMES = 12
DEFAULT_MAX_FRAME_SPREAD = 0.015


class ShelfLiveCalibrator:
    def __init__(
        self,
        node,
        depth_topic: str,
        camera_info_topic: str,
        min_level_separation: float,
        max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
        valid_frames: int = DEFAULT_VALID_FRAMES,
        max_frame_spread: float = DEFAULT_MAX_FRAME_SPREAD,
        joint_states_topic: str = "/xarm/joint_states",
        max_frame_age: float = 1.0,
        min_valid_frames: int | None = None,
    ):
        self.node = node
        self.bridge = CvBridge()
        self.depth = None
        self.camera_info = None
        self.depth_frame = None
        self.depth_stamp = None
        self.joint_states = None
        self.joint_states_topic = joint_states_topic
        self.max_frame_age = max_frame_age
        self._frames = deque(maxlen=3)
        self._last_processed_stamp = None
        self._processed_stamps = deque(maxlen=2 * valid_frames)
        self._last_warning = float("-inf")
        self.consensus = ShelfLevelConsensus(
            valid_frames, max_frame_spread, min_frames=min_valid_frames
        )
        self.geometry_history = deque(maxlen=valid_frames)
        self.geometry_report = None
        self._calibration_warning_emitted = False
        self.min_level_separation = min_level_separation
        self.max_tilt_deg = max_tilt_deg
        qos = rclpy.qos.qos_profile_sensor_data
        node.create_subscription(Image, depth_topic, self._depth_callback, qos)
        node.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos
        )
        node.create_subscription(
            JointState, joint_states_topic, self._joint_states_callback, qos
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def _depth_callback(self, message):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1")
            self.depth_frame = message.header.frame_id
            self.depth_stamp = message.header.stamp
            self._frames.append((self.depth, self.depth_frame, self.depth_stamp))
        except CvBridgeError as exc:
            self.node.get_logger().error(f"Depth conversion failed: {exc}")

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _joint_states_callback(self, message):
        if len(message.position) >= 6 and all(
            math.isfinite(value) for value in message.position
        ):
            self.joint_states = message

    def _stamp_is_fresh(self, stamp) -> bool:
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return stamp_ns > 0 and -0.1 <= age <= self.max_frame_age

    def _warn_wait(self, reason: str) -> None:
        if time.monotonic() - self._last_warning >= 1.0:
            self.node.get_logger().warn(f"Waiting for a valid shelf view: {reason}")
            self._last_warning = time.monotonic()

    def wait_for_calibration(self, timeout: float) -> list[float]:
        """Receive camera, joint and TF callbacks while geometry is processed."""
        executor = SingleThreadedExecutor(context=self.node.context)
        executor.add_node(self.node)

        def receive_messages():
            try:
                executor.spin()
            except (ExternalShutdownException, ShutdownException):
                pass

        receiver = Thread(target=receive_messages, daemon=True)
        receiver.start()
        try:
            return self._collect_calibration(timeout)
        finally:
            executor.shutdown()
            receiver.join()
            executor.remove_node(self.node)

    def _collect_calibration(self, timeout: float) -> list[float]:
        deadline = time.monotonic() + timeout
        pending = None
        pending_since = 0.0
        last_reason = "waiting for depth, camera information and live arm joints"
        while time.monotonic() < deadline:
            time.sleep(0.01)
            if self.camera_info is None:
                continue
            if self.joint_states is None or not self._stamp_is_fresh(
                self.joint_states.header.stamp
            ):
                last_reason = (
                    f"missing or stale live arm feedback on {self.joint_states_topic}"
                )
                self._warn_wait(last_reason)
                pending = None
                self.consensus.reset()
                self.geometry_history.clear()
                continue
            if pending is None:
                if not self._frames:
                    continue
                pending = self._frames.pop()
                self._frames.clear()
                pending_since = time.monotonic()
            depth, frame, stamp = pending
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns in self._processed_stamps:
                pending = None
                continue
            if not frame or not self._stamp_is_fresh(stamp):
                pending = None
                last_reason = "missing depth frame or stale depth timestamp"
                self._warn_wait(last_reason)
                continue
            try:
                # Never block the executor while waiting for its own TF callbacks.
                # Keep a pending image briefly so its matching TF can arrive.
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    frame,
                    rclpy.time.Time.from_msg(stamp),
                )
            except tf2_ros.TransformException as exc:
                last_reason = str(exc)
                self._warn_wait(last_reason)
                if time.monotonic() - pending_since >= 0.3:
                    pending = None
                continue
            self._last_processed_stamp = stamp_ns
            self._processed_stamps.append(stamp_ns)
            pending = None
            try:
                observation = detect_shelf_geometry(
                    depth,
                    self.camera_info,
                    transform,
                    min_levels=1,
                    min_level_separation=self.min_level_separation,
                    max_tilt_deg=self.max_tilt_deg,
                )
                if observation.diagnostics["invalid_frame"]:
                    self.consensus.reset()
                    self.geometry_history.clear()
                    raise ShelfCalibrationError(
                        "Support heights are below base_link; verify live joints and camera TF"
                    )
                levels = observation.levels
                evidence = [
                    s.evidence(observation.calibration_warning)
                    for s in observation.surfaces
                ]
                if (
                    observation.calibration_warning
                    and not self._calibration_warning_emitted
                ):
                    for warning in observation.diagnostics["warnings"]:
                        self.node.get_logger().warn(warning)
                    self._calibration_warning_emitted = True
            except ShelfCalibrationError as exc:
                levels = []
                evidence = []
                observation = None
                last_reason = str(exc)
                self._warn_wait(last_reason)
            self.geometry_history.append(observation)
            stable = self.consensus.add(levels, evidence)
            if stable is not None and all(
                current - previous >= self.min_level_separation
                for previous, current in zip(stable, stable[1:])
            ):
                self.geometry_report = confirmed_geometry_report(
                    stable,
                    self.geometry_history,
                    self.consensus.max_spread,
                    self.consensus.confirmation_mode,
                )
                self.node.get_logger().info(
                    f"Confirmed {len(stable)} shelf levels across "
                    f"{len(self.consensus.samples)} distinct depth frames "
                    f"({self.consensus.confirmation_mode})"
                )
                return stable
            last_reason = "waiting for repeated, stable shelf heights"
        raise ShelfCalibrationError(
            f"Timed out after {timeout:.0f}s waiting for shelf geometry: {last_reason}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arena", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--valid-frames", type=int, default=DEFAULT_VALID_FRAMES)
    parser.add_argument(
        "--min-valid-frames",
        type=int,
        default=None,
        help="Earliest confirmation with high-quality, structurally bounded observations (default: up to 6 frames).",
    )
    parser.add_argument(
        "--geometry-json",
        type=Path,
        help="Write measured planes, estimated outlines, obstacles, unknown cells and diagnostics to this separate JSON file.",
    )
    parser.add_argument(
        "--reference-heights",
        type=float,
        nargs="+",
        help="Optional externally measured heights in base_link, ordered low to high; verifies accuracy without changing TF.",
    )
    parser.add_argument("--max-reference-error", type=float, default=0.02)
    parser.add_argument(
        "--max-frame-spread",
        type=float,
        default=DEFAULT_MAX_FRAME_SPREAD,
        help="Maximum height spread across the confirmation window, in meters.",
    )
    parser.add_argument("--joint-states-topic", default="/xarm/joint_states")
    parser.add_argument(
        "--max-frame-age",
        type=float,
        default=1.0,
        help="Maximum age of depth and live arm feedback, in seconds.",
    )
    parser.add_argument(
        "--max-tilt-deg",
        type=float,
        default=DEFAULT_MAX_TILT_DEG,
        help="Maximum support-surface tilt from horizontal, in degrees (default: 15).",
    )
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
    if args.valid_frames < 3:
        parser.error("--valid-frames must be at least 3")
    if (
        args.min_valid_frames is not None
        and not 3 <= args.min_valid_frames <= args.valid_frames
    ):
        parser.error("--min-valid-frames must be between 3 and --valid-frames")
    for name in ("timeout", "max_frame_spread", "max_frame_age", "max_reference_error"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")

    rclpy.init()
    node = rclpy.create_node("shelf_live_calibrator")
    calibrator = ShelfLiveCalibrator(
        node,
        args.depth_topic,
        args.camera_info_topic,
        args.min_level_separation,
        max_tilt_deg=args.max_tilt_deg,
        valid_frames=args.valid_frames,
        max_frame_spread=args.max_frame_spread,
        joint_states_topic=args.joint_states_topic,
        max_frame_age=args.max_frame_age,
        min_valid_frames=args.min_valid_frames,
    )
    try:
        path = levels_file()
        if (
            args.geometry_json is not None
            and args.geometry_json.resolve() == path.resolve()
        ):
            raise ShelfCalibrationError(
                "Geometry output must differ from the calibration file"
            )
        levels = calibrator.wait_for_calibration(args.timeout)
        report = calibrator.geometry_report
        reference = None
        if args.reference_heights is not None:
            reference = compare_reference_heights(
                levels, args.reference_heights, args.max_reference_error
            )
            report["reference_check"] = reference
        if args.geometry_json is not None:
            write_json_atomic(args.geometry_json, report)
            print(f"Saved shelf geometry and diagnostics to {args.geometry_json}")
        if reference is not None and not reference["within_tolerance"]:
            raise ShelfCalibrationError(
                f"Reference height check failed: maximum error {reference['max_absolute_error_m']:.3f} m; calibration was not changed"
            )
        print("Detected shelf levels in base_link:")
        for index, level in enumerate(levels, start=1):
            print(f"  level {index}: {level:.3f} m")
        if args.dry_run:
            print("Dry run: calibration file was not changed.")
        else:
            save_detected_levels(
                args.arena, levels, path, min_level_separation=args.min_level_separation
            )
            print(f"Saved automatic calibration to {path}")
        return 0
    except (ShelfCalibrationError, OSError, ValueError) as exc:
        print(f"Automatic shelf calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

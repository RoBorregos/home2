#!/usr/bin/env python3
"""
Vision Central
==============

This node is the central runtime controller for the vision stack.

What it does
------------
1) Monitors core requirements (topics + TF frame presence).
2) Monitors CPU/GPU/Memory usage for key vision nodes.
3) Exposes operator services to pause/resume or globally enable/disable vision.
4) Publishes standardized Bool active flags to managed vision modules.

State model
-----------
- `direct_enabled`: operator/global intent (SetBool service).
- `paused`: manual pause state (pause/resume services).
- `requirements_ready`: camera/depth/camera_info + TF checks are healthy.
- `overload_pause`: automatic protection state when CPU/GPU thresholds are exceeded.

Effective runtime state:
    effective_active = direct_enabled and (not paused) and requirements_ready and (not overload_pause)

Behavior notes
--------------
- The node controls modules via standardized Bool active topics.
- Tracker is controlled through `/vision/tracker/active` and is no longer
  handled through fallback service calls.
- Pointing detection is intentionally excluded (legacy).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import psutil
import rclpy
import tf2_ros
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FACE_RECOGNITION_ACTIVE_TOPIC,
    OBJECT_DETECTOR_ACTIVE_TOPIC,
    TRACKER_ACTIVE_TOPIC,
    VISION_CENTRAL_MONITOR_TOPIC,
    VISION_CENTRAL_NODE_NAME,
    VISION_CENTRAL_PAUSE_SERVICE,
    VISION_CENTRAL_RESUME_SERVICE,
    VISION_CENTRAL_SET_ENABLED_SERVICE,
    VISION_CENTRAL_STATUS_SERVICE,
    ZERO_SHOT_DETECTOR_ACTIVE_TOPIC,
)
from frida_interfaces.msg import MonitorReport, NodeStatus
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, Trigger
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

try:
    from pynvml import (
        nvmlDeviceGetHandleByIndex,
        nvmlDeviceGetUtilizationRates,
        nvmlInit,
    )

    HAS_PYNVML = True
except ImportError:
    HAS_PYNVML = False

try:
    from jtop import jtop

    HAS_JTOP = True
except ImportError:
    HAS_JTOP = False

# Sysfs paths used by Jetson platforms. Value is typically 0-1000.
GPU_SYSFS_PATHS = [
    "/sys/devices/platform/gpu.0/load",
    "/sys/devices/gpu.0/load",
    "/sys/devices/platform/bus@0/17000000.gpu/load",
]


@dataclass
class HealthSnapshot:
    """One-shot health view used to make state decisions."""

    topics_ready: bool
    frames_ready: bool
    cpu_peak: float
    gpu_peak: float


class VisionCentral(Node):
    """Central state and health orchestrator for the vision stack."""

    def __init__(self) -> None:
        super().__init__(VISION_CENTRAL_NODE_NAME)

        # Callback groups keep timers/services responsive under load.
        self.cb_group = ReentrantCallbackGroup()

        # ----------------------------- Tunable parameters -----------------------------
        self.required_topics = self.declare_parameter(
            "required_topics",
            [CAMERA_TOPIC, DEPTH_IMAGE_TOPIC, CAMERA_INFO_TOPIC],
        ).value
        self.required_frames = self.declare_parameter(
            "required_frames", [CAMERA_FRAME]
        ).value

        self.monitor_period_s = float(
            self.declare_parameter("monitor_period_s", 1.0).value
        )
        self.missing_limit = int(self.declare_parameter("missing_limit", 3).value)
        self.overload_limit = int(self.declare_parameter("overload_limit", 3).value)
        self.cpu_overload_threshold = float(
            self.declare_parameter("cpu_overload_threshold", 90.0).value
        )
        self.gpu_overload_threshold = float(
            self.declare_parameter("gpu_overload_threshold", 95.0).value
        )

        self.managed_active_topics = self.declare_parameter(
            "managed_active_topics",
            [
                OBJECT_DETECTOR_ACTIVE_TOPIC,
                ZERO_SHOT_DETECTOR_ACTIVE_TOPIC,
                FACE_RECOGNITION_ACTIVE_TOPIC,
                TRACKER_ACTIVE_TOPIC,
            ],
        ).value

        self.nodes_to_monitor = self.declare_parameter(
            "nodes_to_monitor",
            [
                "object_detector_2D_node",
                "zero_shot_object_detector_2D_node",
                "face_recognition_node",
                "tracker_node",
                "hric_commands_node",
                "gpsr_commands_node",
                "restaurant_commands_node",
                "customer_node",
                "trash_detection_node",
                "dishwasher_node",
                "image_orienter",
            ],
        ).value

        # ----------------------------- Internal state -----------------------------
        self.direct_enabled = True
        self.paused = False
        self.requirements_ready = True
        self.overload_pause = False
        self.effective_active = True

        self.no_requirements_count = 0
        self.overload_count = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.active_publishers: Dict[str, object] = {}
        for topic in self.managed_active_topics:
            self.active_publishers[topic] = self.create_publisher(Bool, topic, 10)

        self.monitor_pub = self.create_publisher(
            MonitorReport, VISION_CENTRAL_MONITOR_TOPIC, 10
        )

        # ----------------------------- Services -----------------------------
        self.create_service(
            Empty,
            VISION_CENTRAL_PAUSE_SERVICE,
            self._pause_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            Empty,
            VISION_CENTRAL_RESUME_SERVICE,
            self._resume_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            SetBool,
            VISION_CENTRAL_SET_ENABLED_SERVICE,
            self._set_enabled_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            VISION_CENTRAL_STATUS_SERVICE,
            self._status_cb,
            callback_group=self.cb_group,
        )

        # ----------------------------- GPU monitor setup -----------------------------
        self.gpu_initialized = False
        self.is_jetson = os.path.exists("/sys/module/tegra_fuse") or os.path.exists(
            "/proc/device-tree/model"
        )
        self.gpu_sysfs_path: Optional[str] = None
        for path in GPU_SYSFS_PATHS:
            if os.path.exists(path):
                self.gpu_sysfs_path = path
                break

        if HAS_PYNVML and not self.is_jetson:
            try:
                nvmlInit()
                self.gpu_initialized = True
            except Exception:
                self.gpu_initialized = False

        self.jtop_controller = None
        if self.is_jetson and HAS_JTOP:
            try:
                self.jtop_controller = jtop()
                self.jtop_controller.start()
                self.get_logger().info("Vision central: jtop initialized")
            except Exception as exc:
                self.get_logger().warn(f"Vision central: failed to start jtop: {exc}")

        self.node_procs: Dict[str, psutil.Process] = {}
        self.newly_attached = set()

        self.create_timer(
            self.monitor_period_s, self._tick, callback_group=self.cb_group
        )

        # Force an initial publish so managed nodes get a deterministic state on startup.
        self._apply_effective_state(force=True)

        self.get_logger().info(
            "Vision central ready. Services: pause/resume/set_enabled/status | "
            f"Managed topics: {self.managed_active_topics}"
        )

    # --------------------------------------------------------------------- services

    def _pause_cb(self, _req: Empty.Request, res: Empty.Response) -> Empty.Response:
        """Manual pause: keeps direct_enabled untouched but forces inactive output."""
        self.paused = True
        self._apply_effective_state(force=True)
        self.get_logger().warn("Vision central paused by operator")
        return res

    def _resume_cb(self, _req: Empty.Request, res: Empty.Response) -> Empty.Response:
        """Manual resume: removes manual pause and overload latch."""
        self.paused = False
        self.overload_pause = False
        self.overload_count = 0
        self._apply_effective_state(force=True)
        self.get_logger().info("Vision central resumed by operator")
        return res

    def _set_enabled_cb(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        """Direct global enable/disable switch controlled by operator/task manager."""
        self.direct_enabled = bool(req.data)
        self._apply_effective_state(force=True)
        res.success = True
        res.message = f"direct_enabled={self.direct_enabled}"
        self.get_logger().info(
            f"Vision central direct_enabled set to {self.direct_enabled}"
        )
        return res

    def _status_cb(
        self, _req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        """Return detailed central state in a human-readable message."""
        res.success = self.effective_active
        res.message = (
            f"effective_active={self.effective_active}, "
            f"direct_enabled={self.direct_enabled}, paused={self.paused}, "
            f"requirements_ready={self.requirements_ready}, overload_pause={self.overload_pause}, "
            f"no_requirements_count={self.no_requirements_count}, overload_count={self.overload_count}"
        )
        return res

    # --------------------------------------------------------------------- monitor

    def _tick(self) -> None:
        """Periodic health evaluation + monitor report + active-state application."""
        monitor_report, cpu_peak, gpu_peak = self._build_monitor_report()
        self.monitor_pub.publish(monitor_report)

        topics_ready = self._check_required_topics(self.required_topics)
        frames_ready = self._check_required_frames(self.required_frames)
        snapshot = HealthSnapshot(
            topics_ready=topics_ready,
            frames_ready=frames_ready,
            cpu_peak=cpu_peak,
            gpu_peak=gpu_peak,
        )

        self._update_requirement_state(snapshot)
        self._update_overload_state(snapshot)
        self._apply_effective_state(force=False)

    def _check_required_topics(self, required_topics: List[str]) -> bool:
        """True when all required topics are currently visible in ROS graph."""
        available_topics = {name for name, _types in self.get_topic_names_and_types()}
        return all(topic in available_topics for topic in required_topics)

    def _check_required_frames(self, required_frames: List[str]) -> bool:
        """True when required TF frames are visible in the current TF tree dump."""
        if not required_frames:
            return True
        try:
            frames_yaml = self._tf_buffer.all_frames_as_yaml()
            return all(frame in frames_yaml for frame in required_frames)
        except Exception:
            return False

    def _update_requirement_state(self, snapshot: HealthSnapshot) -> None:
        """Hysteresis for requirements to avoid flapping on short telemetry glitches."""
        req_ok = snapshot.topics_ready and snapshot.frames_ready
        if req_ok:
            if self.no_requirements_count != 0:
                self.get_logger().info("Vision requirements recovered")
            self.no_requirements_count = 0
            self.requirements_ready = True
            return

        self.no_requirements_count += 1
        if self.no_requirements_count >= self.missing_limit and self.requirements_ready:
            self.requirements_ready = False
            self.get_logger().warn(
                "Vision requirements missing. "
                f"topics_ready={snapshot.topics_ready}, frames_ready={snapshot.frames_ready}"
            )

    def _update_overload_state(self, snapshot: HealthSnapshot) -> None:
        """Automatic overload pause with hysteresis based on monitored CPU/GPU peaks."""
        overloaded = (
            snapshot.cpu_peak >= self.cpu_overload_threshold
            or snapshot.gpu_peak >= self.gpu_overload_threshold
        )

        if overloaded:
            self.overload_count += 1
            if self.overload_count >= self.overload_limit and not self.overload_pause:
                self.overload_pause = True
                self.get_logger().warn(
                    "Vision overload pause activated: "
                    f"cpu_peak={snapshot.cpu_peak:.1f}, gpu_peak={snapshot.gpu_peak:.1f}"
                )
        else:
            if self.overload_count != 0:
                self.get_logger().info("Vision overload counters reset")
            self.overload_count = 0
            self.overload_pause = False

    def _apply_effective_state(self, force: bool = False) -> None:
        """Publish effective active state to managed modules and handle tracker pause."""
        effective = (
            self.direct_enabled
            and (not self.paused)
            and self.requirements_ready
            and (not self.overload_pause)
        )

        if not force and effective == self.effective_active:
            return

        self.effective_active = effective
        msg = Bool()
        msg.data = effective
        for topic, publisher in self.active_publishers.items():
            publisher.publish(msg)
            self.get_logger().info(f"Published active={effective} -> {topic}")

    # --------------------------------------------------------------------- node monitor

    def _build_monitor_report(self) -> tuple[MonitorReport, float, float]:
        """Build per-node CPU/MEM/GPU report and return (report, cpu_peak, gpu_peak)."""
        report = MonitorReport()
        cpu_peak = 0.0
        gpu_peak = 0.0

        global_gpu_load = self._get_gpu_load()

        for node_name in self.nodes_to_monitor:
            status = NodeStatus()
            status.name = node_name

            proc = self.node_procs.get(node_name)
            if proc is None or not proc.is_running():
                proc = self._attach_process(node_name)

            if proc is None:
                status.cpu_usage = 0.0
                status.memory_usage = 0.0
                status.gpu_usage = 0.0
                report.nodes.append(status)
                continue

            try:
                if node_name in self.newly_attached:
                    self.newly_attached.discard(node_name)
                    status.cpu_usage = 0.0
                else:
                    status.cpu_usage = float(proc.cpu_percent())
                status.memory_usage = float(proc.memory_percent())
                status.gpu_usage = (
                    float(global_gpu_load) if self._process_uses_gpu(proc.pid) else 0.0
                )
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                status.cpu_usage = 0.0
                status.memory_usage = 0.0
                status.gpu_usage = 0.0
                self.node_procs.pop(node_name, None)

            cpu_peak = max(cpu_peak, status.cpu_usage)
            gpu_peak = max(gpu_peak, status.gpu_usage)
            report.nodes.append(status)

        return report, cpu_peak, gpu_peak

    def _attach_process(self, node_name: str) -> Optional[psutil.Process]:
        """Find and attach a process for a ROS node name."""
        pid = self._find_pid(node_name)
        if pid is None:
            return None
        try:
            proc = psutil.Process(pid)
            proc.cpu_percent()
            self.node_procs[node_name] = proc
            self.newly_attached.add(node_name)
            self.get_logger().info(f"Monitor attached to {node_name} (PID: {pid})")
            return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return None

    def _find_pid(self, node_name: str) -> Optional[int]:
        """Resolve PID by ROS remapping args (`__node:=`) or fallback cmdline name match."""
        candidates = []
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmdline = proc.info["cmdline"]
                if not cmdline:
                    continue
                cmd_str = " ".join(cmdline)
                if f"__node:={node_name}" in cmd_str:
                    return int(proc.info["pid"])
                if node_name in cmd_str:
                    candidates.append(int(proc.info["pid"]))
                    continue
                if proc.info["name"] and node_name in proc.info["name"]:
                    candidates.append(int(proc.info["pid"]))
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        return candidates[0] if candidates else None

    def _get_gpu_load(self) -> float:
        """Best-effort GPU load in percent from jtop/sysfs/NVML."""
        if self.jtop_controller and self.jtop_controller.ok():
            try:
                gpu_data = self.jtop_controller.gpu
                if isinstance(gpu_data, dict):
                    gpu_info = gpu_data.get("gpu", {})
                    if isinstance(gpu_info, dict):
                        load = gpu_info.get("status", {}).get("load")
                        if load is not None:
                            return float(load)
            except Exception:
                pass

        if self.gpu_sysfs_path:
            try:
                with open(self.gpu_sysfs_path, "r", encoding="utf-8") as handle:
                    return float(handle.read().strip()) / 10.0
            except Exception:
                pass

        if self.gpu_initialized:
            try:
                handle = nvmlDeviceGetHandleByIndex(0)
                rates = nvmlDeviceGetUtilizationRates(handle)
                return float(rates.gpu)
            except Exception:
                pass

        return 0.0

    def _process_uses_gpu(self, pid: int) -> bool:
        """Jetson-friendly heuristic: checks open FDs for nvhost usage."""
        pids = [pid]
        try:
            pids.extend(
                [child.pid for child in psutil.Process(pid).children(recursive=True)]
            )
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

        for curr_pid in pids:
            fd_dir = f"/proc/{curr_pid}/fd"
            try:
                for fd in os.listdir(fd_dir):
                    try:
                        if "nvhost" in os.readlink(f"{fd_dir}/{fd}"):
                            return True
                    except OSError:
                        continue
            except OSError:
                continue
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionCentral()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.jtop_controller:
            try:
                node.jtop_controller.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

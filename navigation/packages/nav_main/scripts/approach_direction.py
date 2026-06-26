#!/usr/bin/env python3
"""Approach Direction — drive/strafe the holonomic omnibase in a COMMANDED
direction (forwards/backwards/left/right) until the nearest lidar reading in that
direction is N cm away, then stop.

This is the simpler sibling of table_docker.py: the direction is GIVEN by the
caller, not derived from the scan, so there is no RANSAC, no face locking, no TF
reprojection, and no yaw/lateral alignment. The only control problem is "drive at
a proportional speed along one fixed base_link axis, using the live
nearest-reading-in-that-direction as the stopping condition".

Layering (mirrors table_docker / nav_central):
    task_manager -> ApproachDirection @ APPROACH_DIRECTION_SERVICE (nav_central)
                 -> ApproachDirection @ APPROACH_DIRECTION_EXEC_SERVICE (this node)

It runs only inside the service callback — no free-running timer. Holonomic base
only (left/right need strafing), so it is launched omnibase-only like table_docker.
"""

import math
import time
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

from frida_constants.navigation_constants import (
    APPROACH_DIRECTION_EXEC_SERVICE,
    SCAN_TOPIC,
)
from frida_interfaces.srv import ApproachDirection


# Direction name -> center angle in base_link (rad) and the (vx_sign, vy_sign) of
# the unit motion vector. forwards = +x, left = +y (REP-103).
_DIRECTIONS = {
    "forwards": (0.0, (1.0, 0.0)),
    "forward": (0.0, (1.0, 0.0)),
    "backwards": (math.pi, (-1.0, 0.0)),
    "backward": (math.pi, (-1.0, 0.0)),
    "left": (math.pi / 2.0, (0.0, 1.0)),
    "right": (-math.pi / 2.0, (0.0, -1.0)),
}

# Aliases → canonical key used for per-direction parameters and cone lookup.
_CANONICAL = {
    "forwards": "forwards", "forward": "forwards",
    "backwards": "backwards", "backward": "backwards",
    "left": "left", "right": "right",
}


class ApproachDirectionNode(Node):
    def __init__(self):
        super().__init__("approach_direction")

        # --- Topics / frames ---
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.scan_topic = self.declare_parameter("scan_topic", SCAN_TOPIC).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # --- Per-direction angular windows (asymmetric offsets from direction center) ---
        # Each direction exposes {dir}_angle_min_deg / {dir}_angle_max_deg as runtime
        # parameters. Values are offsets in degrees relative to the direction's center
        # angle (0° for forwards, 180° for backwards, 90° for left, -90° for right).
        # Defaults ±15°. Stored internally in radians as (min, max) tuples.
        self._direction_cones: dict[str, tuple[float, float]] = {}
        for d in ("forwards", "backwards", "left", "right"):
            mn = float(self.declare_parameter(f"{d}_angle_min_deg", -15.0).value)
            mx = float(self.declare_parameter(f"{d}_angle_max_deg",  15.0).value)
            self._direction_cones[d] = (math.radians(mn), math.radians(mx))

        self.max_detect_range = self.declare_parameter("max_detect_range", 3.0).value

        # --- Per-direction body offset (base_link -> FOOTPRINT EDGE in that dir) ---
        # The requested distance_cm is the gap from the robot's FOOTPRINT EDGE in
        # the commanded direction to the obstacle (clearance = nearest_reading -
        # offset), so it matches the footprint nav2 uses for collision.
        #
        # Defaults are the half-extents of the omnibase nav2 footprint
        #   nav2_omni.yaml: [ [0.325, 0.25], [0.325,-0.25], [-0.325,-0.25], [-0.325,0.25] ]
        #   -> front/back = 0.325 m, left/right = 0.25 m (in base_link).
        self.offset_forwards = self.declare_parameter("offset_forwards", 0.325).value
        self.offset_backwards = self.declare_parameter("offset_backwards", 0.325).value
        self.offset_left = self.declare_parameter("offset_left", 0.25).value
        self.offset_right = self.declare_parameter("offset_right", 0.25).value

        # --- Approach targets / safety ---
        self.min_safe = self.declare_parameter("min_safe", 0.05).value      # hard floor (m, from body offset)
        self.dist_tol = self.declare_parameter("dist_tol", 0.03).value

        # --- Control gains / limits ---
        self.k = self.declare_parameter("k", 0.9).value
        self.max_vx = self.declare_parameter("max_vx", 0.22).value
        self.max_vy = self.declare_parameter("max_vy", 0.30).value
        self.min_v = self.declare_parameter("min_v", 0.04).value            # crawl floor near the goal
        self.control_rate = self.declare_parameter("control_rate", 15.0).value
        self.approach_timeout = self.declare_parameter("approach_timeout", 40.0).value
        self.settle_cycles = int(self.declare_parameter("settle_cycles", 3).value)

        # --- Debug scan republishing ---
        # When pub_scan_debug is true, each incoming /scan is filtered to only the
        # angular window of scan_debug_direction and republished on scan_debug_topic.
        # All out-of-window ranges are set to inf so the LaserScan shape is preserved.
        self.pub_scan_debug = bool(self.declare_parameter("pub_scan_debug", False).value)
        self.scan_debug_direction = str(
            self.declare_parameter("scan_debug_direction", "forwards").value
        )
        scan_debug_topic = str(
            self.declare_parameter("scan_debug_topic", "/scan_direction").value
        )

        # --- State ---
        self._lock = threading.Lock()
        self._scan: LaserScan | None = None

        sensor_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb,
                                 qos_profile_sensor_data, callback_group=sensor_cb)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self._debug_pub = self.create_publisher(LaserScan, scan_debug_topic, 10)
        self.create_service(ApproachDirection, APPROACH_DIRECTION_EXEC_SERVICE,
                            self._approach_cb, callback_group=srv_cb)

        self.add_on_set_parameters_callback(self._on_set_params)

        cone_summary = {d: (round(math.degrees(v[0]), 1), round(math.degrees(v[1]), 1))
                        for d, v in self._direction_cones.items()}
        self.log("info", f"approach_direction ready. service={APPROACH_DIRECTION_EXEC_SERVICE} "
                         f"scan={self.scan_topic} cones={cone_summary}")

    # ------------------------------------------------------------------ utils
    def _on_set_params(self, params):
        for p in params:
            name = p.name
            # Per-direction cone parameters — must be matched before the generic fallback.
            matched = False
            for d in ("forwards", "backwards", "left", "right"):
                if name == f"{d}_angle_min_deg":
                    _, mx = self._direction_cones[d]
                    self._direction_cones[d] = (math.radians(float(p.value)), mx)
                    matched = True
                    break
                if name == f"{d}_angle_max_deg":
                    mn, _ = self._direction_cones[d]
                    self._direction_cones[d] = (mn, math.radians(float(p.value)))
                    matched = True
                    break
            if matched:
                continue
            if name == "pub_scan_debug":
                self.pub_scan_debug = bool(p.value)
            elif name == "scan_debug_direction":
                self.scan_debug_direction = str(p.value)
            elif hasattr(self, name):
                setattr(self, name, p.value)
        return SetParametersResult(successful=True)

    def log(self, level, msg):
        text = f"ApproachDirection: {msg}"
        if level == "warn":
            self.get_logger().warn(text)
        elif level == "error":
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
        if self.pub_scan_debug:
            self._publish_debug_scan(msg)

    def _publish_debug_scan(self, scan: LaserScan):
        """Republish scan filtered to the angular window of scan_debug_direction."""
        direction = _CANONICAL.get(self.scan_debug_direction.strip().lower())
        if direction is None:
            return
        center, _ = _DIRECTIONS[direction]
        cone_min, cone_max = self._direction_cones[direction]

        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        da = np.arctan2(np.sin(angles - center), np.cos(angles - center))
        in_cone = (da >= cone_min) & (da <= cone_max)
        filtered = np.where(in_cone, ranges, np.inf)

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = filtered.tolist()
        out.intensities = scan.intensities
        self._debug_pub.publish(out)

    def _publish_cmd(self, vx=0.0, vy=0.0):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = 0.0
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def _cone_nearest(self, canonical: str, center: float) -> float | None:
        """Nearest scan range (m) within the per-direction angular window, or None."""
        with self._lock:
            scan = self._scan
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return None
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        da = np.arctan2(np.sin(angles - center), np.cos(angles - center))
        cone_min, cone_max = self._direction_cones[canonical]
        valid = (
            np.isfinite(ranges)
            & (ranges > max(scan.range_min, 1e-3))
            & (ranges < self.max_detect_range)
            & (da >= cone_min)
            & (da <= cone_max)
        )
        if not np.any(valid):
            return None
        return float(np.min(ranges[valid]))

    def _direction_offset(self, canonical: str) -> float:
        return getattr(self, f"offset_{canonical}", self.offset_forwards)

    # ----------------------------------------------------------------- service
    def _approach_cb(self, request, response):
        direction = (request.direction or "").strip().lower()
        if direction not in _DIRECTIONS:
            response.success = False
            response.error = (f"Unknown direction '{request.direction}'. Use one of: "
                              f"forwards, backwards, left, right")
            self.log("error", response.error)
            self._stop()
            return response

        canonical = _CANONICAL[direction]
        center, (sx, sy) = _DIRECTIONS[direction]
        offset = self._direction_offset(canonical)
        target_m = max(float(request.distance_cm), 0.0) / 100.0
        max_speed = self.max_vx if sy == 0.0 else self.max_vy

        cone_min, cone_max = self._direction_cones[canonical]
        self.log("info", f"Approach '{direction}' until {target_m:.2f} m "
                         f"(offset={offset:.2f} cone=[{math.degrees(cone_min):.1f}, "
                         f"{math.degrees(cone_max):.1f}]deg)")

        dt = 1.0 / self.control_rate
        deadline = time.time() + self.approach_timeout
        settled = 0

        while rclpy.ok():
            if time.time() > deadline:
                self._stop()
                response.success = False
                response.error = "Approach timed out before reaching target distance"
                self.log("error", response.error)
                return response

            nearest = self._cone_nearest(canonical, center)
            clearance = None if nearest is None else (nearest - offset)

            safety_stop = clearance is not None and clearance <= self.min_safe
            at_target = clearance is not None and clearance <= target_m + self.dist_tol

            if at_target or safety_stop:
                settled += 1
                if settled >= self.settle_cycles:
                    self._stop()
                    response.success = True
                    near_txt = "--" if nearest is None else f"{nearest:.3f}"
                    response.error = ""
                    self.log("info", f"Reached: clearance="
                             f"{'--' if clearance is None else round(clearance, 3)}m "
                             f"nearest={near_txt}m")
                    return response
            else:
                settled = 0

            if clearance is None:
                # Nothing within max_detect_range in this direction -> path is clear,
                # drive at the capped speed (the timeout bounds a runaway).
                speed = max_speed
            else:
                e = clearance - target_m
                speed = self._clamp(self.k * e, max_speed)
                if e > self.dist_tol and 0.0 < speed < self.min_v:
                    speed = self.min_v
                if clearance <= self.min_safe:
                    speed = 0.0

            self._publish_cmd(speed * sx, speed * sy)
            time.sleep(dt)

        self._stop()
        response.success = False
        response.error = "Approach aborted (shutdown)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ApproachDirectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

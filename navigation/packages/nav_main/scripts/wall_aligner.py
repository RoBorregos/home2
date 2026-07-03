#!/usr/bin/env python3
"""Washing-machine precision aligner — live-lidar square-up + exact-distance
approach for the holonomic omnibase.

Different job from table_docker: table_docker fits a face ONCE, locks it in
odom and drives against the locked face (jitter-free, but open-loop w.r.t. the
real surface while moving). This node never locks anything — every control
cycle it re-fits the NEAREST straight segment the lidar sees in front (the
washing machine's front panel: a bounded line whose two endpoints are visible
in the scan) and servos on that live fit, so the robot stays perpendicular to
the REAL surface the whole time. Slow speeds, bidirectional final correction
and a stationary multi-scan verification give cm-level repeatability.

Services (names in frida_constants.navigation_constants)
--------------------------------------------------------
  WALL_ALIGN_SERVICE (AlignToWall): rotate in place (optionally also strafe to
      the segment midpoint) until base +x lies along the face normal, then
      verify with the robot stopped. The response reports the measured
      perpendicular distance — park the robot at the desired final depth and
      call this to CALIBRATE the target for the close service.
  WALL_CLOSE_SERVICE (CloseToWall): creep straight in (or back out) until the
      perpendicular distance to the live fitted line equals the request,
      holding yaw square on the live fit and lateral position on odom.

Segment pick rule: cluster the front-sector scan by point-to-point gaps, PCA
line-fit each cluster, keep clusters that are straight (low rms) and at least
min_segment_length long, take the one closest to the robot.

RViz: MarkerArray on /wall_aligner/markers (segment, normal, yaw/dist text).
The scan is treated as base-frame with 0 rad straight ahead, same convention
as table_docker and the door check.
"""

import math
import time
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from frida_constants.navigation_constants import (
    WALL_ALIGN_SERVICE,
    WALL_CLOSE_SERVICE,
    SCAN_TOPIC,
)
from frida_interfaces.srv import AlignToWall, CloseToWall


def reduce_angle_mod_pi(a):
    """Map an (undirected) line/normal angle to (-pi/2, pi/2]."""
    a = math.atan2(math.sin(a), math.cos(a))
    if a > math.pi / 2:
        a -= math.pi
    elif a <= -math.pi / 2:
        a += math.pi
    return a


class WallAligner(Node):
    def __init__(self):
        super().__init__("wall_aligner")

        # --- Topics / frames ---
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.scan_topic = self.declare_parameter("scan_topic", SCAN_TOPIC).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        # --- Segment detection ---
        self.frontal_fov = math.radians(self.declare_parameter("frontal_fov_deg", 100.0).value)
        self.max_detect_range = self.declare_parameter("max_detect_range", 2.0).value
        # Euclidean gap between consecutive (angle-ordered) points that breaks a cluster.
        self.cluster_gap = self.declare_parameter("cluster_gap", 0.10).value
        self.min_cluster_points = int(self.declare_parameter("min_cluster_points", 10).value)
        # A cluster only qualifies as "the machine" if it is straight and long
        # enough — kills chair legs / cables so the CLOSEST qualifying segment
        # really is the front panel.
        self.min_segment_length = self.declare_parameter("min_segment_length", 0.30).value
        self.max_line_rms = self.declare_parameter("max_line_rms", 0.015).value

        # --- Tolerances (the cm-precision knobs) ---
        self.yaw_tol = self.declare_parameter("yaw_tol", 0.008).value      # rad (~0.46 deg)
        self.y_tol = self.declare_parameter("y_tol", 0.02).value           # m (centering)
        self.dist_tol = self.declare_parameter("dist_tol", 0.008).value    # m
        self.settle_cycles = int(self.declare_parameter("settle_cycles", 4).value)
        # Stationary verification: median over this many FRESH scans, robot stopped.
        self.verify_samples = int(self.declare_parameter("verify_samples", 5).value)
        self.settle_wait = self.declare_parameter("settle_wait", 0.35).value
        self.measure_timeout = self.declare_parameter("measure_timeout", 3.0).value

        # --- Control gains / limits (slow on purpose) ---
        self.control_rate = self.declare_parameter("control_rate", 15.0).value
        self.k_yaw = self.declare_parameter("k_yaw", 1.2).value
        self.k_y = self.declare_parameter("k_y", 0.8).value
        self.k_x = self.declare_parameter("k_x", 0.7).value
        self.max_wz = self.declare_parameter("max_wz", 0.30).value
        self.max_vx = self.declare_parameter("max_vx", 0.06).value
        self.max_vy = self.declare_parameter("max_vy", 0.12).value
        # Sign-preserving command floors so the base doesn't stall on stiction
        # during the last mm/mrad (applied only while outside tolerance).
        self.min_wz = self.declare_parameter("min_wz", 0.04).value
        self.min_v = self.declare_parameter("min_v", 0.015).value

        # --- Align service ---
        self.align_timeout = self.declare_parameter("align_timeout", 30.0).value
        self.align_attempts = int(self.declare_parameter("align_attempts", 4).value)

        # --- Close service ---
        self.close_timeout = self.declare_parameter("close_timeout", 60.0).value
        # Abort close if the face reads more than this off square (call align first).
        self.close_yaw_gate = self.declare_parameter("close_yaw_gate", 0.09).value  # rad (~5 deg)
        self.max_travel_default = self.declare_parameter("max_travel_default", 1.0).value
        # Hard floor on the fitted-line distance (minus front_offset); requests
        # below it are rejected and forward motion never pushes past it.
        self.min_distance = self.declare_parameter("min_distance", 0.10).value
        # Raw-beam guard: any front-sector (±10 deg) beam under this blocks
        # forward motion, independent of the fit (protrusions the line misses).
        self.hard_stop_range = self.declare_parameter("hard_stop_range", 0.10).value

        # Distance is measured base_link -> fitted line; subtract this to report
        # it from the robot's front-most part instead (0 = raw base_link).
        self.front_offset = self.declare_parameter("front_offset", 0.0).value

        self.max_fit_fail = int(self.declare_parameter("max_fit_fail", 15).value)
        self.marker_lifetime = self.declare_parameter("marker_lifetime", 1.0).value

        # --- State ---
        self._lock = threading.Lock()
        self._scan = None

        sensor_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()  # align/close never overlap

        # /scan is BEST_EFFORT (see nav_central check_door / move_relative) — a
        # RELIABLE subscription would silently receive nothing.
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb,
                                 qos_profile_sensor_data, callback_group=sensor_cb)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/wall_aligner/markers", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(AlignToWall, WALL_ALIGN_SERVICE, self._align_cb,
                            callback_group=srv_cb)
        self.create_service(CloseToWall, WALL_CLOSE_SERVICE, self._close_cb,
                            callback_group=srv_cb)

        self.add_on_set_parameters_callback(self._on_set_params)

        self.log("info", f"wall_aligner ready. align={WALL_ALIGN_SERVICE} "
                         f"close={WALL_CLOSE_SERVICE} yaw_tol={self.yaw_tol}rad "
                         f"dist_tol={self.dist_tol}m fov={math.degrees(self.frontal_fov):.0f}deg")

    def _on_set_params(self, params):
        """Sync runtime param changes into the cached attributes used by the loops."""
        for p in params:
            if p.name == "frontal_fov_deg":
                self.frontal_fov = math.radians(float(p.value))
            elif hasattr(self, p.name):
                setattr(self, p.name, p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ utils
    def log(self, level, msg):
        text = f"WallAligner: {msg}"
        if level == "warn":
            self.get_logger().warn(text)
        elif level == "error":
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _scan_cb(self, msg):
        with self._lock:
            self._scan = msg

    def _latest_scan(self):
        with self._lock:
            return self._scan

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def _floor_clamp(self, v, floor, lim):
        """Clamp magnitude into [floor, lim], keeping the sign (anti-stiction)."""
        if v == 0.0:
            return 0.0
        mag = min(max(abs(v), floor), lim)
        return math.copysign(mag, v)

    def _odom_pose(self):
        """(x, y, yaw) of base_link in odom, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, RclpyTime(),
                timeout=Duration(seconds=0.5))
        except Exception as e:
            self.log("warn", f"odom TF unavailable: {e}")
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    # ------------------------------------------------------- segment detection
    def _front_points(self, scan):
        """Front-sector scan points (base frame, angle-ordered) as Nx2 array."""
        if scan is None:
            return np.empty((0, 2))
        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return np.empty((0, 2))
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        valid = (np.isfinite(ranges)
                 & (ranges > max(scan.range_min, 1e-3))
                 & (ranges < self.max_detect_range)
                 & (np.abs(wrapped) < self.frontal_fov / 2.0))
        r, a = ranges[valid], wrapped[valid]
        pts = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        pts = pts[pts[:, 0] > 0.0]
        if len(pts) == 0:
            return pts
        order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
        return pts[order]

    def _fit_segment(self, pts):
        """PCA line fit of one cluster (one outlier-trim refit). Returns the
        segment dict or None if it isn't a straight, long-enough face."""
        for _ in range(2):
            centroid = pts.mean(axis=0)
            _, _, vv = np.linalg.svd(pts - centroid)
            direction = vv[0] / (np.linalg.norm(vv[0]) + 1e-12)
            normal = np.array([-direction[1], direction[0]])
            resid = (pts - centroid) @ normal
            rms = float(np.sqrt(np.mean(resid ** 2)))
            keep = np.abs(resid) < max(2.5 * rms, 0.01)
            if keep.sum() < self.min_cluster_points or keep.all():
                break
            pts = pts[keep]
        if len(pts) < self.min_cluster_points or rms > self.max_line_rms:
            return None
        if float(normal @ centroid) < 0:      # point the normal toward the face
            normal = -normal
        t = (pts - centroid) @ direction
        p1 = centroid + direction * float(t.min())
        p2 = centroid + direction * float(t.max())
        length = float(t.max() - t.min())
        if length < self.min_segment_length:
            return None
        distance = float(normal @ centroid)   # perpendicular base_link -> line
        if distance < 0.05:
            return None
        mid = 0.5 * (p1 + p2)
        return {
            "normal": normal, "centroid": centroid, "p1": p1, "p2": p2,
            "length": length, "rms": rms, "distance": distance,
            "mid_y": float(mid[1]),
            "e_yaw": reduce_angle_mod_pi(math.atan2(normal[1], normal[0])),
        }

    def _nearest_segment(self, scan):
        """Closest straight, long-enough segment in the front sector, or None."""
        pts = self._front_points(scan)
        if len(pts) < self.min_cluster_points:
            return None
        # Break clusters where consecutive (angle-ordered) points jump apart.
        gaps = np.hypot(*(np.diff(pts, axis=0).T))
        breaks = np.where(gaps > self.cluster_gap)[0] + 1
        best = None
        for cluster in np.split(pts, breaks):
            if len(cluster) < self.min_cluster_points:
                continue
            seg = self._fit_segment(cluster)
            if seg is not None and (best is None or seg["distance"] < best["distance"]):
                best = seg
        return best

    def _raw_front_min(self, scan, half_angle=math.radians(10.0)):
        """Min raw beam in the narrow front sector (protrusion guard), or None."""
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        if len(ranges) == 0:
            return None
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        valid = (np.isfinite(ranges) & (ranges > max(scan.range_min, 1e-3))
                 & (np.abs(wrapped) <= half_angle))
        return float(ranges[valid].min()) if valid.any() else None

    def _measure(self):
        """Stationary ground truth: stop, wait for verify_samples FRESH scans,
        fit each, return the medians dict or None. Robot must be stopped."""
        self._stop()
        time.sleep(self.settle_wait)
        fits = []
        last_stamp = None
        deadline = time.time() + self.measure_timeout
        while rclpy.ok() and len(fits) < self.verify_samples and time.time() < deadline:
            scan = self._latest_scan()
            if scan is None:
                time.sleep(0.02)
                continue
            stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
            if stamp == last_stamp:
                time.sleep(0.02)
                continue
            last_stamp = stamp
            seg = self._nearest_segment(scan)
            if seg is not None:
                fits.append(seg)
                self._publish_markers(seg)
        if len(fits) < max(2, self.verify_samples // 2):
            return None
        return {k: float(np.median([f[k] for f in fits]))
                for k in ("e_yaw", "distance", "mid_y", "length")}

    # ----------------------------------------------------------- visualization
    def _publish_markers(self, seg):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        life = Duration(seconds=self.marker_lifetime).to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = now
            m.ns = "wall_aligner"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.lifetime = life
            m.pose.orientation.w = 1.0
            return m

        p1, p2, n, c = seg["p1"], seg["p2"], seg["normal"], seg["centroid"]

        line = base(0, Marker.LINE_STRIP)
        line.scale.x = 0.03
        line.color.g = 1.0
        line.color.r = 0.4
        line.color.a = 1.0
        line.points = [Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
                       Point(x=float(p2[0]), y=float(p2[1]), z=0.0)]
        arr.markers.append(line)

        arrow = base(1, Marker.ARROW)
        arrow.scale.x = 0.02
        arrow.scale.y = 0.05
        arrow.scale.z = 0.07
        arrow.color.b = 1.0
        arrow.color.g = 0.4
        arrow.color.a = 1.0
        arrow.points = [Point(x=float(c[0]), y=float(c[1]), z=0.0),
                        Point(x=float(c[0] - n[0] * 0.3), y=float(c[1] - n[1] * 0.3), z=0.0)]
        arr.markers.append(arrow)

        txt = base(2, Marker.TEXT_VIEW_FACING)
        txt.scale.z = 0.10
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.position.x = float(c[0])
        txt.pose.position.y = float(c[1])
        txt.pose.position.z = 0.30
        txt.text = (f"yaw={math.degrees(seg['e_yaw']):+.1f}deg "
                    f"d={seg['distance'] - self.front_offset:.3f}m len={seg['length']:.2f}m")
        arr.markers.append(txt)

        self.marker_pub.publish(arr)

    # ----------------------------------------------------------------- align
    def _servo_align(self, center, deadline):
        """Rotate (and optionally strafe) on the live per-scan fit until inside
        tolerance for settle_cycles. Returns 'settled' | 'lost' | 'timeout'."""
        dt = 1.0 / self.control_rate
        settled = 0
        fit_fail = 0
        while rclpy.ok() and time.time() < deadline:
            seg = self._nearest_segment(self._latest_scan())
            if seg is None:
                fit_fail += 1
                self._stop()
                if fit_fail > self.max_fit_fail:
                    return "lost"
                time.sleep(dt)
                continue
            fit_fail = 0
            self._publish_markers(seg)
            e_yaw = seg["e_yaw"]
            y_mid = seg["mid_y"] if center else 0.0
            ok_yaw = abs(e_yaw) <= self.yaw_tol
            ok_y = (not center) or abs(y_mid) <= self.y_tol
            if ok_yaw and ok_y:
                settled += 1
                self._stop()
                if settled >= self.settle_cycles:
                    return "settled"
            else:
                settled = 0
                wz = 0.0 if ok_yaw else self._floor_clamp(self.k_yaw * e_yaw,
                                                          self.min_wz, self.max_wz)
                vy = 0.0
                if center and not ok_y:
                    vy = self._floor_clamp(self.k_y * y_mid, self.min_v, self.max_vy)
                self._publish_cmd(0.0, vy, wz)
            time.sleep(dt)
        return "timeout"

    def _align_cb(self, request, response):
        old_range = self.max_detect_range
        if request.max_range > 0.0:
            self.max_detect_range = float(request.max_range)
        self.log("info", f"Align requested (center={request.center} "
                         f"range={self.max_detect_range:.2f} m)")
        try:
            deadline = time.time() + self.align_timeout
            measured = None
            for attempt in range(1, self.align_attempts + 1):
                outcome = self._servo_align(request.center, deadline)
                if outcome == "lost":
                    response.error = ("No straight segment in front of the robot "
                                      f"(len>={self.min_segment_length}m within "
                                      f"{self.max_detect_range}m)")
                    break
                if outcome == "timeout":
                    response.error = "Align timed out before converging"
                    break
                measured = self._measure()
                if measured is None:
                    response.error = "Verification lost the face after settling"
                    break
                if abs(measured["e_yaw"]) <= self.yaw_tol and (
                        not request.center or abs(measured["mid_y"]) <= self.y_tol):
                    response.success = True
                    response.distance = measured["distance"] - self.front_offset
                    response.yaw_error = measured["e_yaw"]
                    response.segment_length = measured["length"]
                    self.log("info", f"Aligned: yaw={math.degrees(measured['e_yaw']):+.2f}deg "
                                     f"d={response.distance:.3f}m len={measured['length']:.2f}m "
                                     f"(attempt {attempt})")
                    return response
                self.log("warn", f"Verify off tolerance "
                                 f"(yaw={math.degrees(measured['e_yaw']):+.2f}deg, "
                                 f"mid_y={measured['mid_y']:+.3f}m) — touch-up {attempt}")
            else:
                response.error = f"Did not converge in {self.align_attempts} attempts"
            if measured is not None:
                response.distance = measured["distance"] - self.front_offset
                response.yaw_error = measured["e_yaw"]
                response.segment_length = measured["length"]
            response.success = False
            self.log("error", response.error)
            return response
        finally:
            self.max_detect_range = old_range
            self._stop()

    # ----------------------------------------------------------------- close
    def _close_cb(self, request, response):
        target = float(request.distance)
        max_travel = float(request.max_travel) if request.max_travel > 0.0 \
            else self.max_travel_default
        response.success = False
        if target < self.min_distance:
            response.error = (f"Target {target:.3f} m is below min_distance "
                              f"{self.min_distance:.2f} m")
            self.log("error", response.error)
            return response

        measured = self._measure()
        if measured is None:
            response.error = "No straight segment in front of the robot"
            self.log("error", response.error)
            return response
        if abs(measured["e_yaw"]) > self.close_yaw_gate:
            response.error = (f"Face reads {math.degrees(measured['e_yaw']):+.1f} deg off "
                              f"square — call {WALL_ALIGN_SERVICE} first")
            self.log("error", response.error)
            return response

        start = self._odom_pose()
        if start is None:
            response.error = "odom TF unavailable"
            self.log("error", response.error)
            return response
        x0, y0, yaw0 = start

        self.log("info", f"Close requested: target={target:.3f} m "
                         f"(now {measured['distance'] - self.front_offset:.3f} m, "
                         f"cap {max_travel:.2f} m)")

        dt = 1.0 / self.control_rate
        deadline = time.time() + self.close_timeout
        settled = 0
        fit_fail = 0
        traveled = 0.0
        try:
            while rclpy.ok() and time.time() < deadline:
                pose = self._odom_pose()
                if pose is None:
                    response.error = "odom TF lost mid-drive"
                    response.traveled = traveled
                    self.log("error", response.error)
                    return response
                rx, ry, _ = pose
                traveled = math.cos(yaw0) * (rx - x0) + math.sin(yaw0) * (ry - y0)
                e_lat = -math.sin(yaw0) * (rx - x0) + math.cos(yaw0) * (ry - y0)
                if abs(traveled) > max_travel:
                    response.error = f"Travel cap {max_travel:.2f} m reached"
                    response.traveled = traveled
                    self.log("error", response.error)
                    return response

                scan = self._latest_scan()
                seg = self._nearest_segment(scan)
                if seg is None:
                    fit_fail += 1
                    self._stop()
                    if fit_fail > self.max_fit_fail:
                        response.error = "Lost the face mid-drive"
                        response.traveled = traveled
                        self.log("error", response.error)
                        return response
                    time.sleep(dt)
                    continue
                fit_fail = 0
                self._publish_markers(seg)

                d = seg["distance"] - self.front_offset
                e_x = d - target
                if abs(e_x) <= self.dist_tol:
                    settled += 1
                    self._stop()
                    if settled >= self.settle_cycles:
                        final = self._measure()
                        if final is not None and \
                                abs(final["distance"] - self.front_offset - target) <= self.dist_tol:
                            pose = self._odom_pose()
                            if pose is not None:
                                traveled = (math.cos(yaw0) * (pose[0] - x0)
                                            + math.sin(yaw0) * (pose[1] - y0))
                            response.success = True
                            response.traveled = traveled
                            response.final_distance = final["distance"] - self.front_offset
                            self.log("info", f"Closed: d={response.final_distance:.3f} m "
                                             f"(target {target:.3f}) traveled={traveled:+.3f} m "
                                             f"yaw={math.degrees(final['e_yaw']):+.2f}deg")
                            return response
                        settled = 0  # verify disagreed — keep servoing
                else:
                    settled = 0
                    vx = self._floor_clamp(self.k_x * e_x, self.min_v, self.max_vx)
                    if vx > 0.0:
                        raw = self._raw_front_min(scan)
                        if d <= self.min_distance or (raw is not None
                                                      and raw <= self.hard_stop_range):
                            vx = 0.0
                    wz = self._clamp(self.k_yaw * seg["e_yaw"], self.max_wz)
                    vy = self._clamp(-self.k_y * e_lat, self.max_vy)
                    self._publish_cmd(vx, vy, wz)
                time.sleep(dt)
            response.error = "Close timed out before converging"
            response.traveled = traveled
            self.log("error", response.error)
            return response
        finally:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = WallAligner()
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

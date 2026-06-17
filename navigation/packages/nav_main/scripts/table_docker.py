#!/usr/bin/env python3
"""Table/shelf docking — perpendicular approach for the holonomic omnibase.

Idea: nav2 brings the robot to a STATIC "near" pose in front of a table/shelf.
On a service call this node detects the table's front face ONCE from a few
accumulated cloud/scan frames (stable RANSAC line -> polygon -> normal vector),
LOCKS that face in the odom frame, then closed-loop drives the (holonomic) base
so it ends up PERPENDICULAR to the face, centered on it, and as close as the arm
can safely get. Locking the orientation kills the per-frame jitter you get from
re-fitting every tick; the live lidar is still used as a safety stop.

Flow
----
  1. nav2 -> static near pose (existing go_to_area).
  2. /navigation/preview_dock (std_srvs/Trigger): detect + publish RViz markers
     ONLY (no motion) — use this to check the fit/orientation before committing.
  3. /navigation/dock_to_surface (std_srvs/Trigger): detect+lock, then approach.
  4. /navigation/undock_from_surface (std_srvs/Trigger): back off retreat_distance
     (odom-measured) so nav2 can plan the next goal. nav_central calls this before
     every new location goal automatically.

Stop rule: the robot's FRONT-MOST part (the arm, at front_offset from base_link)
is kept target_distance from the surface, and never closer than min_safe to the
nearest thing the lidar sees. MEASURE front_offset.

Detection runs only when a service is called — there is no free-running timer.
"""

import math
import time
import threading
import collections

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAVE_PC2 = True
except Exception:
    _HAVE_PC2 = False

from frida_constants.navigation_constants import (
    DOCK_SERVICE,
    UNDOCK_SERVICE,
    DOCK_PREVIEW_SERVICE,
    DOCKED_TOPIC,
    SCAN_TOPIC,
    POINT_CLOUD_TOPIC,
)


def quat_to_rot(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def reduce_angle_mod_pi(a):
    """Map an (undirected) line/normal angle to (-pi/2, pi/2]."""
    a = math.atan2(math.sin(a), math.cos(a))  # to (-pi, pi]
    if a > math.pi / 2:
        a -= math.pi
    elif a <= -math.pi / 2:
        a += math.pi
    return a


def ransac_line(pts, iters, thresh, min_inliers):
    """RANSAC 2D line fit. pts: Nx2. Returns dict (normal, direction, centroid,
    inlier mask, count) or None."""
    n = len(pts)
    if n < max(2, min_inliers):
        return None
    best_mask = None
    best_count = 0
    for _ in range(iters):
        i, j = np.random.randint(0, n), np.random.randint(0, n)
        if i == j:
            continue
        p1, p2 = pts[i], pts[j]
        d = p2 - p1
        norm = math.hypot(d[0], d[1])
        if norm < 1e-6:
            continue
        nx, ny = -d[1] / norm, d[0] / norm          # line normal
        c = -(nx * p1[0] + ny * p1[1])
        dist = np.abs(pts[:, 0] * nx + pts[:, 1] * ny + c)
        mask = dist < thresh
        cnt = int(mask.sum())
        if cnt > best_count:
            best_count, best_mask = cnt, mask
    if best_mask is None or best_count < min_inliers:
        return None
    # Total-least-squares refit on the inliers (PCA).
    inl = pts[best_mask]
    centroid = inl.mean(axis=0)
    _, _, vv = np.linalg.svd(inl - centroid)
    direction = vv[0] / (np.linalg.norm(vv[0]) + 1e-12)
    normal = np.array([-direction[1], direction[0]])
    return {"normal": normal, "direction": direction, "centroid": centroid,
            "mask": best_mask, "count": best_count}


class TableDocker(Node):
    def __init__(self):
        super().__init__("table_docker")

        # --- Topics / frames ---
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.scan_topic = self.declare_parameter("scan_topic", SCAN_TOPIC).value
        self.cloud_topic = self.declare_parameter("cloud_topic", POINT_CLOUD_TOPIC).value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/filtered").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        # --- Detection ---
        # 'scan' | 'cloud' | 'both'. Cloud catches the table EDGE/shelf face above
        # the lidar plane; scan is the reliable safety distance + a fallback face.
        self.detect_source = self.declare_parameter("detect_source", "both").value
        self.frontal_fov = math.radians(self.declare_parameter("frontal_fov_deg", 70.0).value)
        self.max_detect_range = self.declare_parameter("max_detect_range", 1.5).value
        self.det_min_height = self.declare_parameter("det_min_height", 0.20).value
        self.det_max_height = self.declare_parameter("det_max_height", 1.20).value
        self.ransac_iters = int(self.declare_parameter("ransac_iters", 300).value)
        self.ransac_thresh = self.declare_parameter("ransac_thresh", 0.03).value
        self.ransac_min_inliers = int(self.declare_parameter("ransac_min_inliers", 12).value)
        # How many recent cloud/scan frames to accumulate for ONE stable fit.
        self.num_samples = int(self.declare_parameter("num_samples", 3).value)
        self.collect_timeout = self.declare_parameter("collect_timeout", 3.0).value
        self.fit_attempts = int(self.declare_parameter("fit_attempts", 5).value)
        # Fit the FRONT EDGE only: keep the nearest point per angular bin (the
        # contour facing the robot) before RANSAC, so the line locks onto the table
        # edge instead of cutting through the middle of the table-top points.
        self.use_front_contour = self.declare_parameter("use_front_contour", True).value
        self.contour_bin_deg = self.declare_parameter("contour_bin_deg", 1.0).value

        # --- Geometry: forward reach of the robot's FRONT-MOST part (the arm) ---
        # Distance from base_link to whatever would hit the table first (arm tip in
        # its docking posture). MEASURE this and set it — the stop distances below
        # are clearances measured from THIS point, not from base_link.
        self.front_offset = self.declare_parameter("front_offset", 0.40).value          # base_link -> arm front (m)

        # --- Approach targets / safety (clearances from the arm front) ---
        self.target_distance = self.declare_parameter("target_distance", 0.10).value   # desired arm-front -> surface gap (m)
        self.min_safe = self.declare_parameter("min_safe", 0.05).value                 # hard floor arm-front -> nearest point (m)
        self.yaw_tol = self.declare_parameter("yaw_tol", 0.03).value
        self.y_tol = self.declare_parameter("y_tol", 0.03).value
        self.dist_tol = self.declare_parameter("dist_tol", 0.03).value

        # --- Control gains / limits ---
        self.k_yaw = self.declare_parameter("k_yaw", 0.8).value
        self.k_y = self.declare_parameter("k_y", 0.6).value
        self.k_x = self.declare_parameter("k_x", 0.5).value
        self.max_wz = self.declare_parameter("max_wz", 0.4).value
        self.max_vx = self.declare_parameter("max_vx", 0.12).value
        self.max_vy = self.declare_parameter("max_vy", 0.12).value
        self.control_rate = self.declare_parameter("control_rate", 15.0).value
        self.approach_timeout = self.declare_parameter("approach_timeout", 40.0).value
        self.settle_cycles = int(self.declare_parameter("settle_cycles", 5).value)
        self.max_tf_fail = int(self.declare_parameter("max_tf_fail", 30).value)

        # --- Retreat (undock) ---
        self.retreat_distance = self.declare_parameter("retreat_distance", 0.5).value
        self.retreat_speed = self.declare_parameter("retreat_speed", 0.12).value
        self.retreat_timeout = self.declare_parameter("retreat_timeout", 20.0).value

        # --- Visualization (RViz) ---
        self.polygon_depth = self.declare_parameter("polygon_depth", 0.40).value
        self.marker_lifetime = self.declare_parameter("marker_lifetime", 5.0).value

        # --- State (guarded by lock) ---
        self._lock = threading.Lock()
        self._scan = None
        self._odom = None
        self._cloud_buf = collections.deque(maxlen=max(self.num_samples, 1))
        self._scan_buf = collections.deque(maxlen=max(self.num_samples, 1))
        self._face = None       # locked face in odom frame
        self.docked = False

        sensor_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10, callback_group=sensor_cb)
        self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_cb, 1, callback_group=sensor_cb)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10, callback_group=sensor_cb)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        docked_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.docked_pub = self.create_publisher(Bool, DOCKED_TOPIC, docked_qos)
        self._publish_docked()
        # RViz: add a MarkerArray display on /table_docker/markers.
        self.marker_pub = self.create_publisher(MarkerArray, "/table_docker/markers", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, DOCK_SERVICE, self._dock_cb, callback_group=srv_cb)
        self.create_service(Trigger, UNDOCK_SERVICE, self._undock_cb, callback_group=srv_cb)
        self.create_service(Trigger, DOCK_PREVIEW_SERVICE, self._preview_cb, callback_group=srv_cb)

        self.log("info", f"table_docker ready. dock={DOCK_SERVICE} preview={DOCK_PREVIEW_SERVICE} "
                         f"samples={self.num_samples} front_offset={self.front_offset}m "
                         f"target={self.target_distance}m source={self.detect_source}")

    # ------------------------------------------------------------------ utils
    def log(self, level, msg):
        # Each severity on its own line — rclpy caches severity per call site.
        text = f"TableDocker: {msg}"
        if level == "warn":
            self.get_logger().warn(text)
        elif level == "error":
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _scan_cb(self, msg):
        with self._lock:
            self._scan = msg
            self._scan_buf.append(msg)

    def _cloud_cb(self, msg):
        with self._lock:
            self._cloud_buf.append(msg)

    def _odom_cb(self, msg):
        with self._lock:
            self._odom = msg

    def _publish_docked(self):
        self.docked_pub.publish(Bool(data=self.docked))

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish_cmd(0.0, 0.0, 0.0)

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def _lookup_Rt(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, RclpyTime())
        except Exception as e:
            self.log("warn", f"TF {target}<-{source} unavailable: {e}")
            return None
        q = tf.transform.rotation
        tr = tf.transform.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])

    @staticmethod
    def _xform_pt(R, t, p):
        return (R @ np.array([p[0], p[1], 0.0]) + t)[:2]

    @staticmethod
    def _xform_vec(R, v):
        return (R @ np.array([v[0], v[1], 0.0]))[:2]

    # -------------------------------------------------------------- point sets
    def _scan_points(self, scan):
        """Frontal scan points (base_link) within FOV + range as Nx2 array."""
        if scan is None:
            return np.empty((0, 2))
        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return np.empty((0, 2))
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > max(scan.range_min, 1e-3)) & (ranges < self.max_detect_range)
        valid &= np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < (self.frontal_fov / 2.0)
        r, a = ranges[valid], angles[valid]
        pts = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        return pts[pts[:, 0] > 0.0]

    def _cloud_points(self, cloud):
        """Frontal cloud points transformed to base_link, filtered by height band."""
        if cloud is None or not _HAVE_PC2:
            return np.empty((0, 2))
        try:
            raw = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.array([[p[0], p[1], p[2]] for p in raw], dtype=float)
            if arr.size == 0:
                return np.empty((0, 2))
            if len(arr) > 6000:
                arr = arr[:: len(arr) // 6000 + 1]
            if cloud.header.frame_id and cloud.header.frame_id != self.base_frame:
                Rt = self._lookup_Rt(self.base_frame, cloud.header.frame_id)
                if Rt is None:
                    return np.empty((0, 2))
                R, t = Rt
                arr = arr @ R.T + t
            m = (
                (arr[:, 2] > self.det_min_height)
                & (arr[:, 2] < self.det_max_height)
                & (arr[:, 0] > 0.0)
                & (np.hypot(arr[:, 0], arr[:, 1]) < self.max_detect_range)
                & (np.abs(np.arctan2(arr[:, 1], arr[:, 0])) < self.frontal_fov / 2.0)
            )
            return arr[m][:, :2]
        except Exception as e:
            self.log("warn", f"cloud read/transform failed ({e})")
            return np.empty((0, 2))

    def _accumulate_points(self):
        """Combine points from the last num_samples cloud/scan frames (base_link)."""
        with self._lock:
            clouds = list(self._cloud_buf)
            scans = list(self._scan_buf)
        parts = []
        if self.detect_source in ("scan", "both"):
            for s in scans:
                parts.append(self._scan_points(s))
        if self.detect_source in ("cloud", "both"):
            for c in clouds:
                parts.append(self._cloud_points(c))
        parts = [p for p in parts if len(p)]
        return np.vstack(parts) if parts else np.empty((0, 2))

    def _wait_for_samples(self):
        """Block briefly until enough recent frames are buffered (robot stationary)."""
        need_cloud = self.detect_source in ("cloud", "both")
        end = time.time() + self.collect_timeout
        while rclpy.ok() and time.time() < end:
            with self._lock:
                nc, ns = len(self._cloud_buf), len(self._scan_buf)
            if need_cloud and nc >= self.num_samples:
                return
            if not need_cloud and ns >= 1:
                return
            time.sleep(0.05)

    # ----------------------------------------------------------------- fit/lock
    def _front_contour(self, pts, bin_deg):
        """Keep only the NEAREST point per angular bin — the near-side contour
        facing the robot (the front edge), discarding the filled table interior so
        RANSAC doesn't cut a line through the middle of the surface points."""
        if len(pts) == 0:
            return pts
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        b = np.round(ang / math.radians(max(bin_deg, 0.1))).astype(np.int64)
        seen = set()
        keep = []
        for i in np.argsort(rng):            # nearest first
            bi = int(b[i])
            if bi not in seen:
                seen.add(bi)
                keep.append(i)
        return pts[np.array(keep, dtype=int)]

    def _fit_face(self):
        """One stable fit from the accumulated samples. Returns base_link geometry
        dict {normal, centroid, direction, p1, p2, pts, contour, nearest} or None."""
        pts = self._accumulate_points()
        if len(pts) < self.ransac_min_inliers:
            return None
        # Fit the front edge (contour), not the filled surface.
        fit_pts = self._front_contour(pts, self.contour_bin_deg) if self.use_front_contour else pts
        if len(fit_pts) < self.ransac_min_inliers:
            fit_pts = pts
        fit = ransac_line(fit_pts, self.ransac_iters, self.ransac_thresh, self.ransac_min_inliers)
        if fit is None:
            return None
        normal, centroid, direction = fit["normal"], fit["centroid"], fit["direction"]
        if np.dot(normal, centroid) < 0:        # point the normal toward the face
            normal = -normal
        inl = fit_pts[fit["mask"]]
        t = (inl - centroid) @ direction
        p1 = centroid + direction * float(t.min())
        p2 = centroid + direction * float(t.max())
        nearest = float(np.min(np.hypot(pts[:, 0], pts[:, 1])))
        return {"normal": normal, "centroid": centroid, "direction": direction,
                "p1": p1, "p2": p2, "pts": pts, "contour": fit_pts, "nearest": nearest}

    def _lock_face(self, fit):
        """Store the fitted face in the odom frame so it stays world-fixed while
        the robot drives (no per-cycle re-fit -> no orientation jitter)."""
        Rt = self._lookup_Rt(self.odom_frame, self.base_frame)
        if Rt is None:
            return False
        R, t = Rt
        with self._lock:
            self._face = {
                "normal": self._xform_vec(R, fit["normal"]),
                "centroid": self._xform_pt(R, t, fit["centroid"]),
                "p1": self._xform_pt(R, t, fit["p1"]),
                "p2": self._xform_pt(R, t, fit["p2"]),
            }
        return True

    def _face_in_base(self):
        """Transform the locked (odom) face into the CURRENT base_link frame."""
        with self._lock:
            face = self._face
        if face is None:
            return None
        Rt = self._lookup_Rt(self.base_frame, self.odom_frame)
        if Rt is None:
            return None
        R, t = Rt
        n = self._xform_vec(R, face["normal"])
        n = n / (np.linalg.norm(n) + 1e-12)
        c = self._xform_pt(R, t, face["centroid"])
        if np.dot(n, c) < 0:
            n = -n
        return {"normal": n, "centroid": c,
                "p1": self._xform_pt(R, t, face["p1"]),
                "p2": self._xform_pt(R, t, face["p2"])}

    def _live_nearest(self):
        """Closest frontal point from the latest scan (safety override)."""
        with self._lock:
            scan = self._scan
        sp = self._scan_points(scan)
        return float(np.min(np.hypot(sp[:, 0], sp[:, 1]))) if len(sp) else None

    # ----------------------------------------------------------- visualization
    def _publish_markers(self, face, scan_nearest, pts=None, contour=None):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        if face is None:
            clr = Marker()
            clr.header.frame_id = self.base_frame
            clr.header.stamp = now
            clr.ns = "table_docker"
            clr.action = Marker.DELETEALL
            arr.markers.append(clr)
            self.marker_pub.publish(arr)
            return

        life = Duration(seconds=self.marker_lifetime).to_msg()
        n, c = face["normal"], face["centroid"]
        p1, p2 = face["p1"], face["p2"]

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = now
            m.ns = "table_docker"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.lifetime = life
            m.pose.orientation.w = 1.0
            return m

        # Fitted line (green).
        line = base(0, Marker.LINE_STRIP)
        line.scale.x = 0.03
        line.color.g = 1.0
        line.color.b = 0.2
        line.color.a = 1.0
        line.points = [Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
                       Point(x=float(p2[0]), y=float(p2[1]), z=0.0)]
        arr.markers.append(line)

        # Polygon (cyan) — the fitted face extruded into the surface by polygon_depth.
        poly = base(1, Marker.LINE_STRIP)
        poly.scale.x = 0.02
        poly.color.g = 0.8
        poly.color.b = 1.0
        poly.color.a = 1.0
        q1 = p1 + n * self.polygon_depth
        q2 = p2 + n * self.polygon_depth
        poly.points = [Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
                       Point(x=float(p2[0]), y=float(p2[1]), z=0.0),
                       Point(x=float(q2[0]), y=float(q2[1]), z=0.0),
                       Point(x=float(q1[0]), y=float(q1[1]), z=0.0),
                       Point(x=float(p1[0]), y=float(p1[1]), z=0.0)]
        arr.markers.append(poly)

        # Approach normal (blue) — from the face back toward the robot.
        arrow = base(2, Marker.ARROW)
        arrow.scale.x = 0.02
        arrow.scale.y = 0.05
        arrow.scale.z = 0.07
        arrow.color.b = 1.0
        arrow.color.g = 0.4
        arrow.color.a = 1.0
        arrow.points = [Point(x=float(c[0]), y=float(c[1]), z=0.0),
                        Point(x=float(c[0] - n[0] * 0.3), y=float(c[1] - n[1] * 0.3), z=0.0)]
        arr.markers.append(arrow)

        # All candidate points (dim yellow) — only when provided (preview).
        if pts is not None and len(pts):
            sp = pts[:: len(pts) // 400 + 1] if len(pts) > 400 else pts
            pm = base(3, Marker.POINTS)
            pm.scale.x = pm.scale.y = 0.02
            pm.color.r = 1.0
            pm.color.g = 0.85
            pm.color.a = 0.35
            pm.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in sp]
            arr.markers.append(pm)

        # Front-contour points actually fed to RANSAC (orange).
        if contour is not None and len(contour):
            cm = base(5, Marker.POINTS)
            cm.scale.x = cm.scale.y = 0.03
            cm.color.r = 1.0
            cm.color.g = 0.45
            cm.color.a = 1.0
            cm.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in contour]
            arr.markers.append(cm)

        # Text readout.
        distance = abs(float(n @ c))
        arm_clear = distance - self.front_offset
        e_yaw = reduce_angle_mod_pi(math.atan2(n[1], n[0]))
        near_txt = f"{scan_nearest:.2f}" if scan_nearest is not None else "--"
        txt = base(4, Marker.TEXT_VIEW_FACING)
        txt.scale.z = 0.12
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.position.x = float(c[0])
        txt.pose.position.y = float(c[1])
        txt.pose.position.z = 0.35
        txt.text = f"yaw_err={math.degrees(e_yaw):.0f}deg  clr={arm_clear:.2f}m  near={near_txt}m"
        arr.markers.append(txt)

        self.marker_pub.publish(arr)

    # --------------------------------------------------------- detect + lock
    def _detect_and_lock(self):
        """Collect samples, fit once, lock in odom. Returns the base-frame fit or None."""
        self._stop()
        self._wait_for_samples()
        fit = None
        for _ in range(self.fit_attempts):
            fit = self._fit_face()
            if fit is not None:
                break
            time.sleep(0.1)
        if fit is None:
            return None
        if not self._lock_face(fit):
            return None
        return fit

    # ----------------------------------------------------------------- preview
    def _preview_cb(self, request, response):
        self.log("info", "Preview requested — detecting (no motion)")
        fit = self._detect_and_lock()
        if fit is None:
            self._publish_markers(None, None)
            response.success = False
            response.message = "Could not detect a surface in front of the robot"
            self.log("error", response.message)
            return response
        face = self._face_in_base()
        self._publish_markers(face, self._live_nearest(), pts=fit["pts"], contour=fit["contour"])
        e_yaw = reduce_angle_mod_pi(math.atan2(fit["normal"][1], fit["normal"][0]))
        distance = abs(float(fit["normal"] @ fit["centroid"]))
        response.success = True
        response.message = (f"Detected: yaw_err={math.degrees(e_yaw):.1f}deg "
                            f"dist={distance:.3f}m points={len(fit['pts'])}")
        self.log("info", response.message)
        return response

    # ----------------------------------------------------------------- dock
    def _dock_cb(self, request, response):
        self.log("info", "Dock requested — detecting & locking the surface")
        fit = self._detect_and_lock()
        if fit is None:
            self._publish_markers(None, None)
            response.success = False
            response.message = "Could not detect a surface in front of the robot"
            self.log("error", response.message)
            return response

        e0 = reduce_angle_mod_pi(math.atan2(fit["normal"][1], fit["normal"][0]))
        self.log("info", f"Locked face: yaw_err={math.degrees(e0):.1f}deg "
                         f"dist={abs(float(fit['normal'] @ fit['centroid'])):.3f}m — approaching")

        dt = 1.0 / self.control_rate
        deadline = time.time() + self.approach_timeout
        settled = 0
        tf_fail = 0

        while rclpy.ok():
            if time.time() > deadline:
                self._stop()
                response.success = False
                response.message = "Dock timed out before converging"
                self.log("error", response.message)
                return response

            face = self._face_in_base()
            if face is None:
                tf_fail += 1
                self._stop()
                if tf_fail > self.max_tf_fail:
                    response.success = False
                    response.message = "Lost the locked face transform (TF)"
                    self.log("error", response.message)
                    return response
                time.sleep(dt)
                continue
            tf_fail = 0

            n, c = face["normal"], face["centroid"]
            e_yaw = reduce_angle_mod_pi(math.atan2(n[1], n[0]))   # want robot +x along normal
            y_center = float(c[1])                                 # centre on the face
            distance = abs(float(n @ c))                           # base_link -> locked face (perp)
            arm_clearance = distance - self.front_offset
            e_x = arm_clearance - self.target_distance

            # Live lidar safety override (independent of the locked face).
            scan_near = self._live_nearest()
            safety_stop = scan_near is not None and (scan_near - self.front_offset) <= self.min_safe

            wz = self._clamp(self.k_yaw * e_yaw, self.max_wz)
            vy = self._clamp(self.k_y * y_center, self.max_vy)
            vx = self._clamp(self.k_x * e_x, self.max_vx) if e_x > 0 else 0.0
            if arm_clearance <= self.min_safe or safety_stop:
                vx = 0.0

            close_enough = (abs(e_x) <= self.dist_tol) or (arm_clearance <= self.min_safe + 0.01) or safety_stop
            if abs(e_yaw) <= self.yaw_tol and abs(y_center) <= self.y_tol and close_enough:
                settled += 1
                if settled >= self.settle_cycles:
                    self._stop()
                    self.docked = True
                    self._publish_docked()
                    self._publish_markers(face, scan_near)
                    response.success = True
                    response.message = (f"Docked: arm_clearance={arm_clearance:.3f}m "
                                        f"scan_near={scan_near if scan_near is None else round(scan_near,3)}m "
                                        f"yaw_err={math.degrees(e_yaw):.1f}deg")
                    self.log("info", response.message)
                    return response
            else:
                settled = 0

            self._publish_markers(face, scan_near)
            self._publish_cmd(vx, vy, wz)
            time.sleep(dt)

        self._stop()
        response.success = False
        response.message = "Dock aborted (shutdown)"
        return response

    # --------------------------------------------------------------- undock
    def _undock_cb(self, request, response):
        if not self.docked:
            response.success = True
            response.message = "Not docked — nothing to undock"
            return response

        self.log("info", f"Undock requested — backing off {self.retreat_distance:.2f} m")
        with self._lock:
            start = self._odom
        dt = 1.0 / self.control_rate

        if start is None:
            self.log("warn", "No odometry; timed retreat")
            t_end = time.time() + (self.retreat_distance / max(self.retreat_speed, 1e-3))
            while rclpy.ok() and time.time() < t_end:
                self._publish_cmd(-self.retreat_speed, 0.0, 0.0)
                time.sleep(dt)
        else:
            x0, y0 = start.pose.pose.position.x, start.pose.pose.position.y
            deadline = time.time() + self.retreat_timeout
            while rclpy.ok():
                with self._lock:
                    cur = self._odom
                traveled = math.hypot(cur.pose.pose.position.x - x0,
                                      cur.pose.pose.position.y - y0) if cur else 0.0
                if traveled >= self.retreat_distance or time.time() > deadline:
                    break
                self._publish_cmd(-self.retreat_speed, 0.0, 0.0)
                time.sleep(dt)

        self._stop()
        with self._lock:
            self._face = None
        self.docked = False
        self._publish_docked()
        self._publish_markers(None, None)
        response.success = True
        response.message = f"Retreated {self.retreat_distance:.2f} m"
        self.log("info", response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TableDocker()
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

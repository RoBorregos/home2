#!/usr/bin/env python3
"""Table/shelf docking — perpendicular approach for the holonomic omnibase.

Idea: nav2 brings the robot to a STATIC "near" pose in front of a table/shelf.
From there this node detects the table's front face with the lidar + ZED cloud,
then closed-loop drives the (holonomic) base so it ends up PERPENDICULAR to the
face, centered on it, and as close as it can safely get. Because detection is
done live each time, the table can be moved/rotated and the robot still parks
correctly relative to wherever it actually is.

Flow
----
  1. nav2 -> static near pose (existing go_to_area).
  2. /navigation/dock_to_surface (std_srvs/Trigger):
       detect front face (RANSAC 2D line on /scan + /point_cloud in base_link),
       then control wz (face fronto-parallel), vy (center), vx (approach) until
       target_distance is reached OR the nearest frontal point hits min_safe.
  3. /navigation/undock_from_surface (std_srvs/Trigger):
       drive straight back retreat_distance (odom-measured) so the robot leaves
       nav2's inflated/lethal zone and nav2 can plan the next goal.

nav_central calls undock automatically before every new location goal, so you
never have to remember to back off first.

All behaviour is parameterized — tune target_distance / min_safe / gains / the
detection height band + FOV on the real robot.
"""

import math
import time
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time as RclpyTime

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry

import tf2_ros

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAVE_PC2 = True
except Exception:
    _HAVE_PC2 = False

from frida_constants.navigation_constants import (
    DOCK_SERVICE,
    UNDOCK_SERVICE,
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
    """RANSAC 2D line fit. pts: Nx2. Returns dict or None.

    Returns normal (unit, perpendicular to the line), centroid of inliers,
    direction (unit, along the line) and inlier count.
    """
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
    return {
        "normal": normal,
        "direction": direction,
        "centroid": centroid,
        "count": best_count,
    }


class TableDocker(Node):
    def __init__(self):
        super().__init__("table_docker")

        # --- Topics / frames ---
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.scan_topic = self.declare_parameter("scan_topic", SCAN_TOPIC).value
        self.cloud_topic = self.declare_parameter("cloud_topic", POINT_CLOUD_TOPIC).value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/filtered").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # --- Detection ---
        # 'scan' | 'cloud' | 'both'. Cloud catches the table EDGE/shelf face above
        # the lidar plane; scan is the reliable safety distance + a fallback face.
        self.detect_source = self.declare_parameter("detect_source", "both").value
        self.frontal_fov = math.radians(self.declare_parameter("frontal_fov_deg", 70.0).value)
        self.max_detect_range = self.declare_parameter("max_detect_range", 1.5).value
        self.det_min_height = self.declare_parameter("det_min_height", 0.20).value
        self.det_max_height = self.declare_parameter("det_max_height", 1.20).value
        self.ransac_iters = int(self.declare_parameter("ransac_iters", 200).value)
        self.ransac_thresh = self.declare_parameter("ransac_thresh", 0.03).value
        self.ransac_min_inliers = int(self.declare_parameter("ransac_min_inliers", 12).value)

        # --- Approach targets / safety ---
        self.target_distance = self.declare_parameter("target_distance", 0.45).value   # base_link -> face (m)
        self.min_safe = self.declare_parameter("min_safe", 0.30).value                 # nearest frontal point floor (m)
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
        self.max_detect_fail = int(self.declare_parameter("max_detect_fail", 30).value)

        # --- Retreat (undock) ---
        self.retreat_distance = self.declare_parameter("retreat_distance", 0.5).value
        self.retreat_speed = self.declare_parameter("retreat_speed", 0.12).value
        self.retreat_timeout = self.declare_parameter("retreat_timeout", 20.0).value

        # --- State (guarded by lock) ---
        self._lock = threading.Lock()
        self._scan = None
        self._cloud = None
        self._odom = None
        self.docked = False

        sensor_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10, callback_group=sensor_cb)
        self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_cb, 1, callback_group=sensor_cb)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10, callback_group=sensor_cb)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        docked_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.docked_pub = self.create_publisher(Bool, DOCKED_TOPIC, docked_qos)
        self._publish_docked()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, DOCK_SERVICE, self._dock_cb, callback_group=srv_cb)
        self.create_service(Trigger, UNDOCK_SERVICE, self._undock_cb, callback_group=srv_cb)

        self.log("info", f"table_docker ready. dock={DOCK_SERVICE} undock={UNDOCK_SERVICE} "
                         f"target={self.target_distance}m min_safe={self.min_safe}m source={self.detect_source}")

    # ------------------------------------------------------------------ utils
    def log(self, level, msg):
        getattr(self.get_logger(), level)(f"TableDocker: {msg}")

    def _scan_cb(self, msg):
        with self._lock:
            self._scan = msg

    def _cloud_cb(self, msg):
        with self._lock:
            self._cloud = msg

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

    # -------------------------------------------------------------- detection
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
        x = r * np.cos(a)
        y = r * np.sin(a)
        pts = np.stack([x, y], axis=1)
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
            # Subsample very dense clouds to keep this snappy.
            if len(arr) > 6000:
                arr = arr[:: len(arr) // 6000 + 1]
            if cloud.header.frame_id and cloud.header.frame_id != self.base_frame:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, cloud.header.frame_id, RclpyTime())
                q = tf.transform.rotation
                t = tf.transform.translation
                R = quat_to_rot(q.x, q.y, q.z, q.w)
                arr = arr @ R.T + np.array([t.x, t.y, t.z])
            m = (
                (arr[:, 2] > self.det_min_height)
                & (arr[:, 2] < self.det_max_height)
                & (arr[:, 0] > 0.0)
                & (np.hypot(arr[:, 0], arr[:, 1]) < self.max_detect_range)
                & (np.abs(np.arctan2(arr[:, 1], arr[:, 0])) < self.frontal_fov / 2.0)
            )
            return arr[m][:, :2]
        except Exception as e:
            self.log("warn", f"cloud transform/read failed ({e}); using scan only this cycle")
            return np.empty((0, 2))

    def _detect_face(self):
        """Return (e_yaw, y_center, distance, nearest_frontal) or None."""
        with self._lock:
            scan, cloud = self._scan, self._cloud

        pts_list = []
        if self.detect_source in ("scan", "both"):
            pts_list.append(self._scan_points(scan))
        if self.detect_source in ("cloud", "both"):
            pts_list.append(self._cloud_points(cloud))
        pts = np.vstack([p for p in pts_list if len(p)]) if any(len(p) for p in pts_list) else np.empty((0, 2))
        if len(pts) < self.ransac_min_inliers:
            return None

        fit = ransac_line(pts, self.ransac_iters, self.ransac_thresh, self.ransac_min_inliers)
        if fit is None:
            return None

        normal, centroid = fit["normal"], fit["centroid"]
        # Orient the normal to point from the robot toward the face (centroid x>0).
        if np.dot(normal, centroid) < 0:
            normal = -normal
        # We want the normal aligned with robot +x (robot facing the face square-on).
        e_yaw = reduce_angle_mod_pi(math.atan2(normal[1], normal[0]))
        # Perpendicular distance from base_link origin to the face line.
        distance = abs(float(np.dot(normal, centroid)))
        y_center = float(centroid[1])

        # Nearest frontal point (safety) — always from the scan if available.
        scan_pts = self._scan_points(scan)
        nearest = float(np.min(np.hypot(scan_pts[:, 0], scan_pts[:, 1]))) if len(scan_pts) else distance
        return e_yaw, y_center, distance, nearest

    # ----------------------------------------------------------------- dock
    def _dock_cb(self, request, response):
        self.log("info", "Dock requested — detecting & approaching the surface")
        dt = 1.0 / self.control_rate
        deadline = time.time() + self.approach_timeout
        settled = 0
        fails = 0

        while rclpy.ok():
            if time.time() > deadline:
                self._stop()
                response.success = False
                response.message = "Dock timed out before converging"
                self.log("error", response.message)
                return response

            det = self._detect_face()
            if det is None:
                fails += 1
                self._stop()
                if fails > self.max_detect_fail:
                    response.success = False
                    response.message = "Could not detect a surface in front of the robot"
                    self.log("error", response.message)
                    return response
                time.sleep(dt)
                continue
            fails = 0

            e_yaw, y_center, distance, nearest = det
            e_x = distance - self.target_distance

            # Controls.
            wz = self._clamp(self.k_yaw * e_yaw, self.max_wz)
            vy = self._clamp(self.k_y * y_center, self.max_vy)
            vx = self._clamp(self.k_x * e_x, self.max_vx) if e_x > 0 else 0.0
            # Safety: never push forward past the closest safe distance.
            if nearest <= self.min_safe:
                vx = 0.0

            # Converged? aligned + centered + (at target distance OR can't get closer).
            close_enough = (abs(e_x) <= self.dist_tol) or (nearest <= self.min_safe + 0.01)
            if abs(e_yaw) <= self.yaw_tol and abs(y_center) <= self.y_tol and close_enough:
                settled += 1
                if settled >= self.settle_cycles:
                    self._stop()
                    self.docked = True
                    self._publish_docked()
                    response.success = True
                    response.message = (f"Docked: dist={distance:.3f}m nearest={nearest:.3f}m "
                                        f"yaw_err={math.degrees(e_yaw):.1f}deg")
                    self.log("info", response.message)
                    return response
            else:
                settled = 0

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
        if start is None:
            # No odom: fall back to a timed retreat.
            self.log("warn", "No odometry; timed retreat")
            t_end = time.time() + (self.retreat_distance / max(self.retreat_speed, 1e-3))
            while rclpy.ok() and time.time() < t_end:
                self._publish_cmd(-self.retreat_speed, 0.0, 0.0)
                time.sleep(1.0 / self.control_rate)
            self._stop()
            self.docked = False
            self._publish_docked()
            response.success = True
            response.message = "Retreated (timed)"
            return response

        x0 = start.pose.pose.position.x
        y0 = start.pose.pose.position.y
        deadline = time.time() + self.retreat_timeout
        while rclpy.ok():
            with self._lock:
                cur = self._odom
            traveled = math.hypot(cur.pose.pose.position.x - x0,
                                  cur.pose.pose.position.y - y0) if cur else 0.0
            if traveled >= self.retreat_distance or time.time() > deadline:
                break
            self._publish_cmd(-self.retreat_speed, 0.0, 0.0)
            time.sleep(1.0 / self.control_rate)

        self._stop()
        self.docked = False
        self._publish_docked()
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

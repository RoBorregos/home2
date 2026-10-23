#!/usr/bin/env python3
"""Table/shelf docking — perpendicular approach for the holonomic omnibase.

Idea: nav2 brings the robot to a STATIC "near" pose in front of a table/shelf.
On a service call this node detects the table's front face ONCE from a few
accumulated cloud/scan frames (stable RANSAC line -> polygon -> normal vector),
LOCKS that face in the odom frame, then closed-loop drives the (holonomic) base
so it ends up PERPENDICULAR to the face, centered on it, and as close as the arm
can safely get. Locking the orientation kills the per-frame jitter you get from
re-fitting every tick; the live lidar is still used as a safety stop.

Round tables are supported too: set the `table_shape` param to 'circle' (or 'auto')
and the front face is fit as a CIRCLE instead of a line. The circle is collapsed to
its tangent at the planned point, so the approach reuses the exact same
perpendicular-drive machinery — the robot ends up on the table's radius, facing the
centre, at the planned gap from the rim.

Flow
----
  1. nav2 -> static near pose (existing go_to_area).
  2. /navigation/preview_dock (std_srvs/Trigger): detect + publish RViz markers
     ONLY (no motion) — use this to check the fit/orientation before committing.
  3. /navigation/dock_to_surface (std_srvs/Trigger): detect+lock, PLAN, then approach.
  4. /navigation/undock_from_surface (std_srvs/Trigger): back off retreat_distance
     (odom-measured) so nav2 can plan the next goal. nav_central calls this before
     every new location goal automatically.

Planned approach (nav_main/approach_planner.py)
-----------------------------------------------
No per-location stand-off. The caller names the SURFACE TYPE (`surface_type`
param, profiles in config/approach_profiles.yaml) and optionally the object to
work on (`target_point`). Once the face is locked:
  a. Base placement: candidate base poses along the face (or around the round
     table) inside the type's reach band [gap_min, gap_max] are scored on the
     local costmap with the FULL footprint (a chair next to the table rejects
     the poses it overlaps), lateral offset from the target and travel.
  b. Motion: MPPI (nav2 controller_server, Omni model, footprint-aware) drives a
     planned path to a pre-dock pose `predock_distance` in front of the best pose,
     with the goal checker tightened for the duration.
  c. Final straight-in: the P-controller below closes the last few cm against the
     LOCKED face (perpendicular, centred on the planned point, at `gap`).

Stop rule: the footprint's front edge is kept `gap` from the face, and the live
lidar keeps every point at least `safety_clearance` outside the footprint.

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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from rcl_interfaces.msg import SetParametersResult, Parameter as ParameterMsg, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import TwistStamped, Point, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import FollowPath, ComputePathToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs import do_transform_point

from nav_main import approach_planner as ap

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


def circle_from_3(p1, p2, p3):
    """Exact circle through 3 points. Returns (cx, cy, r) or None if collinear."""
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return ux, uy, math.hypot(ax - ux, ay - uy)


def fit_circle_lsq(pts):
    """Algebraic (Kåsa) least-squares circle refit. pts: Nx2. Returns (cx, cy, r)
    or None."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(pts))])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy, c0 = sol
    r2 = c0 + cx * cx + cy * cy
    if r2 <= 0:
        return None
    return float(cx), float(cy), math.sqrt(r2)


def ransac_circle(pts, iters, thresh, min_inliers, rmin, rmax):
    """RANSAC 2D circle fit from random 3-point samples (radius gated to
    [rmin, rmax]). pts: Nx2. Returns dict (center, radius, mask, count) or None."""
    n = len(pts)
    if n < max(3, min_inliers):
        return None
    best_mask = None
    best_count = 0
    best_center = None
    best_radius = 0.0
    for _ in range(iters):
        i, j, k = np.random.randint(0, n, size=3)
        if i == j or j == k or i == k:
            continue
        c = circle_from_3(pts[i], pts[j], pts[k])
        if c is None:
            continue
        cx, cy, r = c
        if r < rmin or r > rmax:
            continue
        dist = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        mask = dist < thresh
        cnt = int(mask.sum())
        if cnt > best_count:
            best_count, best_mask = cnt, mask
            best_center, best_radius = np.array([cx, cy]), r
    if best_mask is None or best_count < min_inliers:
        return None
    # Least-squares refit on the inliers for a stable centre/radius.
    refit = fit_circle_lsq(pts[best_mask])
    if refit is not None and rmin <= refit[2] <= rmax:
        best_center = np.array([refit[0], refit[1]])
        best_radius = refit[2]
    return {"center": best_center, "radius": float(best_radius),
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

        # --- Surface shape -------------------------------------------------------
        # 'auto' (default — fit both, pick the better inlier support), 'line' (flat
        # face: table edge / shelf, RANSAC line) or 'circle' (round table — RANSAC
        # circle, approached along its radius). The surface profile's `shape` wins;
        # this param only applies when the profile says 'auto'.
        self.table_shape = self.declare_parameter("table_shape", "auto").value
        # Round table is ~0.40 m radius; gate the RANSAC circle to [0.20, 0.90] m so
        # noise/legs/walls can't masquerade as a plausible table.
        self.circle_min_radius = self.declare_parameter("circle_min_radius", 0.20).value
        self.circle_max_radius = self.declare_parameter("circle_max_radius", 0.90).value

        # --- Geometry: the robot footprint (same polygon as nav2's costmaps) ---
        # Gaps are measured from the footprint's front edge; the live safety check
        # uses the whole polygon. Keep in sync with nav2_omni*.yaml `footprint`.
        self.footprint = self._parse_footprint(self.declare_parameter(
            "footprint", "[[0.325, 0.25], [0.325, -0.25], [-0.325, -0.25], [-0.325, 0.25]]").value)

        # --- Per-call request (nav_central sets these before each dock) ---
        # surface_type selects the reach band in approach_profiles.yaml; target_gap
        # >= 0 overrides its preferred gap; target_point [x, y] in target_frame is
        # the object to stand in front of (empty = centre of the detected face).
        self.surface_type = self.declare_parameter("surface_type", "").value
        self.target_gap = self.declare_parameter("target_gap", -1.0).value
        self.target_point = list(self.declare_parameter(
            "target_point", rclpy.Parameter.Type.DOUBLE_ARRAY).value or [])
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.profiles_file = self.declare_parameter("profiles_file", "").value
        self._load_profiles()

        # --- Planned approach (base placement + MPPI) ---
        self.costmap_topic = self.declare_parameter("costmap_topic", "/local_costmap/costmap").value
        self.use_planned_motion = self.declare_parameter("use_planned_motion", True).value
        # Pre-dock pose: this far (m) in front of the planned pose; MPPI drives
        # there, the straight-in controller closes the rest.
        self.predock_distance = self.declare_parameter("predock_distance", 0.20).value
        # Skip MPPI when already this close to the pre-dock pose.
        self.predock_skip = self.declare_parameter("predock_skip", 0.06).value
        self.motion_timeout = self.declare_parameter("motion_timeout", 45.0).value
        self.goal_checker = self.declare_parameter("goal_checker", "general_goal_checker").value
        # MPPI only has to reach the pre-dock neighbourhood; the straight-in
        # controller corrects the rest against the re-detected face.
        self.precise_xy_tol = self.declare_parameter("precise_xy_tol", 0.10).value
        self.precise_yaw_tol = self.declare_parameter("precise_yaw_tol", 0.15).value
        # After each MPPI leg the face is re-detected from the pre-dock pose and the
        # dock re-planned (closer, better view); another leg runs if the plan moved
        # more than replan_shift.
        self.max_motion_legs = int(self.declare_parameter("max_motion_legs", 2).value)
        self.replan_shift = self.declare_parameter("replan_shift", 0.15).value
        # Once docked, re-detect up close and correct if the plan moves > refine_shift.
        self.refine_passes = int(self.declare_parameter("refine_passes", 2).value)
        self.refine_shift = self.declare_parameter("refine_shift", 0.03).value
        # Close-range re-detection only looks at the strip the arm will work on.
        self.refine_fov_deg = self.declare_parameter("refine_fov_deg", 50.0).value
        # Cells within this distance of the docked face belong to the surface itself.
        self.face_ignore_margin = self.declare_parameter("face_ignore_margin", 0.06).value
        # Debug: write each plan's grid/face/candidates to this .npz path ("" = off).
        self.debug_dump = self.declare_parameter("debug_dump", "").value

        # --- Safety: live lidar points must stay this far outside the footprint ---
        self.safety_clearance = self.declare_parameter("safety_clearance", 0.01).value
        self.yaw_tol = self.declare_parameter("yaw_tol", 0.03).value
        self.y_tol = self.declare_parameter("y_tol", 0.03).value
        self.dist_tol = self.declare_parameter("dist_tol", 0.015).value

        # --- Control gains / limits ---
        self.k_yaw = self.declare_parameter("k_yaw", 1.4).value
        self.k_y = self.declare_parameter("k_y", 1.0).value
        self.k_x = self.declare_parameter("k_x", 0.9).value
        self.max_wz = self.declare_parameter("max_wz", 0.8).value
        self.max_vx = self.declare_parameter("max_vx", 0.22).value   # forward = collision dir, keep moderate
        self.max_vy = self.declare_parameter("max_vy", 0.30).value
        # Minimum forward speed while approaching (avoids the proportional crawl in
        # the last few cm). Gated by the safety stop, so it stays safe.
        self.min_vx = self.declare_parameter("min_vx", 0.04).value
        # Same crawl problem on the yaw/strafe axes: commands go RAW to the base
        # (no MPPI behind them to push through stiction), and on the 3-WHEEL limp
        # base small commands stall against the dead corner's drag — the loop then
        # sits just outside yaw_tol/y_tol until approach_timeout. Floors apply only
        # while OUTSIDE tolerance, so they can't cause oscillation inside it.
        self.min_wz = self.declare_parameter("min_wz", 0.06).value
        self.min_vy = self.declare_parameter("min_vy", 0.04).value
        self.control_rate = self.declare_parameter("control_rate", 15.0).value
        self.approach_timeout = self.declare_parameter("approach_timeout", 40.0).value
        self.settle_cycles = int(self.declare_parameter("settle_cycles", 3).value)
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
        self._plan = None       # planned dock (odom frame)
        self._target_odom = None
        self._costmap = None
        # Lethal cells seen since the current request started (odom cell keys).
        # Thin obstacles (chair legs) get raytraced out of nav2's costmap as soon
        # as they fall between rays or into the lidar's near blind zone.
        self._memory = set()
        self._remember = False
        self.docked = False

        sensor_cb = ReentrantCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()
        client_cb = ReentrantCallbackGroup()

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data, callback_group=sensor_cb)
        self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_cb, 1, callback_group=sensor_cb)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10, callback_group=sensor_cb)
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self._costmap_cb, costmap_qos,
                                 callback_group=sensor_cb)
        # nav2 only re-sends the full grid when the window moves; while the robot
        # stands still the marks arrive as patches on <topic>_updates.
        self.create_subscription(OccupancyGridUpdate, self.costmap_topic + "_updates",
                                 self._costmap_update_cb, 10, callback_group=sensor_cb)

        # MPPI (controller_server) + planner for the planned approach.
        self.follow_client = ActionClient(self, FollowPath, "/follow_path", callback_group=client_cb)
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose",
                                        callback_group=client_cb)
        self.ctrl_param_client = self.create_client(SetParameters, "/controller_server/set_parameters",
                                                    callback_group=client_cb)
        self.ctrl_get_client = self.create_client(GetParameters, "/controller_server/get_parameters",
                                                  callback_group=client_cb)
        self.path_pub = self.create_publisher(Path, "/approach_planner/path", 1)
        self.cand_pub = self.create_publisher(MarkerArray, "/approach_planner/candidates", 1)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        docked_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.docked_pub = self.create_publisher(Bool, DOCKED_TOPIC, docked_qos)
        self._publish_docked()
        # RViz: add a MarkerArray display on /table_docker/markers.
        self.marker_pub = self.create_publisher(MarkerArray, "/table_docker/markers", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self._publish_footprint)

        self.create_service(Trigger, DOCK_SERVICE, self._dock_cb, callback_group=srv_cb)
        self.create_service(Trigger, UNDOCK_SERVICE, self._undock_cb, callback_group=srv_cb)
        self.create_service(Trigger, DOCK_PREVIEW_SERVICE, self._preview_cb, callback_group=srv_cb)

        # Apply runtime parameter changes live (nav_central sets the per-call
        # request before approaching; also makes `ros2 param set` effective).
        self.add_on_set_parameters_callback(self._on_set_params)

        self.log("info", f"table_docker ready. dock={DOCK_SERVICE} preview={DOCK_PREVIEW_SERVICE} "
                         f"samples={self.num_samples} front={self.front:.3f}m "
                         f"surfaces={sorted(self.profiles)} source={self.detect_source} "
                         f"planned_motion={self.use_planned_motion}")

    def _on_set_params(self, params):
        """Sync runtime param changes into the cached attributes used by the loop."""
        for p in params:
            if p.name == "frontal_fov_deg":
                self.frontal_fov = math.radians(float(p.value))
            elif p.name == "footprint":
                self.footprint = self._parse_footprint(p.value)
                self._fp_samples_padded = ap.footprint_samples(
                    self.footprint, 0.025, self.footprint_padding)
                self._fp_samples_sweep = ap.footprint_samples(
                    self.footprint, 0.025, self.footprint_padding / 2)
            elif p.name == "target_point":
                self.target_point = list(p.value or [])
            elif p.name == "profiles_file":
                self.profiles_file = p.value
                self._load_profiles()
            elif hasattr(self, p.name):
                setattr(self, p.name, p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------- planning config
    def _parse_footprint(self, text):
        fp = np.asarray(yaml.safe_load(text) if isinstance(text, str) else text, dtype=float)
        self.front, _, self.half_width = ap.footprint_extent(fp)
        self._fp_samples = ap.footprint_samples(fp, 0.025)
        return fp

    def _load_profiles(self):
        path = self.profiles_file or f"{get_package_share_directory('nav_main')}/config/approach_profiles.yaml"
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        self.profiles = ap.load_profiles(data.get("surfaces"))
        self.weights = ap.load_weights(data.get("weights"), "dock")
        self.footprint_padding = float(data.get("footprint_padding", 0.0))
        self._fp_samples_padded = ap.footprint_samples(self.footprint, 0.025, self.footprint_padding)
        # Margins shrink from planning (padding) to sweeps (padding/2) to the live
        # guard (safety_clearance), so a pose that was planned never trips the guard.
        self._fp_samples_sweep = ap.footprint_samples(self.footprint, 0.025, self.footprint_padding / 2)
        if "default" not in self.profiles:
            self.profiles["default"] = ap.ReachProfile(0.03, 0.02, 0.15, "auto")

    def _profile(self):
        """Reach band for this call: surface_type's profile (+ target_gap override)."""
        prof = self.profiles.get(self.surface_type or "default")
        if prof is None:
            self.log("warn", f"unknown surface_type '{self.surface_type}', using 'default'")
            prof = self.profiles["default"]
        prof = ap.ReachProfile(prof.gap, prof.gap_min, prof.gap_max, prof.shape)
        if self.target_gap is not None and self.target_gap >= 0.0:
            # Explicit gap (e.g. a legacy offset): a band centred on it.
            prof.gap = float(self.target_gap)
            prof.gap_min = max(0.01, prof.gap - 0.05)
            prof.gap_max = prof.gap + 0.05
        return prof

    def _reset_request(self):
        """Per-call fields must not leak into the next dock."""
        self._target_odom = None
        self.surface_type = ""
        self.target_gap = -1.0
        self.target_point = []
        self.target_frame = "map"

    def _costmap_cb(self, msg):
        """Full local costmap: keep it as an array + its geometry."""
        info = msg.info
        with self._lock:
            self._costmap = {
                "data": np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width),
                "info": info, "frame": msg.header.frame_id.lstrip("/"),
                "stamp": RclpyTime.from_msg(msg.header.stamp),
            }
            self._remember_lethal(self._costmap)

    def _remember_lethal(self, cm):
        """Add the costmap's lethal cells to the request's obstacle memory (lock held)."""
        if not self._remember:
            return
        info = cm["info"]
        rows, cols = np.nonzero(cm["data"] >= 100)
        res = info.resolution
        kx = np.floor((info.origin.position.x + (cols + 0.5) * res) / res).astype(int)
        ky = np.floor((info.origin.position.y + (rows + 0.5) * res) / res).astype(int)
        self._memory.update(zip(kx.tolist(), ky.tolist()))

    def _costmap_update_cb(self, msg):
        """Patch the stored costmap with an OccupancyGridUpdate."""
        with self._lock:
            cm = self._costmap
            if cm is None:
                return
            h, w = cm["data"].shape
            if msg.x + msg.width > w or msg.y + msg.height > h:
                return
            cm["data"][msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = np.asarray(
                msg.data, dtype=np.int16).reshape(msg.height, msg.width)
            cm["stamp"] = RclpyTime.from_msg(msg.header.stamp)
            self._remember_lethal(cm)

    def _wait_costmap(self, after, timeout=3.0):
        """Block until the costmap has been updated after `after` (rclpy Time), e.g.
        right after nav2 resumed and reset its layers. Returns True if it was."""
        end = self._now() + timeout
        while rclpy.ok() and self._now() < end:
            with self._lock:
                cm = self._costmap
            if cm is not None and cm["stamp"] > after:
                return True
            time.sleep(0.05)
        return False

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

    def _now(self):
        """Node clock in seconds: sim time in simulation, wall time on the robot.
        Motion deadlines use it so a slow simulation does not time them out."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_docked(self):
        self.docked_pub.publish(Bool(data=self.docked))

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        # Nav2 1.4.0 uses TwistStamped on cmd_vel (odrive_dashboard subscribes stamped).
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = float(wz)
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
        end = self._now() + self.collect_timeout
        while rclpy.ok() and self._now() < end:
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

    def _fit_face(self, shape):
        """One stable fit from the accumulated samples. Returns base_link geometry
        dict {normal, centroid, direction, p1, p2, pts, contour, nearest, count}
        (a flat 'face' usable by the approach loop regardless of shape) or None."""
        pts = self._accumulate_points()
        if len(pts) < self.ransac_min_inliers:
            return None
        # Fit the front edge (contour), not the filled surface.
        fit_pts = self._front_contour(pts, self.contour_bin_deg) if self.use_front_contour else pts
        if len(fit_pts) < self.ransac_min_inliers:
            fit_pts = pts
        shape = str(shape).lower()
        if shape in ("circle", "circular", "round"):
            return self._fit_circle_face(fit_pts, pts)
        if shape == "auto":
            line = self._fit_line_face(fit_pts, pts)
            circ = self._fit_circle_face(fit_pts, pts)
            if line is None:
                return circ
            if circ is None:
                return line
            # Prefer the shape with stronger inlier support; bias toward the line so
            # a flat table isn't mistaken for a large-radius circle on a tie.
            if circ["count"] <= line["count"] * 1.15:
                return line
            # Two walls meeting at a corner also look "round": if a second line on
            # what the first one leaves out explains as much as the circle, it is
            # piecewise flat (a corner), not a round table.
            rest = fit_pts[~line["mask"]]
            line2 = ransac_line(rest, self.ransac_iters, self.ransac_thresh, self.ransac_min_inliers)
            if line2 is not None and line["count"] + line2["count"] >= circ["count"]:
                return line
            # A circle only wins if its support is actually CURVED: an uneven flat
            # front (steps, handles) can feed RANSAC a circle with more inliers.
            sup = circ["contour"][self._circle_mask(circ)]
            if len(sup) >= 3:
                c = sup.mean(axis=0)
                _, sv, _ = np.linalg.svd(sup - c, full_matrices=False)
                if sv[-1] / math.sqrt(len(sup)) < self.ransac_thresh:
                    return line
            return circ
        return self._fit_line_face(fit_pts, pts)

    def _circle_mask(self, circ):
        d = np.abs(np.hypot(circ["contour"][:, 0] - circ["center"][0],
                            circ["contour"][:, 1] - circ["center"][1]) - circ["radius"])
        return d < self.ransac_thresh

    def _fit_line_face(self, fit_pts, pts):
        """Flat face (table edge / shelf): RANSAC line -> perpendicular approach."""
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
                "p1": p1, "p2": p2, "pts": pts, "contour": fit_pts, "nearest": nearest,
                "count": fit["count"], "mask": fit["mask"]}

    def _fit_circle_face(self, fit_pts, pts):
        """Round table: RANSAC circle -> approach along the radius. The circle is
        reduced to the TANGENT line at the point nearest the robot, so the rest of
        the pipeline (lock + perpendicular approach + centring) is identical to the
        flat-face case: driving perpendicular to that tangent and centring on the
        tangent point puts the robot on the radial line, facing the table centre."""
        fit = ransac_circle(fit_pts, self.ransac_iters, self.ransac_thresh,
                            self.ransac_min_inliers, self.circle_min_radius,
                            self.circle_max_radius)
        if fit is None:
            return None
        center, radius = fit["center"], fit["radius"]
        d = float(np.hypot(center[0], center[1]))     # robot -> circle centre
        if d < 1e-6 or d <= radius:                   # robot must be outside the table
            return None
        normal = center / d                           # toward the centre == toward the face
        centroid = center - normal * radius           # nearest point on the circle
        direction = np.array([-normal[1], normal[0]])  # tangent at that point
        half = float(min(radius, 0.30))               # short tangent segment (viz)
        p1 = centroid - direction * half
        p2 = centroid + direction * half
        nearest = float(np.min(np.hypot(pts[:, 0], pts[:, 1])))
        return {"normal": normal, "centroid": centroid, "direction": direction,
                "p1": p1, "p2": p2, "pts": pts, "contour": fit_pts, "nearest": nearest,
                "count": fit["count"], "center": center, "radius": radius}

    def _lock_face(self, fit):
        """Store the fitted face in the odom frame so it stays world-fixed while
        the robot drives (no per-cycle re-fit -> no orientation jitter)."""
        Rt = self._lookup_Rt(self.odom_frame, self.base_frame)
        if Rt is None:
            return False
        R, t = Rt
        face = {
            "normal": self._xform_vec(R, fit["normal"]),
            "centroid": self._xform_pt(R, t, fit["centroid"]),
            "p1": self._xform_pt(R, t, fit["p1"]),
            "p2": self._xform_pt(R, t, fit["p2"]),
        }
        if "center" in fit:
            face["center"] = self._xform_pt(R, t, fit["center"])
            face["radius"] = fit["radius"]
        with self._lock:
            self._face = face
            self._plan = None
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

    def _live_clearance(self, radius=1.2, with_point=False):
        """Distance (m) from the footprint to the nearest live lidar point (all
        around, not just ahead: the approach strafes and turns). The 3rd smallest
        value is used so a single noisy ray cannot trigger the stop. With
        with_point, also returns that point (base_link) or None."""
        with self._lock:
            scan = self._scan
        none = (None, None) if with_point else None
        if scan is None:
            return none
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > max(scan.range_min, 1e-3)) & (r < radius)
        if not ok.any():
            return none
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        d = ap.footprint_clearance(pts, self.footprint)
        k = min(2, len(d) - 1)
        i = int(np.argpartition(d, k)[k])
        return (float(d[i]), pts[i]) if with_point else float(d[i])

    def _live_front_clearance(self):
        """Gap (m) from the footprint's front edge to the nearest live lidar point
        straight ahead of it: the surface's real nearest point, which can stick out
        in front of the fitted line (uneven fronts, handles, steps)."""
        with self._lock:
            scan = self._scan
        if scan is None:
            return None
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > max(scan.range_min, 1e-3)) & (r < self.front + 0.5)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        ahead = (x > self.front - 0.02) & (np.abs(y) <= self.half_width)
        if ahead.sum() < 3:
            return None
        return float(np.partition(x[ahead], 2)[2] - self.front)

    # ---------------------------------------------------------------- planning
    def _robot_pose(self, frame):
        Rt = self._lookup_Rt(frame, self.base_frame)
        if Rt is None:
            return None
        R, t = Rt
        return float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])

    def _target_in(self, frame):
        """The per-call target_point in `frame`, or None."""
        if len(self.target_point) < 2:
            return None
        ps = PointStamped()
        ps.header.frame_id = self.target_frame or "map"
        ps.point.x, ps.point.y = float(self.target_point[0]), float(self.target_point[1])
        if ps.header.frame_id == frame:
            return np.array([ps.point.x, ps.point.y])
        try:
            tf = self.tf_buffer.lookup_transform(frame, ps.header.frame_id, RclpyTime(),
                                                 timeout=Duration(seconds=0.5))
        except Exception as e:
            self.log("warn", f"target TF {ps.header.frame_id}->{frame} failed ({e}); ignoring target")
            return None
        p = do_transform_point(ps, tf).point
        return np.array([p.x, p.y])

    def _fresh_grid(self, max_age=3.0):
        """Local costmap as an ap.Grid in the odom frame, or None if stale/missing."""
        with self._lock:
            cm = self._costmap
            data = None if cm is None else cm["data"].copy()
        if cm is None:
            return None
        if cm["frame"] != self.odom_frame:
            self.log("warn", f"costmap frame '{cm['frame']}' != {self.odom_frame}; no collision check")
            return None
        age = (self.get_clock().now() - cm["stamp"]).nanoseconds * 1e-9
        if age > max_age:
            self.log("warn", f"costmap is {age:.1f}s old (nav2 paused?); planning without it")
            return None
        info = cm["info"]
        with self._lock:
            memory = list(self._memory)
        if memory:
            res = info.resolution
            k = np.asarray(memory)
            cols = np.floor(((k[:, 0] + 0.5) * res - info.origin.position.x) / res).astype(int)
            rows = np.floor(((k[:, 1] + 0.5) * res - info.origin.position.y) / res).astype(int)
            inside = (cols >= 0) & (cols < info.width) & (rows >= 0) & (rows < info.height)
            data[rows[inside], cols[inside]] = 100
        return ap.Grid(ap.occupancy_to_cost(data.ravel()), info.width, info.height, info.resolution,
                       info.origin.position.x, info.origin.position.y)

    def _plan_dock(self, profile):
        """Base placement on the locked face (odom frame). Returns the plan dict
        (best pose + face point + pre-dock pose) or None."""
        with self._lock:
            face = self._face
        robot = self._robot_pose(self.odom_frame)
        if face is None or robot is None:
            return None
        target = self._target_odom
        grid = self._fresh_grid()
        if "center" in face:
            center, radius = face["center"], face["radius"]
            ref = math.atan2(robot[1] - center[1], robot[0] - center[0])
            cands = ap.circle_face_candidates(center, radius, self.front, profile, ref, target,
                                              angle_step_deg=5.0, max_angle_deg=120.0)
            ignore = ap.disc_ignore(center, radius, self.face_ignore_margin)
        else:
            n = face["normal"]
            # No object given: stand straight ahead of where nav2 left the robot (the
            # staging pose an operator placed in front of this furniture), not at the
            # centre of whatever stretch of face the lidar happened to see.
            aim = target if target is not None else np.array(robot[:2])
            cands = ap.line_face_candidates(face["p1"], face["p2"], n, self.front, self.half_width,
                                            profile, aim)
            ignore = ap.halfplane_ignore(face["centroid"], n, self.face_ignore_margin)
        # Padded footprint: keep a margin from everything but the surface itself.
        valid, all_c = ap.rank(cands, profile, self.weights, grid, self._fp_samples_padded,
                               robot[:2], ignore)
        if grid is not None:
            # The approach corridor matters too: the pre-dock pose in front of each
            # candidate must be free as well.
            for c in valid:
                px = c.x - math.cos(c.yaw) * self.predock_distance
                py = c.y - math.sin(c.yaw) * self.predock_distance
                if ap.footprint_cost(grid, px, py, c.yaw, self._fp_samples_padded, ignore) >= ap.LETHAL:
                    c.valid, c.reason = False, "pre-dock blocked"
            valid = [c for c in valid if c.valid]
        self._publish_candidates(all_c, valid[0] if valid else None)
        if self.debug_dump:
            self._dump_n = getattr(self, "_dump_n", 0) + 1
            np.savez(f"{self.debug_dump}_{self._dump_n:03d}.npz", cells=grid.cells if grid else np.zeros((0, 0)),
                     geo=np.array([grid.resolution, grid.origin_x, grid.origin_y]) if grid else np.zeros(3),
                     face=np.array([face.get("center", face["centroid"]).tolist() + [face.get("radius", 0.0)]
                                    + list(face["normal"]) + list(face["p1"]) + list(face["p2"])]),
                     cands=np.array([[c.x, c.y, c.yaw, c.gap, c.cost, c.valid, c.score] for c in all_c]),
                     robot=np.array(robot), footprint=self.footprint)
        if not valid:
            return None
        best = valid[0]
        nvec = np.array([math.cos(best.yaw), math.sin(best.yaw)])
        pose = np.array([best.x, best.y])
        pre = pose - nvec * self.predock_distance
        plan = {"pose": (best.x, best.y, best.yaw), "pre": (float(pre[0]), float(pre[1]), best.yaw),
                "q": pose + nvec * (self.front + best.gap), "normal": nvec, "gap": best.gap,
                "cost": best.cost, "lateral": best.lateral, "target": target,
                "n_valid": len(valid), "n_total": len(all_c), "checked": grid is not None}
        with self._lock:
            self._plan = plan
        self._publish_plan_markers(plan)
        return plan

    def _same_surface(self, old, max_offset=0.10, max_deeper=0.35):
        """Whether the freshly locked face is the surface we are docking at.

        * A face more than max_offset CLOSER than the locked one is something in
          front of it (a chair before the table): rejected.
        * A face up to max_deeper FARTHER is accepted only if the lidar confirms the
          strip straight ahead is free up to it (the back of a cabinet niche, seen
          once the side panels are out of view).
        """
        with self._lock:
            new = self._face
        if old is None or new is None:
            return True
        if ("center" in old) != ("center" in new):
            return False
        if "center" in old:
            return float(np.linalg.norm(new["center"] - old["center"])) <= max_offset
        n = old["normal"] / (np.linalg.norm(old["normal"]) + 1e-12)
        offset = float((new["centroid"] - old["centroid"]) @ n)  # > 0: farther
        if -max_offset <= offset <= max_offset:
            return True
        if offset < 0 or offset > max_deeper:
            return False
        fb = self._face_in_base()
        live = self._live_front_clearance()
        if fb is None or live is None:
            return False
        return live >= float(fb["normal"] @ fb["centroid"]) - self.front - 0.05

    def _fuse_face_orientation(self, old):
        """Up close the lidar sees only a short piece of a flat face, so its
        orientation is noisy while its distance is precise. Keep the orientation of
        whichever fit spanned the longer segment; keep the new fit's position."""
        with self._lock:
            new = self._face
        if old is None or new is None or "center" in new or "center" in old:
            return
        len_old = old.get("span", float(np.linalg.norm(old["p2"] - old["p1"])))
        len_new = float(np.linalg.norm(new["p2"] - new["p1"]))
        if len_new >= len_old:
            return
        n = old["normal"] / (np.linalg.norm(old["normal"]) + 1e-12)
        if n @ new["normal"] < 0:
            n = -n
        d = np.array([-n[1], n[0]])
        c = new["centroid"]
        t1, t2 = float((new["p1"] - c) @ d), float((new["p2"] - c) @ d)
        fused = dict(new, normal=n, p1=c + d * t1, p2=c + d * t2, span=len_old)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ (new["normal"] / (np.linalg.norm(new["normal"]) + 1e-12)))))))
        if ang > 0.5:
            self.log("info", f"Kept the orientation of the longer face fit ({len_old:.2f}m vs "
                             f"{len_new:.2f}m, {ang:.1f}deg apart)")
        with self._lock:
            self._face = fused

    def _replan(self, profile, plan, narrow=False):
        """Re-detect the face from where the robot is now and plan again. Keeps the
        previous plan (and its locked face) if either step fails. With `narrow`,
        only the strip straight ahead is used (close-range refinement). Returns
        (plan, shift in m between the old and the new dock pose)."""
        with self._lock:
            old_face = self._face
        fov = self.frontal_fov
        if narrow:
            self.frontal_fov = math.radians(self.refine_fov_deg)
        try:
            fit = self._detect_and_lock(profile.shape)
        finally:
            self.frontal_fov = fov
        if fit is not None and not self._same_surface(old_face):
            # Data association: from closer up something else (a chair in front of
            # the table) can become the nearest "face". Not the same surface -> ignore.
            self.log("warn", "Re-detection found a different surface; keeping the locked face")
            fit = None
        if fit is not None:
            self._fuse_face_orientation(old_face)
        new = self._plan_dock(profile) if fit is not None else None
        if new is None:
            with self._lock:
                self._face, self._plan = old_face, plan
            self.log("warn", "Re-plan at pre-dock failed; keeping the first plan")
            return plan, 0.0
        shift = math.hypot(new["pose"][0] - plan["pose"][0], new["pose"][1] - plan["pose"][1])
        self.log("info", f"Re-planned at pre-dock: moved {shift:.2f}m, gap={new['gap']:.2f}m "
                         f"lateral_err={new['lateral']:.2f}m")
        return new, shift

    def _plan_in_base(self):
        """Planned face point + approach normal in the CURRENT base_link frame."""
        with self._lock:
            plan = self._plan
        if plan is None:
            return None
        Rt = self._lookup_Rt(self.base_frame, self.odom_frame)
        if Rt is None:
            return None
        R, t = Rt
        n = self._xform_vec(R, plan["normal"])
        return {"normal": n / (np.linalg.norm(n) + 1e-12), "q": self._xform_pt(R, t, plan["q"])}

    # ------------------------------------------------------- planned motion
    def _wait(self, future, timeout):
        end = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < end:
            time.sleep(0.02)
        return future.done()

    def _goal_checker_params(self, xy=None, yaw=None):
        """Read (no args) or set the controller's goal checker tolerances."""
        names = [f"{self.goal_checker}.xy_goal_tolerance", f"{self.goal_checker}.yaw_goal_tolerance"]
        if xy is None:
            if not self.ctrl_get_client.wait_for_service(timeout_sec=1.0):
                return None
            fut = self.ctrl_get_client.call_async(GetParameters.Request(names=names))
            if not self._wait(fut, 2.0) or fut.result() is None or len(fut.result().values) != 2:
                return None
            return tuple(v.double_value for v in fut.result().values)
        if not self.ctrl_param_client.wait_for_service(timeout_sec=1.0):
            return False
        req = SetParameters.Request()
        for name, val in zip(names, (xy, yaw)):
            req.parameters.append(ParameterMsg(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))))
        fut = self.ctrl_param_client.call_async(req)
        return self._wait(fut, 2.0) and fut.result() is not None and all(
            r.successful for r in fut.result().results)

    def _pose_msg(self, frame, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return p

    def _face_ignore(self):
        with self._lock:
            face = self._face
        if face is None:
            return None
        if "center" in face:
            return ap.disc_ignore(face["center"], face["radius"], self.face_ignore_margin)
        return ap.halfplane_ignore(face["centroid"], face["normal"], self.face_ignore_margin)

    def _path_free(self, path, grid, ignore):
        """Every pose of a nav_msgs/Path (any frame) keeps the footprint off lethal
        cells of `grid` (odom), the remembered obstacles included."""
        frame = (path.header.frame_id or self.odom_frame).lstrip("/")
        Rt = None if frame == self.odom_frame else self._lookup_Rt(self.odom_frame, frame)
        for p in path.poses[::2]:
            q = p.pose.orientation
            xy, yaw = (p.pose.position.x, p.pose.position.y), math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if Rt is not None:
                xy = self._xform_pt(Rt[0], Rt[1], xy)
                yaw += math.atan2(Rt[0][1, 0], Rt[0][0, 0])
            if ap.footprint_cost(grid, xy[0], xy[1], yaw, self._fp_samples_sweep, ignore) >= ap.LETHAL:
                return False
        return True

    def _straight_path_free(self, start, goal, grid, ignore):
        """Whether the footprint swept along start->goal (yaw interpolated) stays
        clear of lethal cells."""
        if grid is None:
            return False
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        dyaw = (goal[2] - start[2] + math.pi) % (2 * math.pi) - math.pi
        steps = max(2, int(max(dist / 0.05, abs(dyaw) / 0.1)) + 1)
        for s in np.linspace(0.0, 1.0, steps):
            x = start[0] + s * (goal[0] - start[0])
            y = start[1] + s * (goal[1] - start[1])
            if ap.footprint_cost(grid, x, y, start[2] + s * dyaw, self._fp_samples_sweep, ignore) >= ap.LETHAL:
                return False
        return True

    def _approach_path(self, plan):
        """Path (odom) from the robot to the pre-dock pose: the straight line when the
        swept footprint is collision-free, otherwise nav2's planner (global costmap)."""
        start = self._robot_pose(self.odom_frame)
        goal = plan["pre"]
        with self._lock:
            face = self._face
        if "center" in face:
            ignore = ap.disc_ignore(face["center"], face["radius"], self.face_ignore_margin)
        else:
            ignore = ap.halfplane_ignore(face["centroid"], face["normal"], self.face_ignore_margin)
        if self._straight_path_free(start, goal, self._fresh_grid(), ignore):
            path = Path()
            path.header.frame_id = self.odom_frame
            path.header.stamp = self.get_clock().now().to_msg()
            dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
            dyaw = (goal[2] - start[2] + math.pi) % (2 * math.pi) - math.pi
            for s in np.linspace(0.0, 1.0, max(2, int(dist / 0.05) + 1)):
                path.poses.append(self._pose_msg(self.odom_frame,
                                                 start[0] + s * (goal[0] - start[0]),
                                                 start[1] + s * (goal[1] - start[1]),
                                                 start[2] + s * dyaw))
            return path, "straight"
        # Blocked: plan around it on the local grid, which includes the obstacles
        # remembered during this request (nav2's own planner may have lost them).
        grid = self._fresh_grid()
        if grid is not None:
            pts = ap.footprint_astar(grid, start[:2], goal[:2], goal[2], self._fp_samples_sweep, ignore)
            if pts is not None:
                path = Path()
                path.header.frame_id = self.odom_frame
                path.header.stamp = self.get_clock().now().to_msg()
                dyaw = (goal[2] - start[2] + math.pi) % (2 * math.pi) - math.pi
                turn = max(1, int(len(pts) * 0.3))  # settle the heading early on
                for i, (x, y) in enumerate(pts):
                    path.poses.append(self._pose_msg(self.odom_frame, x, y,
                                                     start[2] + dyaw * min(1.0, i / turn)))
                return path, "local A*"
        # Still nothing: let the global planner route around (goal given in odom;
        # the planner transforms it into the map frame).
        if not self.plan_client.wait_for_server(timeout_sec=2.0):
            return None, "planner unavailable"
        g = ComputePathToPose.Goal()
        g.goal = self._pose_msg(self.odom_frame, *goal)
        g.planner_id = "GridBased"
        send = self.plan_client.send_goal_async(g)
        if not self._wait(send, 5.0) or not send.result().accepted:
            return None, "planner rejected"
        res = send.result().get_result_async()
        if not self._wait(res, 10.0) or not res.result().result.path.poses:
            return None, "no path"
        path = res.result().result.path
        grid = self._fresh_grid()
        # NavFn leaves intermediate yaws at 0: hold the start heading, then the goal's.
        for i, p in enumerate(path.poses):
            yaw = goal[2] if i >= len(path.poses) - 3 else start[2]
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        path.poses[-1] = self._pose_msg(path.header.frame_id or "map", *self._to_frame(goal, path.header.frame_id))
        # nav2's costmaps lose thin obstacles (chair legs) between rays: check the
        # path against what this request has seen.
        if grid is not None and not self._path_free(path, grid, ignore):
            return None, "planner path crosses a remembered obstacle"
        return path, "planner"

    def _to_frame(self, pose_odom, frame):
        frame = (frame or "map").lstrip("/")
        if frame == self.odom_frame:
            return pose_odom
        Rt = self._lookup_Rt(frame, self.odom_frame)
        if Rt is None:
            return pose_odom
        R, t = Rt
        p = self._xform_pt(R, t, pose_odom[:2])
        return float(p[0]), float(p[1]), pose_odom[2] + math.atan2(R[1, 0], R[0, 0])

    def _drive_to_predock(self, plan, xy_tol=None, yaw_tol=None):
        """MPPI to the pre-dock pose. Returns (ok, message)."""
        xy_tol = self.precise_xy_tol if xy_tol is None else xy_tol
        yaw_tol = self.precise_yaw_tol if yaw_tol is None else yaw_tol
        start = self._robot_pose(self.odom_frame)
        pre = plan["pre"]
        dyaw = abs((pre[2] - start[2] + math.pi) % (2 * math.pi) - math.pi)
        if math.hypot(pre[0] - start[0], pre[1] - start[1]) < min(self.predock_skip, xy_tol) \
                and dyaw < min(0.1, yaw_tol):
            return True, "already at pre-dock"
        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            return False, "controller_server not available"
        path, how = self._approach_path(plan)
        if path is None:
            return False, how
        self.path_pub.publish(path)
        saved = self._goal_checker_params()
        if saved is None or not self._goal_checker_params(xy_tol, yaw_tol):
            return False, "could not tighten the goal checker"
        try:
            goal = FollowPath.Goal()
            goal.path = path
            goal.controller_id = "FollowPath"
            send = self.follow_client.send_goal_async(goal)
            if not self._wait(send, 5.0) or not send.result().accepted:
                return False, "FollowPath rejected (nav2 paused?)"
            handle = send.result()
            res = handle.get_result_async()
            end = self._now() + self.motion_timeout
            ignore = self._face_ignore()
            guard = ap.footprint_samples(self.footprint, 0.025, self.safety_clearance)
            tick = 0
            while rclpy.ok() and not res.done():
                clr = self._live_clearance()
                if self._now() > end or (clr is not None and clr <= self.safety_clearance):
                    handle.cancel_goal_async()
                    self._stop()
                    return False, ("timeout" if self._now() > end else f"safety stop (clearance {clr:.3f} m)")
                tick += 1
                if tick % 4 == 0:
                    # Remembered obstacles the lidar can no longer see (blind zone).
                    grid, pose = self._fresh_grid(), self._robot_pose(self.odom_frame)
                    if grid is not None and pose is not None and ap.footprint_cost(
                            grid, pose[0], pose[1], pose[2], guard, ignore) >= ap.LETHAL:
                        handle.cancel_goal_async()
                        self._stop()
                        return False, "remembered obstacle at the footprint"
                time.sleep(0.05)
            status = res.result().status
            return status == GoalStatus.STATUS_SUCCEEDED, f"MPPI via {how} path (status {status})"
        finally:
            self._goal_checker_params(*saved)

    def _publish_candidates(self, cands, best):
        arr = MarkerArray()
        clr = Marker()
        clr.header.frame_id = self.odom_frame
        clr.ns = "candidates"
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)
        now = self.get_clock().now().to_msg()
        valid = [c for c in cands if c.valid]
        lo = min((c.score for c in valid), default=0.0)
        hi = max((c.score for c in valid), default=1.0)
        pts = Marker()
        pts.header.frame_id = self.odom_frame
        pts.header.stamp = now
        pts.ns = "candidates"
        pts.id = 1
        pts.type = Marker.SPHERE_LIST
        pts.pose.orientation.w = 1.0
        pts.scale.x = pts.scale.y = pts.scale.z = 0.035
        for c in cands:
            pts.points.append(Point(x=c.x, y=c.y, z=0.05))
            if c.valid:
                s = (c.score - lo) / (hi - lo + 1e-9)
                pts.colors.append(ColorRGBA(r=float(s), g=float(1.0 - s), b=0.1, a=0.9))
            else:
                pts.colors.append(ColorRGBA(r=0.25, g=0.25, b=0.25, a=0.6))
        arr.markers.append(pts)
        self.cand_pub.publish(arr)

    def _publish_plan_markers(self, plan):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def mk(mid, mtype):
            m = Marker()
            m.header.frame_id = self.odom_frame
            m.header.stamp = now
            m.ns = "plan"
            m.id = mid
            m.type = mtype
            m.pose.orientation.w = 1.0
            return m

        # Planned footprint (green outline) at the chosen dock pose.
        x, y, yaw = plan["pose"]
        fp = mk(10, Marker.LINE_STRIP)
        fp.scale.x = 0.025
        fp.color.g, fp.color.a = 1.0, 1.0
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py in list(self.footprint) + [self.footprint[0]]:
            fp.points.append(Point(x=x + c * px - s * py, y=y + s * px + c * py, z=0.02))
        arr.markers.append(fp)
        for mid, (px, py, pyaw), rgb in ((11, plan["pose"], (0.0, 1.0, 0.0)),
                                         (12, plan["pre"], (1.0, 0.0, 1.0))):
            a = mk(mid, Marker.ARROW)
            a.scale.x, a.scale.y, a.scale.z = 0.02, 0.05, 0.07
            a.color.r, a.color.g, a.color.b, a.color.a = (*rgb, 1.0)
            a.points = [Point(x=px, y=py, z=0.05),
                        Point(x=px + 0.3 * math.cos(pyaw), y=py + 0.3 * math.sin(pyaw), z=0.05)]
            arr.markers.append(a)
        if plan["target"] is not None:
            t = mk(13, Marker.CYLINDER)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = (
                float(plan["target"][0]), float(plan["target"][1]), 0.1)
            t.scale.x = t.scale.y = 0.08
            t.scale.z = 0.2
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.6, 0.0, 1.0
            arr.markers.append(t)
        txt = mk(14, Marker.TEXT_VIEW_FACING)
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, 0.4
        txt.scale.z = 0.09
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = (f"gap={plan['gap']:.2f}m  {plan['n_valid']}/{plan['n_total']} free"
                    + ("" if plan["checked"] else " (no costmap)"))
        arr.markers.append(txt)
        self.cand_pub.publish(arr)

    def _publish_footprint(self):
        """Robot footprint locked to base_link (nav2's is stale while it is paused)."""
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns = "robot_footprint"
        m.type = Marker.LINE_STRIP
        m.frame_locked = True
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.a = 1.0, 1.0, 1.0
        for px, py in list(self.footprint) + [self.footprint[0]]:
            m.points.append(Point(x=float(px), y=float(py), z=0.03))
        self.marker_pub.publish(MarkerArray(markers=[m]))

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
        arm_clear = distance - self.front
        e_yaw = reduce_angle_mod_pi(math.atan2(n[1], n[0]))
        txt = base(4, Marker.TEXT_VIEW_FACING)
        txt.scale.z = 0.12
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.position.x = float(c[0])
        txt.pose.position.y = float(c[1])
        txt.pose.position.z = 0.35
        txt.text = f"yaw_err={math.degrees(e_yaw):.0f}deg  clr={arm_clear:.2f}m"
        arr.markers.append(txt)

        self.marker_pub.publish(arr)

    # --------------------------------------------------------- detect + lock
    def _detect_and_lock(self, shape="auto"):
        """Collect samples, fit once, lock in odom. Returns the base-frame fit or None.
        `shape` comes from the surface profile; 'auto' defers to the table_shape param."""
        self._stop()
        self._wait_for_samples()
        fit = None
        for _ in range(self.fit_attempts):
            fit = self._fit_face(self.table_shape if shape == "auto" else shape)
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
        self.log("info", "Preview requested — detecting + planning (no motion)")
        profile = self._profile()
        # Fixed in odom once: a target given in base_link/camera must not move with the robot.
        self._target_odom = self._target_in(self.odom_frame)
        try:
            fit = self._detect_and_lock(profile.shape)
            if fit is None:
                self._publish_markers(None, None)
                response.success = False
                response.message = "Could not detect a surface in front of the robot"
                self.log("error", response.message)
                return response
            face = self._face_in_base()
            self._publish_markers(face, self._live_nearest(), pts=fit["pts"], contour=fit["contour"])
            plan = self._plan_dock(profile)
            e_yaw = reduce_angle_mod_pi(math.atan2(fit["normal"][1], fit["normal"][0]))
            distance = abs(float(fit["normal"] @ fit["centroid"]))
            shape_txt = f"circle r={fit['radius']:.3f}m " if "radius" in fit else "line "
            response.success = plan is not None
            response.message = (f"Detected ({shape_txt.strip()}): yaw_err={math.degrees(e_yaw):.1f}deg "
                                f"dist={distance:.3f}m points={len(fit['pts'])}; "
                                + (f"plan gap={plan['gap']:.2f}m lateral={plan['lateral']:.2f}m "
                                   f"({plan['n_valid']}/{plan['n_total']} free)" if plan
                                   else "no collision-free dock pose"))
            self.log("info", response.message)
            return response
        finally:
            self._reset_request()

    # ----------------------------------------------------------------- dock
    def _dock_cb(self, request, response):
        profile = self._profile()
        self.log("info", f"Dock requested — surface={self.surface_type or 'default'} "
                         f"gap={profile.gap:.2f}m band=[{profile.gap_min:.2f}, {profile.gap_max:.2f}] "
                         f"target={self.target_point or '-'} [{self.target_frame}]")
        # Fixed in odom once: a target given in base_link/camera must not move with the robot.
        self._target_odom = self._target_in(self.odom_frame)
        with self._lock:
            self._memory = set()
            self._remember = True
        try:
            return self._dock(profile, response)
        finally:
            with self._lock:
                self._remember = False
                self._memory = set()
            self._reset_request()

    def _dock(self, profile, response):
        start = self.get_clock().now()
        fit = self._detect_and_lock(profile.shape)
        if fit is not None and profile.shape == "auto":
            # Re-detections must not flip between a line and a circle.
            profile.shape = "circle" if "radius" in fit else "line"
        # nav2 was just resumed: let the costmap integrate fresh scans (it is reset
        # on activation) so thin obstacles such as chair legs are in it.
        if not self._wait_costmap(start + Duration(seconds=0.5)):
            self.log("warn", "No fresh local costmap; planning without collision check")
        if fit is None:
            self._publish_markers(None, None)
            response.success = False
            response.message = "Could not detect a surface in front of the robot"
            self.log("error", response.message)
            return response

        e0 = reduce_angle_mod_pi(math.atan2(fit["normal"][1], fit["normal"][0]))
        shape_txt = f"circle r={fit['radius']:.3f}m" if "radius" in fit else "line"
        self.log("info", f"Locked face ({shape_txt}): yaw_err={math.degrees(e0):.1f}deg "
                         f"dist={abs(float(fit['normal'] @ fit['centroid'])):.3f}m")
        self._publish_markers(self._face_in_base(), None, contour=fit["contour"])

        # (a) base placement
        plan = self._plan_dock(profile)
        if plan is None:
            self._stop()
            response.success = False
            response.message = "No collision-free dock pose inside the reach band"
            self.log("error", response.message)
            return response
        self.log("info", f"Planned dock: gap={plan['gap']:.2f}m lateral_err={plan['lateral']:.2f}m "
                         f"cost={plan['cost']} ({plan['n_valid']}/{plan['n_total']} candidates free"
                         + ("" if plan["checked"] else ", no costmap") + ")")

        # (b) MPPI to the pre-dock pose, re-detecting + re-planning on arrival
        if self.use_planned_motion:
            for leg in range(1, self.max_motion_legs + 1):
                ok, msg = self._drive_to_predock(plan)
                self.log("info" if ok else "warn", f"Approach motion (leg {leg}): {msg}"
                         + ("" if ok else " — finishing with the docking controller"))
                if not ok:
                    break
                plan, shift = self._replan(profile, plan)
                if shift <= self.replan_shift:
                    break

        # (c) straight-in against the locked face, then (d) re-detect from up close
        # and correct: the face fit is far more accurate at the docked distance.
        # The servo moves in a straight line, so it only runs when that sweep is
        # clear (e.g. never cuts across a round table after MPPI stopped short).
        safe, why = self._servo_path_safe(plan)
        if not safe and self.use_planned_motion:
            # MPPI stopped inside its loose tolerance where the straight sweep grazes
            # something (tight spot): one more, tighter MPPI leg before giving up.
            self.log("warn", f"Straight-in blocked ({why}); tighter MPPI leg to the pre-dock")
            ok, msg = self._drive_to_predock(plan, self.precise_xy_tol / 2, self.precise_yaw_tol / 2)
            self.log("info" if ok else "warn", f"Approach motion (tight): {msg}")
            safe, why = self._servo_path_safe(plan)
        if not safe:
            self._stop()
            response.success = False
            response.message = f"No safe straight-in to the planned dock pose ({why})"
            self.log("error", response.message)
            return response
        ok, msg = self._servo(plan)
        for _ in range(self.refine_passes if ok else 0):
            # Flat faces: look only at the strip ahead (a round table needs its arc).
            new, shift = self._replan(profile, plan, narrow=profile.shape == "line")
            if shift <= self.refine_shift:
                break
            if shift > self.replan_shift + 0.10:
                # A big jump is a different spot, not a correction: no blind moves.
                self.log("warn", f"Close-range re-plan moved {shift:.2f}m; keeping the docked pose")
                with self._lock:
                    self._plan = plan
                break
            plan = new
            ok, msg = self._servo(plan)
        self._stop()
        if ok:
            self.docked = True
            self._publish_docked()
        response.success = ok
        response.message = msg
        self.log("info" if ok else "error", msg)
        return response

    def _servo_path_safe(self, plan, blind_limit=0.30):
        """Whether the straight sweep from here to the planned pose is collision-free
        on the local costmap (surface cells ignored). Without a costmap only a short
        move (blind_limit) is allowed."""
        start = self._robot_pose(self.odom_frame)
        if start is None:
            return False, "no robot pose"
        goal = plan["pose"]
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        grid = self._fresh_grid()
        if grid is None:
            return dist <= blind_limit, f"no costmap and {dist:.2f}m away"
        with self._lock:
            face = self._face
        if "center" in face:
            ignore = ap.disc_ignore(face["center"], face["radius"], self.face_ignore_margin)
            # Never sweep THROUGH a round table: every intermediate pose must stay outside it.
            for s_ in np.linspace(0.0, 1.0, max(2, int(dist / 0.05) + 1)):
                x = start[0] + s_ * (goal[0] - start[0])
                y = start[1] + s_ * (goal[1] - start[1])
                if math.hypot(x - face["center"][0], y - face["center"][1]) < face["radius"] + self.front:
                    return False, "straight line crosses the round table"
        else:
            ignore = ap.halfplane_ignore(face["centroid"], face["normal"], self.face_ignore_margin)
        if self._straight_path_free(start, goal, grid, ignore):
            return True, ""
        return False, "swept footprint hits an obstacle"

    def _servo(self, plan):
        """Holonomic P-controller to the planned pose against the locked face:
        perpendicular, the planned face point straight ahead, front `gap` from it.
        Returns (ok, message)."""
        gap = plan["gap"]
        dt = 1.0 / self.control_rate
        deadline = self._now() + self.approach_timeout
        settled = 0
        tf_fail = 0
        tick = 0

        while rclpy.ok():
            tick += 1
            if tick % 10 == 0:
                self._publish_markers(self._face_in_base(), None)
            if self._now() > deadline:
                self._stop()
                return False, "Dock timed out before converging"

            pb = self._plan_in_base()
            if pb is None:
                tf_fail += 1
                self._stop()
                if tf_fail > self.max_tf_fail:
                    return False, "Lost the locked face transform (TF)"
                time.sleep(dt)
                continue
            tf_fail = 0

            n, q = pb["normal"], pb["q"]
            e_yaw = reduce_angle_mod_pi(math.atan2(n[1], n[0]))   # want robot +x along normal
            y_err = float(q[1])                                    # planned point straight ahead
            clearance = float(n @ q) - self.front                  # robot front -> face
            front = self._live_front_clearance()
            if front is not None and abs(e_yaw) < 0.1:
                clearance = min(clearance, front)                  # never closer than seen
            e_x = clearance - gap

            # Live lidar safety (whole footprint, independent of the locked face).
            live, near = self._live_clearance(with_point=True)
            safety_stop = live is not None and live <= self.safety_clearance

            wz = self._clamp(self.k_yaw * e_yaw, self.max_wz)
            if abs(e_yaw) > self.yaw_tol and 0.0 < abs(wz) < self.min_wz:
                wz = math.copysign(self.min_wz, wz)
            vy = self._clamp(self.k_y * y_err, self.max_vy)
            if abs(y_err) > self.y_tol and 0.0 < abs(vy) < self.min_vy:
                vy = math.copysign(self.min_vy, vy)
            if e_x > 0:
                vx = self._clamp(self.k_x * e_x, self.max_vx)
                # Keep at least min_vx until within tolerance (no final-cm crawl).
                if e_x > self.dist_tol and 0.0 < vx < self.min_vx:
                    vx = self.min_vx
            elif e_x < -self.dist_tol:
                # Too close (re-plan moved the face, or planned further back): back off.
                vx = max(self.k_x * e_x, -self.min_vx * 2)
            else:
                vx = 0.0
            if safety_stop:
                # Drop the velocity component heading into the nearest point.
                u = near / (np.linalg.norm(near) + 1e-9)
                into = vx * u[0] + vy * u[1]
                if into > 0.0:
                    vx -= into * u[0]
                    vy -= into * u[1]

            # A safety stop only ends the approach when the blocking point is AHEAD;
            # brushing past something at the side must not count as "arrived".
            blocked_ahead = safety_stop and near is not None and near[0] > self.front - 0.02
            close_enough = abs(e_x) <= self.dist_tol or (blocked_ahead and e_x > 0)
            if abs(e_yaw) <= self.yaw_tol and abs(y_err) <= self.y_tol and close_enough:
                settled += 1
                if settled >= self.settle_cycles:
                    self._stop()
                    self._publish_markers(self._face_in_base(), None)
                    return True, (f"Docked: gap={clearance:.3f}m (planned {gap:.2f}) "
                                  f"lateral_err={plan['lateral']:.2f}m "
                                  f"live_clearance={live if live is None else round(live, 3)}m "
                                  f"yaw_err={math.degrees(e_yaw):.1f}deg")
            else:
                settled = 0

            self._publish_cmd(vx, vy, wz)
            time.sleep(dt)

        self._stop()
        return False, "Dock aborted (shutdown)"

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
            t_end = self._now() + (self.retreat_distance / max(self.retreat_speed, 1e-3))
            while rclpy.ok() and self._now() < t_end:
                self._publish_cmd(-self.retreat_speed, 0.0, 0.0)
                time.sleep(dt)
        else:
            x0, y0 = start.pose.pose.position.x, start.pose.pose.position.y
            deadline = self._now() + self.retreat_timeout
            while rclpy.ok():
                with self._lock:
                    cur = self._odom
                traveled = math.hypot(cur.pose.pose.position.x - x0,
                                      cur.pose.pose.position.y - y0) if cur else 0.0
                if traveled >= self.retreat_distance or self._now() > deadline:
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

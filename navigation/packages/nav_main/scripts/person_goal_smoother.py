#!/usr/bin/env python3

"""
Person Goal Smoother — Bridges tracker output to Nav2 GoalUpdater.

Transforms the tracked person's 3D position from camera frame to map frame,
applies EMA smoothing to remove jitter, leads the goal along the person's
velocity, keeps the goal out of the costmap's inscribed/lethal cells (the person
is a lidar obstacle), and publishes PoseStamped goals for the Nav2 GoalUpdater BT
node — at a steady heartbeat, so a re-sent NavigateToPose goal always finds a
goal newer than its seed (the Jazzy GoalUpdater ignores older ones).

When the tracker goes silent it runs a recovery state machine and reports it on
/navigation/follow_status (std_msgs/String, latched):

    following -> lost      no tracker data for `timeout` s: drive to the person's
                           last-known position, facing their direction of travel
    lost -> searching      at that position (or after lost_goto_timeout): turn in
                           place through the search_yaws sweep
    searching -> gave_up   search_timeout elapsed: hold still, keep listening
    any -> following       tracker data resumes (ReID re-lock)
    any -> idle            follow mode switched off

Pure geometry lives in nav_main/follow_planner.py (unit-tested offline).

Usage:
    ros2 run nav_main person_goal_smoother.py
    # nav_central's /navigation/follow_person switches this node to follow mode
    # and sends NavigateToPose with bt/follow_dynamic_point.xml.
"""

import math
import os

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from frida_constants.vision_constants import RESULTS_TOPIC
from frida_constants.navigation_constants import (
    MAP_TOPIC, GLOBAL_COSTMAP_TOPIC, GOAL_UPDATE_TOPIC, FOLLOW_MODE_SERVICE,
    FOLLOW_STATUS_TOPIC, FOLLOW_STATUS_IDLE, FOLLOW_STATUS_FOLLOWING,
    FOLLOW_STATUS_LOST, FOLLOW_STATUS_SEARCHING, FOLLOW_STATUS_GAVE_UP,
)
from nav_main import approach_planner as ap
from nav_main import follow_planner as fp


# Which MPPI controller params to switch (dot-separated keys under controller_server.ros__parameters)
CONTROLLER_KEYS = [
    "FollowPath.vx_max",
    "FollowPath.vx_min",
    "FollowPath.vy_max",                       # omnibase only — lateral limit
    "FollowPath.wz_max",
    "FollowPath.GoalCritic.cost_weight",
    "FollowPath.GoalAngleCritic.cost_weight",
    # Raised in follow mode so the base keeps facing the person while it drives
    # (the goal yaw points at them); standard only aligns yaw within 0.5 m.
    "FollowPath.GoalAngleCritic.threshold_to_consider",
    "FollowPath.PathFollowCritic.cost_weight",
    "FollowPath.PathAlignCritic.cost_weight",
    "FollowPath.PathAngleCritic.cost_weight",
    "FollowPath.PreferForwardCritic.cost_weight",  # dashgo only
    "FollowPath.CostCritic.cost_weight",
    "general_goal_checker.xy_goal_tolerance",
    "general_goal_checker.yaw_goal_tolerance",
]

# Inflation params to switch (under local_costmap.local_costmap.ros__parameters)
COSTMAP_KEYS = [
    "inflation_layer.inflation_radius",
    "inflation_layer.cost_scaling_factor",
]

# Velocity smoother params (under velocity_smoother.ros__parameters)
SMOOTHER_KEYS = [
    "max_velocity",
    "min_velocity",
    "max_accel",
    "max_decel",
]

# Costmap layers to toggle in follow mode (depth sees person as obstacle)
# Only layers present in BOTH configs' plugins list will succeed — missing ones are silently skipped
LAYERS_TO_DISABLE_IN_FOLLOW = [
    "rgbd_obstacle_layer",
]

# Costmap layers to always keep enabled (lidar-based, doesn't see person)
LAYERS_ALWAYS_ENABLED = [
    "obstacle_layer",
]


def _resolve_yaml(yaml_data, dotted_key):
    """Resolve a dot-separated key in a nested dict. Returns None if not found."""
    node = yaml_data
    for k in dotted_key.split("."):
        if isinstance(node, dict) and k in node:
            node = node[k]
        else:
            return None
    return node


def _load_nav_yaml(filename):
    """Load a nav2 YAML config file. Relative names resolve inside the
    nav_main config dir; absolute paths (as passed by the launch file when it
    forwards the active nav2 config) are used as-is."""
    if os.path.isabs(filename):
        path = filename
    else:
        path = os.path.join(get_package_share_directory("nav_main"), "config", filename)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _make_param(name, value):
    """Create a rcl_interfaces Parameter msg from a Python value."""
    p = Parameter()
    p.name = name
    if isinstance(value, bool):
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, int):
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    elif isinstance(value, float):
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    elif isinstance(value, str):
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    elif isinstance(value, list):
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in value],
        )
    return p


class PersonGoalSmoother(Node):
    def __init__(self):
        super().__init__("person_goal_smoother")
        cb = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter("alpha", 0.5)            # EMA smoothing (0=full smooth, 1=no smooth)
        self.declare_parameter("distance_gate", 0.08)   # Min person movement (m) to recompute the goal
        self.declare_parameter("follow_distance", 0.8)  # Stay this far (m) behind the person
        # Tracker publishes at a solid 10-20 Hz since the async-ReID rework, so a
        # 2 s silence really means the person is lost.
        self.declare_parameter("timeout", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("snap_radius", 20)       # Grid cells for the spiral fallback
        self.declare_parameter("snap_max_back", 1.0)    # Max m to pull a blocked goal toward the robot
        # Aim the goal ahead of the person by lead_time * estimated velocity —
        # cuts the lag on corners (the classic loss scenario). 0.0 disables.
        self.declare_parameter("lead_time", 0.6)
        self.declare_parameter("max_lead", 1.0)          # m cap on the lead
        self.declare_parameter("max_person_speed", 2.0)  # m/s clamp on the velocity estimate
        self.declare_parameter("heartbeat_rate", 2.0)    # Hz re-publish of the current goal
        # Lost-person recovery (see module docstring)
        self.declare_parameter("lost_goto_timeout", 8.0)
        self.declare_parameter("lost_arrive_radius", 0.5)
        self.declare_parameter("search_dwell", 2.5)
        self.declare_parameter("search_step_deg", 60.0)
        self.declare_parameter("search_timeout", 15.0)
        # "last_position" (default) runs the recovery above; "halt" freezes at the
        # robot's own pose on loss (legacy behavior) and reports gave_up at once.
        self.declare_parameter("lost_behavior", "last_position")
        # Selects the (standard, follow) nav2 config PAIR to switch between.
        self.declare_parameter("default_base", "omnibase")
        # Explicit config-pair override. The launch file forwards the ACTUAL nav2
        # config it started Nav2 with (e.g. nav2_omni_limp.yaml) so leaving follow
        # mode restores THOSE values, not the healthy-base nav2_omni.yaml speeds.
        self.declare_parameter("standard_config_file", "")
        self.declare_parameter("follow_config_file", "")

        # --- State ---
        self.smooth_x = None
        self.smooth_y = None
        self.last_goal_x = None        # person position the current goal was computed from
        self.last_goal_y = None
        self.current_goal = None       # (x, y, yaw) last published, re-sent by the heartbeat
        self.last_tracker_time = None  # rclpy Time of the last accepted tracker point
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.map_grid = None           # ap.Grid from /map (fallback)
        self.costmap_msg = None        # latest global costmap OccupancyGrid
        self.costmap_data = None       # its data as int8 numpy (patched in place)
        self.costmap_grid = None
        self.costmap_dirty = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self._prev_smooth = None       # (x, y, Time)
        self.status = FOLLOW_STATUS_IDLE
        self.state_since = None
        self.search_yaws = []
        self.search_index = -1
        self.search_next_time = None
        self.lost_goal = None

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(PointStamped, RESULTS_TOPIC, self._tracker_cb, 10, callback_group=cb)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._map_cb, 1, callback_group=cb)
        # Global costmap (static map + inflation + live obstacles incl. the person).
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, GLOBAL_COSTMAP_TOPIC, self._costmap_cb, costmap_qos, callback_group=cb)
        self.create_subscription(
            OccupancyGridUpdate, GLOBAL_COSTMAP_TOPIC + "_updates", self._costmap_update_cb, 10,
            callback_group=cb)

        # --- Publishers ---
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_UPDATE_TOPIC, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, FOLLOW_STATUS_TOPIC, status_qos)

        # --- Load the (standard, follow) YAML pair for the active base ---
        base = self.get_parameter("default_base").value
        standard_file = self.get_parameter("standard_config_file").value or \
            os.path.join("omni_config", "nav2_omni.yaml")
        follow_file = self.get_parameter("follow_config_file").value or \
            os.path.join("omni_config", "nav2_omni_following.yaml")
        try:
            self.follow_yaml = _load_nav_yaml(follow_file)
            self.standard_yaml = _load_nav_yaml(standard_file)
            self.get_logger().info(
                f"[base={base}] Loaded follow='{follow_file}', standard='{standard_file}'")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML configs: {e}")
            self.follow_yaml = {}
            self.standard_yaml = {}

        # --- Nav2 param switching clients ---
        self.param_clients = {
            "controller_server": self.create_client(
                SetParameters, "/controller_server/set_parameters", callback_group=cb),
            "local_costmap": self.create_client(
                SetParameters, "/local_costmap/local_costmap/set_parameters", callback_group=cb),
            "global_costmap": self.create_client(
                SetParameters, "/global_costmap/global_costmap/set_parameters", callback_group=cb),
            "velocity_smoother": self.create_client(
                SetParameters, "/velocity_smoother/set_parameters", callback_group=cb),
        }
        self.follow_mode_active = False

        self.create_service(SetBool, FOLLOW_MODE_SERVICE, self._follow_mode_cb, callback_group=cb)

        # --- Timers ---
        self.create_timer(0.1, self._update_robot_pose, callback_group=cb)
        self.create_timer(0.2, self._watchdog, callback_group=cb)
        self.create_timer(
            1.0 / self.get_parameter("heartbeat_rate").value, self._heartbeat, callback_group=cb)
        self._set_status(FOLLOW_STATUS_IDLE)
        self.get_logger().info("Person Goal Smoother ready")

    # ── Nav2 param switching ──────────────────────────────────

    def _follow_mode_cb(self, request, response):
        """Service to switch between follow and standard nav params."""
        self._reset_person()
        if request.data:
            self._apply_nav_params(follow=True)
            self.follow_mode_active = True
            # Don't let a stale timestamp trip the watchdog before the tracker warms up.
            self.last_tracker_time = self.get_clock().now()
            self._set_status(FOLLOW_STATUS_FOLLOWING)
            response.message = "Follow mode activated"
        else:
            self._apply_nav_params(follow=False)
            self.follow_mode_active = False
            self.last_tracker_time = None
            self._set_status(FOLLOW_STATUS_IDLE)
            response.message = "Standard mode restored"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _reset_person(self):
        self.smooth_x = None
        self.smooth_y = None
        self.last_goal_x = None
        self.last_goal_y = None
        self.current_goal = None
        self.vel_x = 0.0
        self.vel_y = 0.0
        self._prev_smooth = None
        self.lost_goal = None
        self.search_yaws = []
        self.search_index = -1

    def _apply_nav_params(self, follow: bool):
        """Set Nav2 params for follow or standard mode, loaded from YAMLs."""
        src = self.follow_yaml if follow else self.standard_yaml
        fallback = self.standard_yaml if follow else self.follow_yaml
        self._set_params("controller_server", self._extract_params(
            src, fallback, "controller_server.ros__parameters", CONTROLLER_KEYS))
        for name, section in (("local_costmap", "local_costmap.local_costmap.ros__parameters"),
                              ("global_costmap", "global_costmap.global_costmap.ros__parameters")):
            params = self._extract_params(src, fallback, section, COSTMAP_KEYS)
            for layer in LAYERS_TO_DISABLE_IN_FOLLOW:
                params.append(_make_param(f"{layer}.enabled", not follow))
            for layer in LAYERS_ALWAYS_ENABLED:
                params.append(_make_param(f"{layer}.enabled", True))
            self._set_params(name, params)
        self._set_params("velocity_smoother", self._extract_params(
            src, fallback, "velocity_smoother.ros__parameters", SMOOTHER_KEYS))

    def _extract_params(self, primary, fallback, section, keys):
        params = []
        for key in keys:
            val = _resolve_yaml(primary, f"{section}.{key}")
            if val is None:
                val = _resolve_yaml(fallback, f"{section}.{key}")
            if val is not None:
                params.append(_make_param(key, val))
        return params

    def _set_params(self, node_name, params):
        if not params:
            return
        client = self.param_clients.get(node_name)
        if client is None or not client.service_is_ready():
            self.get_logger().warn(f"SetParameters not ready for {node_name}")
            return
        req = SetParameters.Request()
        req.parameters = params
        client.call_async(req).add_done_callback(
            lambda f, n=node_name: self._param_result_cb(f, n))

    def _param_result_cb(self, future, node_name):
        try:
            failed = [r for r in future.result().results if not r.successful]
            if failed:
                self.get_logger().warn(f"Some params failed on {node_name}: {[r.reason for r in failed]}")
            else:
                self.get_logger().info(f"Params set on {node_name}")
        except Exception as e:
            self.get_logger().error(f"SetParameters failed on {node_name}: {e}")

    # ── Status ─────────────────────────────────────────────────

    def _set_status(self, status):
        if status != self.status:
            self.get_logger().info(f"Follow status: {self.status} -> {status}")
        self.status = status
        self.state_since = self.get_clock().now()
        self.status_pub.publish(String(data=status))

    def _elapsed(self, since):
        if since is None:
            return 0.0
        return (self.get_clock().now() - since).nanoseconds / 1e9

    # ── Callbacks ──────────────────────────────────────────────

    def _tracker_cb(self, msg: PointStamped):
        """Receive tracker point, transform, smooth, gate, publish."""
        if not self.follow_mode_active:
            return
        map_frame = self.get_parameter("map_frame").value
        # Transform at the point's own stamp: the camera rides on the arm, which
        # pans while following. Fall back to latest if TF lacks that stamp.
        point_map, err = None, None
        for stamp in (rclpy.time.Time.from_msg(msg.header.stamp), rclpy.time.Time()):
            try:
                transform = self.tf_buffer.lookup_transform(
                    map_frame, msg.header.frame_id, stamp,
                    timeout=rclpy.duration.Duration(seconds=0.1))
                point_map = do_transform_point(msg, transform)
                break
            except Exception as e:
                err = e
        if point_map is None:
            self.get_logger().warn(f"TF failed: {err}", throttle_duration_sec=2.0)
            return

        px, py = point_map.point.x, point_map.point.y
        if not math.isfinite(px) or not math.isfinite(py):
            return

        now = self.get_clock().now()
        self.last_tracker_time = now
        if self.status != FOLLOW_STATUS_FOLLOWING:
            self.get_logger().info("Tracker data resumed — following again")
            self.lost_goal = None
            self.search_yaws = []
            self._set_status(FOLLOW_STATUS_FOLLOWING)

        alpha = self.get_parameter("alpha").value
        if self.smooth_x is None:
            self.smooth_x, self.smooth_y = px, py
        else:
            self.smooth_x = alpha * px + (1.0 - alpha) * self.smooth_x
            self.smooth_y = alpha * py + (1.0 - alpha) * self.smooth_y
        self._update_velocity(now)

        # Distance gate: only recompute when the person moved enough (the
        # heartbeat keeps re-sending the current goal regardless).
        if self.last_goal_x is not None and math.hypot(
                self.smooth_x - self.last_goal_x, self.smooth_y - self.last_goal_y) \
                < self.get_parameter("distance_gate").value:
            return
        self._publish_follow_goal()

    def _update_velocity(self, now):
        if self._prev_smooth is not None:
            phx, phy, pht = self._prev_smooth
            dt = (now - pht).nanoseconds / 1e9
            self.vel_x, self.vel_y = fp.update_velocity(
                self.vel_x, self.vel_y, (phx, phy), (self.smooth_x, self.smooth_y), dt,
                self.get_parameter("max_person_speed").value)
        self._prev_smooth = (self.smooth_x, self.smooth_y, now)

    def _map_cb(self, msg: OccupancyGrid):
        self.get_logger().info("Occupancy grid received", once=True)
        self.map_grid = ap.Grid.from_occupancy_grid(msg)

    def _costmap_cb(self, msg: OccupancyGrid):
        self.get_logger().info("Global costmap received", once=True)
        self.costmap_msg = msg
        self.costmap_data = np.asarray(msg.data, dtype=np.int8).copy()
        self.costmap_dirty = True

    def _costmap_update_cb(self, msg: OccupancyGridUpdate):
        """Apply an OccupancyGridUpdate patch to the stored global costmap."""
        if self.costmap_msg is None:
            return
        w, h = self.costmap_msg.info.width, self.costmap_msg.info.height
        if msg.x + msg.width > w or msg.y + msg.height > h:
            return
        patch = np.asarray(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        self.costmap_data.reshape(h, w)[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch
        self.costmap_dirty = True

    def _grid(self):
        """Global costmap (knows the person and the inflation) if available, else /map."""
        if self.costmap_msg is not None:
            if self.costmap_dirty or self.costmap_grid is None:
                info = self.costmap_msg.info
                self.costmap_grid = ap.Grid(
                    ap.occupancy_to_cost(self.costmap_data), info.width, info.height,
                    info.resolution, info.origin.position.x, info.origin.position.y)
                self.costmap_dirty = False
            return self.costmap_grid
        return self.map_grid

    def _update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value, "base_link",
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            pass

    # ── Lost-person recovery ───────────────────────────────────

    def _watchdog(self):
        if not self.follow_mode_active or self.last_tracker_time is None:
            return
        if self.status == FOLLOW_STATUS_FOLLOWING:
            silent = self._elapsed(self.last_tracker_time)
            if silent >= self.get_parameter("timeout").value:
                self._enter_lost(silent)
        elif self.status == FOLLOW_STATUS_LOST:
            gx, gy, _ = self.lost_goal
            arrived = math.hypot(gx - self.robot_x, gy - self.robot_y) \
                <= self.get_parameter("lost_arrive_radius").value
            if arrived or self._elapsed(self.state_since) >= self.get_parameter("lost_goto_timeout").value:
                self._enter_searching()
        elif self.status == FOLLOW_STATUS_SEARCHING:
            if self._elapsed(self.state_since) >= self.get_parameter("search_timeout").value:
                self._enter_gave_up()
            elif self.get_clock().now() >= self.search_next_time:
                self._next_search_yaw()

    def _enter_lost(self, silent):
        if self.get_parameter("lost_behavior").value != "last_position" or self.smooth_x is None:
            self.get_logger().warn(
                f"No tracker data for {silent:.1f}s — halting until the person is re-acquired")
            yaw = self.robot_yaw
            if self.smooth_x is not None:
                yaw = math.atan2(self.smooth_y - self.robot_y, self.smooth_x - self.robot_x)
            self.lost_goal = (self.robot_x, self.robot_y, yaw)
            self._publish_goal(*self.lost_goal)
            self._set_status(FOLLOW_STATUS_GAVE_UP)
            return
        gx, gy, yaw = fp.lost_goal(
            self.robot_x, self.robot_y, self.smooth_x, self.smooth_y,
            self.vel_x, self.vel_y, self.get_parameter("follow_distance").value)
        gx, gy = self._snap(gx, gy)
        self.lost_goal = (gx, gy, yaw)
        self.get_logger().warn(
            f"No tracker data for {silent:.1f}s — going to last-known person position "
            f"({gx:.2f}, {gy:.2f}) facing {math.degrees(yaw):.0f}° to re-acquire")
        self._publish_goal(gx, gy, yaw)
        self._set_status(FOLLOW_STATUS_LOST)

    def _enter_searching(self):
        gx, gy, travel_yaw = self.lost_goal
        self.search_yaws = fp.search_yaws(travel_yaw, self.get_parameter("search_step_deg").value)
        self.search_index = -1
        self._set_status(FOLLOW_STATUS_SEARCHING)
        self.get_logger().warn(
            f"Person not re-acquired at ({gx:.2f}, {gy:.2f}) — sweeping "
            f"{[round(math.degrees(y)) for y in self.search_yaws]}°")
        self._next_search_yaw()

    def _next_search_yaw(self):
        gx, gy, _ = self.lost_goal
        self.search_index = (self.search_index + 1) % len(self.search_yaws)
        self.search_next_time = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.get_parameter("search_dwell").value)
        self._publish_goal(gx, gy, self.search_yaws[self.search_index])

    def _enter_gave_up(self):
        self.get_logger().error(
            "Search timed out — holding position until the tracker re-locks "
            "(task manager re-locks on follow_status gave_up)")
        self.lost_goal = (self.robot_x, self.robot_y, self.robot_yaw)
        self._publish_goal(*self.lost_goal)
        self._set_status(FOLLOW_STATUS_GAVE_UP)

    # ── Goal publishing ────────────────────────────────────────

    def _publish_follow_goal(self):
        target_x, target_y = fp.lead_target(
            self.smooth_x, self.smooth_y, self.vel_x, self.vel_y,
            self.get_parameter("lead_time").value, self.get_parameter("max_lead").value)
        goal = fp.follow_goal(self.robot_x, self.robot_y, target_x, target_y,
                              self.get_parameter("follow_distance").value)
        if goal is None:
            return  # on top of the person, no meaningful direction
        goal_x, goal_y = self._snap(goal[0], goal[1])
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)  # face the (led) person
        self._publish_goal(goal_x, goal_y, yaw)
        self.last_goal_x, self.last_goal_y = self.smooth_x, self.smooth_y
        self.get_logger().info(
            f"Goal: ({goal_x:.2f}, {goal_y:.2f}) yaw={math.degrees(yaw):.0f}° "
            f"person=({self.smooth_x:.2f}, {self.smooth_y:.2f}) v=({self.vel_x:.2f}, {self.vel_y:.2f}) "
            f"dist={math.hypot(target_x - self.robot_x, target_y - self.robot_y):.2f}m",
            throttle_duration_sec=1.0)

    def _snap(self, gx, gy):
        """Keep the goal out of inscribed/lethal cost: pull it toward the robot
        along the line first (stays on our side of the person), then ring search."""
        grid = self._grid()
        x, y, moved = fp.snap_along_line(grid, gx, gy, self.robot_x, self.robot_y,
                                         max_back=self.get_parameter("snap_max_back").value)
        if moved > 0.0:
            return x, y
        cost = fp.cell_cost(grid, x, y)
        if cost is not None and cost >= fp.MAX_GOAL_COST:
            return fp.snap_spiral(grid, x, y, self.get_parameter("snap_radius").value)
        return x, y

    def _publish_goal(self, x, y, yaw):
        self.current_goal = (x, y, yaw)
        self._send_goal_msg()

    def _send_goal_msg(self):
        if self.current_goal is None:
            return
        x, y, yaw = self.current_goal
        msg = PoseStamped()
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def _heartbeat(self):
        """Re-send the current goal with a fresh stamp. The GoalUpdater only
        takes goals newer than the NavigateToPose seed, and nav_central re-sends
        that goal whenever the BT aborts."""
        if self.follow_mode_active and self.current_goal is not None:
            self._send_goal_msg()
        self.status_pub.publish(String(data=self.status))


def main(args=None):
    rclpy.init(args=args)
    node = PersonGoalSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

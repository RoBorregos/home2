#!/usr/bin/env python3

"""
Person Goal Smoother — Bridges tracker output to Nav2 GoalUpdater.

Transforms the tracked person's 3D position from camera frame to map frame,
applies EMA smoothing to remove jitter, gates updates by distance threshold,
snaps goals to free space on the occupancy grid, and publishes stable
PoseStamped goals for the Nav2 GoalUpdater BT node.

Usage:
    ros2 run nav_main person_goal_smoother.py

    # Then use NavigateToPose action (the BT uses follow_dynamic_point.xml
    # with GoalUpdater subscribing to /goal_update)
"""

import math
import os
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


TRACKER_TOPIC = "/vision/tracking_results"
GOAL_UPDATE_TOPIC = "/goal_update"
MAP_TOPIC = "/map"
FOLLOW_MODE_SERVICE = "/navigation/set_follow_mode"

# Which MPPI controller params to switch (dot-separated keys under controller_server.ros__parameters)
CONTROLLER_KEYS = [
    "FollowPath.vx_max",
    "FollowPath.vx_min",
    "FollowPath.wz_max",
    "FollowPath.GoalCritic.cost_weight",
    "FollowPath.PathFollowCritic.cost_weight",
    "FollowPath.PathAlignCritic.cost_weight",
    "FollowPath.PathAngleCritic.cost_weight",
    "FollowPath.PreferForwardCritic.cost_weight",
    "FollowPath.CostCritic.cost_weight",
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
    keys = dotted_key.split(".")
    node = yaml_data
    for k in keys:
        if isinstance(node, dict) and k in node:
            node = node[k]
        else:
            return None
    return node


def _load_nav_yaml(filename):
    """Load a nav2 YAML config file from the nav_main package."""
    config_dir = os.path.join(get_package_share_directory("nav_main"), "config")
    path = os.path.join(config_dir, filename)
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
        floats = [float(v) for v in value]
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=floats,
        )
    return p


class PersonGoalSmoother(Node):
    def __init__(self):
        super().__init__("person_goal_smoother")
        cb = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter("alpha", 0.3)           # EMA smoothing (0=full smooth, 1=no smooth)
        self.declare_parameter("distance_gate", 0.3)    # Min distance (m) to publish new goal
        self.declare_parameter("follow_distance", 0.8)  # Stay this far behind the person
        self.declare_parameter("timeout", 3.0)           # Seconds without tracker data to stop
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("snap_radius", 20)        # Grid cells to search for free space

        # --- State ---
        self.smooth_x = None
        self.smooth_y = None
        self.last_pub_x = None
        self.last_pub_y = None
        self.last_tracker_time = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_data = None
        self.map_info = None

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(
            PointStamped, TRACKER_TOPIC, self._tracker_cb, 10,
            callback_group=cb,
        )
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._map_cb, 1,
            callback_group=cb,
        )

        # --- Publisher ---
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_UPDATE_TOPIC, 10)

        # --- Load both YAML configs ---
        try:
            self.follow_yaml = _load_nav_yaml("nav2_following.yaml")
            self.standard_yaml = _load_nav_yaml("nav2_standard.yaml")
            self.get_logger().info("Loaded nav2_following.yaml and nav2_standard.yaml")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML configs: {e}")
            self.follow_yaml = {}
            self.standard_yaml = {}

        # --- Nav2 param switching clients ---
        self.param_clients = {
            "controller_server": self.create_client(
                SetParameters, "/controller_server/set_parameters", callback_group=cb,
            ),
            "local_costmap": self.create_client(
                SetParameters, "/local_costmap/local_costmap/set_parameters", callback_group=cb,
            ),
            "global_costmap": self.create_client(
                SetParameters, "/global_costmap/global_costmap/set_parameters", callback_group=cb,
            ),
            "velocity_smoother": self.create_client(
                SetParameters, "/velocity_smoother/set_parameters", callback_group=cb,
            ),
        }
        self.follow_mode_active = False

        # --- Follow mode service ---
        self.create_service(
            SetBool, FOLLOW_MODE_SERVICE, self._follow_mode_cb, callback_group=cb,
        )

        # --- Timer ---
        self.create_timer(0.1, self._update_robot_pose, callback_group=cb)

        self.get_logger().info("Person Goal Smoother ready")

    # ── Nav2 param switching ──────────────────────────────────

    def _follow_mode_cb(self, request, response):
        """Service to switch between follow and standard nav params."""
        if request.data:
            self._apply_nav_params(follow=True)
            self.follow_mode_active = True
            response.message = "Follow mode activated"
        else:
            self._apply_nav_params(follow=False)
            self.follow_mode_active = False
            self.smooth_x = None
            self.smooth_y = None
            self.last_pub_x = None
            self.last_pub_y = None
            response.message = "Standard mode restored"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _apply_nav_params(self, follow: bool):
        """Set Nav2 params for follow or standard mode, loaded from YAMLs."""
        src = self.follow_yaml if follow else self.standard_yaml
        fallback = self.standard_yaml if follow else self.follow_yaml

        # 1. Controller server (MPPI)
        params = self._extract_params(src, fallback, "controller_server.ros__parameters", CONTROLLER_KEYS)
        self._set_params("controller_server", params)

        # 2. Local costmap — inflation + toggle depth layers
        params = self._extract_params(src, fallback, "local_costmap.local_costmap.ros__parameters", COSTMAP_KEYS)
        for layer in LAYERS_TO_DISABLE_IN_FOLLOW:
            params.append(_make_param(f"{layer}.enabled", not follow))
        for layer in LAYERS_ALWAYS_ENABLED:
            params.append(_make_param(f"{layer}.enabled", True))
        self._set_params("local_costmap", params)

        # 3. Global costmap — same
        params = self._extract_params(src, fallback, "global_costmap.global_costmap.ros__parameters", COSTMAP_KEYS)
        for layer in LAYERS_TO_DISABLE_IN_FOLLOW:
            params.append(_make_param(f"{layer}.enabled", not follow))
        for layer in LAYERS_ALWAYS_ENABLED:
            params.append(_make_param(f"{layer}.enabled", True))
        self._set_params("global_costmap", params)

        # 4. Velocity smoother
        params = self._extract_params(src, fallback, "velocity_smoother.ros__parameters", SMOOTHER_KEYS)
        self._set_params("velocity_smoother", params)

    def _extract_params(self, primary, fallback, section, keys):
        """Extract Parameter msgs from YAML data for the given keys."""
        params = []
        for key in keys:
            full_key = f"{section}.{key}"
            val = _resolve_yaml(primary, full_key)
            if val is None:
                val = _resolve_yaml(fallback, full_key)
            if val is not None:
                params.append(_make_param(key, val))
        return params

    def _set_params(self, node_name, params):
        """Call SetParameters service on a Nav2 node."""
        if not params:
            return
        client = self.param_clients.get(node_name)
        if client is None or not client.service_is_ready():
            self.get_logger().warn(f"SetParameters not ready for {node_name}")
            return
        req = SetParameters.Request()
        req.parameters = params
        future = client.call_async(req)
        future.add_done_callback(lambda f, n=node_name: self._param_result_cb(f, n))

    def _param_result_cb(self, future, node_name):
        try:
            result = future.result()
            failed = [r for r in result.results if not r.successful]
            if failed:
                self.get_logger().warn(
                    f"Some params failed on {node_name}: "
                    f"{[r.reason for r in failed]}"
                )
            else:
                self.get_logger().info(f"Params set on {node_name}")
        except Exception as e:
            self.get_logger().error(f"SetParameters failed on {node_name}: {e}")

    # ── Callbacks ──────────────────────────────────────────────

    def _tracker_cb(self, msg: PointStamped):
        """Receive tracker point, transform, smooth, gate, publish."""
        map_frame = self.get_parameter("map_frame").value

        # Transform point to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, msg.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1),
            )
            point_map = do_transform_point(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}", throttle_duration_sec=2.0)
            return

        px = point_map.point.x
        py = point_map.point.y

        # Reject NaN / inf
        if not math.isfinite(px) or not math.isfinite(py):
            return

        self.last_tracker_time = time.time()

        # EMA smoothing
        alpha = self.get_parameter("alpha").value
        if self.smooth_x is None:
            self.smooth_x = px
            self.smooth_y = py
        else:
            self.smooth_x = alpha * px + (1.0 - alpha) * self.smooth_x
            self.smooth_y = alpha * py + (1.0 - alpha) * self.smooth_y

        # Distance gate: only publish if person moved significantly
        dist_gate = self.get_parameter("distance_gate").value
        if self.last_pub_x is not None:
            dx = self.smooth_x - self.last_pub_x
            dy = self.smooth_y - self.last_pub_y
            if math.sqrt(dx * dx + dy * dy) < dist_gate:
                return

        self._publish_goal()

    def _map_cb(self, msg: OccupancyGrid):
        print("MAP OBTAINED")
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    def _update_robot_pose(self):
        """Get current robot position in map frame for orientation computation."""
        map_frame = self.get_parameter("map_frame").value
        try:
            t = self.tf_buffer.lookup_transform(
                map_frame, "base_link",
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05),
            )
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
        except Exception:
            pass

    # ── Goal publishing ────────────────────────────────────────

    def _publish_goal(self):
        """Compute follow point, snap to free space, publish PoseStamped."""
        follow_dist = self.get_parameter("follow_distance").value
        map_frame = self.get_parameter("map_frame").value

        # Compute goal: point between robot and person, follow_dist behind person
        dx = self.smooth_x - self.robot_x
        dy = self.smooth_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.1:
            return  # Too close, no meaningful direction

        # Goal is follow_dist meters behind the person (toward robot)
        if dist > follow_dist:
            ratio = (dist - follow_dist) / dist
            goal_x = self.robot_x + dx * ratio
            goal_y = self.robot_y + dy * ratio
        else:
            goal_x = self.robot_x
            goal_y = self.robot_y

        # Snap to free space on map
        goal_x, goal_y = self._snap_to_free(goal_x, goal_y)

        # Orientation: face toward the person
        yaw = math.atan2(self.smooth_y - goal_y, self.smooth_x - goal_x)

        # Build PoseStamped
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal_msg)

        self.last_pub_x = self.smooth_x
        self.last_pub_y = self.smooth_y

        self.get_logger().info(
            f"Goal: ({goal_x:.2f}, {goal_y:.2f}) yaw={math.degrees(yaw):.0f}° "
            f"person=({self.smooth_x:.2f}, {self.smooth_y:.2f}) dist={dist:.2f}m",
            throttle_duration_sec=1.0,
        )

    def _snap_to_free(self, wx: float, wy: float) -> tuple:
        """Snap world coordinates to the nearest free cell on the occupancy grid."""
        if self.map_data is None or self.map_info is None:
            return wx, wy

        info = self.map_info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        # World to grid
        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)
        h, w = self.map_data.shape

        # Check if already free (0 = free, 100 = occupied, -1 = unknown)
        if 0 <= mx < w and 0 <= my < h and self.map_data[my, mx] == 0:
            return wx, wy

        # Spiral search for nearest free cell
        radius = self.get_parameter("snap_radius").value
        for r in range(1, radius + 1):
            for ddx in range(-r, r + 1):
                for ddy in range(-r, r + 1):
                    if abs(ddx) != r and abs(ddy) != r:
                        continue  # Only check perimeter
                    nx, ny = mx + ddx, my + ddy
                    if 0 <= nx < w and 0 <= ny < h and self.map_data[ny, nx] == 0:
                        snapped_wx = nx * res + ox + res / 2.0
                        snapped_wy = ny * res + oy + res / 2.0
                        return snapped_wx, snapped_wy

        return wx, wy  # No free cell found, return original


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

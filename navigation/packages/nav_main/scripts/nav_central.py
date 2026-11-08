#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from composition_interfaces.srv import LoadNode
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from nav2_msgs.srv import ManageLifecycleNodes, ClearEntireCostmap
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.srv import GetMap
from std_srvs.srv import Empty, Trigger, SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
        APPROACH_POINT_SERVICE,
        GLOBAL_COSTMAP_TOPIC,
        MAP_TOPIC,
        CHECK_DOOR_SERVICE,
        DOOR_CHECK,
        AREAS_SERVICE,
        GET_AREA_FOR_POINT_SERVICE,
        SUBLOCATION_MAX_DISTANCE,
        TIMEOUT_REQUIREMENTS,
        CAMERA_RGB_TOPIC,
        CAMERA_INFO_TOPIC,
        CAMERA_DEPTH_TOPIC,
        TIMEOUT_RTABMAP,
        TIMEOUT_RTAB_SERVICE,
        TIMEOUT_NAV2_LIFECYCLE,
        RTAB_PAUSE_SERVICE,
        RTAB_RESUME_SERVICE,
        NAV2_LIFECYCLE_SERVICE,
        RTAB_CHECK_TOPIC,
        RTAB_MAPS_PATH,
        RTAB_CONTAINER_NODE,
        NO_TF_LIMIT,
        NO_TOPICS_LIMIT,
        MONITOR_RATE,
        MOVE_LOCATION_SERVICE,
        GOAL_NAV_ACTION_SERVER,
        INITIAL_POSE_TOPIC,
        RESUME_NAV_SERVICE,
        FOLLOW_MODE_SERVICE,
        FOLLOW_PERSON_NAV_SERVICE,
        GOAL_UPDATE_TOPIC,
        FOLLOW_MODE_SERVICE_TIMEOUT,
        FOLLOW_GOAL_UPDATE_TIMEOUT,
        FOLLOW_ACTION_SERVER_TIMEOUT,
        NAV_QUERY_SERVICE,
        GO_TO_POSE_SERVICE,
        GET_ROBOT_POSE_SERVICE,
        COMPUTE_PATH_ACTION_SERVER,
        TIMEOUT_NAV_QUERY,
        UNDOCK_SERVICE,
        DOCK_SERVICE,
        DOCK_TABLE_SERVICE,
        DEFAULT_DOCK_OFFSET,
        )
from ament_index_python.packages import get_package_share_directory
import os
from frida_interfaces.srv import (
        CheckDoor,
        MapAreas,
        MoveLocation,
        NavQuery,
        DockTable,
        GoToPose,
        GetRobotPose,
        ApproachPoint,
        GetAreaForPoint,
        )
from nav_main.semantic.areas import area_for_point
import tf2_ros
import json
import time as t
import math
import statistics
import yaml
import re


# Seconds to wait between NavigateToPose retries when a goal is rejected or
# aborted. send_nav_goal keeps retrying (by default forever) until Nav2 reports
# the goal SUCCEEDED, so a transient abort no longer leaves the robot stranded.
NAV_GOAL_RETRY_DELAY = 2.0
APPROACH_DIRECT_ALIGN_RANGE = 2.0
# Max time to wait for the arm pointer to home back to its normal pose before
# returning from a nav goal. Bounded so a stuck/absent arm never blocks nav.
ARM_HOME_TIMEOUT = 10.0


def make_param(name, value):
    """Convert a Python value to a rcl_interfaces Parameter msg."""
    p = Parameter()
    p.name = name
    if isinstance(value, bool):
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, int):
        p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    elif isinstance(value, float):
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    elif isinstance(value, str):
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    return p


def params_from_yaml(yaml_path, node_name):
    """Load parameters for a specific node from a YAML file."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    ros_params = data.get(node_name, {}).get('ros__parameters', {})
    return [make_param(k, v) for k, v in ros_params.items()]


def door_window_median(ranges, lo, hi, sensor_min, far_value):
    """Median range over beam indices lo..hi (wraps past the end when lo > hi).
    inf/over-range beams count as far_value (door open -> beam passes through);
    nan/under-range beams are ignored. None when no beam is valid. Median, not
    mean: the merged /scan leaves a few empty (inf) bins even in front of a
    closed door, and at far_value each one drags a mean past DOOR_DISTANCE."""
    window = list(ranges[lo:]) + list(ranges[:hi + 1]) if lo > hi else list(ranges[lo:hi + 1])
    pts = [far_value if (math.isinf(r) or r > far_value) else r for r in window
           if math.isinf(r) or not (math.isnan(r) or r < sensor_min)]
    return statistics.median(pts) if pts else None


class Nav_Central(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.nav_logger("info", "NAV_CENTRAL STARTED") 
        self.localization = self.declare_parameter('localization', False).value
        self.mapping = self.declare_parameter('mapping', False).value
        # use_nav2: enable Nav2 even in mapping mode (hybrid SLAM+navigation)
        # Defaults to True when not in pure-mapping mode (backward compatible)
        self.use_nav2 = self.declare_parameter('use_nav2', not self.mapping).value
        self.map_name= self.declare_parameter('map_name', 'rtabmap_map.db').value
        self.mapping_config = self.declare_parameter('rtab_mapping_config', '').value
        self.localization_config = self.declare_parameter('rtab_localization_config', '').value

        # Base + SLAM backend selection.
        #   default_base: 'omnibase' (slam_toolbox)
        #   nav_type:     '2d' (slam_toolbox / lidar) or '3d' (RTABMap)
        self.default_base = self.declare_parameter('default_base', 'omnibase').value
        self.nav_type = self.declare_parameter('nav_type', '2d').value
        self.use_slam_toolbox = (self.default_base == 'omnibase' and self.nav_type == '2d')
        # Topic that proves the active SLAM backend is alive (setup + monitor use it).
        self.slam_check_topic = '/map' if self.use_slam_toolbox else RTAB_CHECK_TOPIC

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None
        self.required_frames = {'link_eef'}
        self.required_topics = {'/cmd_vel', '/scan'}
        if not self.use_slam_toolbox:
            # RTABMap is RGBD; slam_toolbox is lidar-only.
            self.required_topics.add('/zed/zed_node/rgb/camera_info')

        self.rtabmap_remapping = [
            f'rgb/image:={CAMERA_RGB_TOPIC}',
            f'rgb/camera_info:={CAMERA_INFO_TOPIC}',
            f'depth/image:={CAMERA_DEPTH_TOPIC}'
                ]         

        self.config_path = self.localization_config if self.localization else self.mapping_config


        self.areas_map_name = self.declare_parameter('areas_map_name', 'default_map').value
        self.areas_data = None

        # Opt-in navigation tuning (defaults keep the historical behavior).
        # 0 = unlimited attempts / no deadline; False/True as named.
        self.nav_goal_max_attempts = self.declare_parameter('nav_goal_max_attempts', 0).value
        self.nav_goal_deadline_s = self.declare_parameter('nav_goal_deadline_s', 0.0).value
        self.clear_global_on_goal = self.declare_parameter('clear_global_on_goal', True).value
        self.approach_allow_unknown = self.declare_parameter('approach_allow_unknown', False).value

        self.lidar_group = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.rtab_service_group = ReentrantCallbackGroup()
        self.rtabmap_pause_client = self.create_client(
            Empty, RTAB_PAUSE_SERVICE, callback_group=self.rtab_service_group)
        self.rtabmap_resume_client = self.create_client(
            Empty, RTAB_RESUME_SERVICE, callback_group=self.rtab_service_group)
        # Undock (retreat) client — lets us back away from a docked surface before
        # planning a new goal. No-op if the table_docker node isn't running.
        self.undock_client = self.create_client(
            Trigger, UNDOCK_SERVICE, callback_group=self.rtab_service_group)
        # Dock (perpendicular approach) client + a parameter client to set the
        # per-location front_offset on table_docker before approaching.
        self.dock_client = self.create_client(
            Trigger, DOCK_SERVICE, callback_group=self.rtab_service_group)
        self.dock_param_client = self.create_client(
            SetParameters, '/table_docker/set_parameters', callback_group=self.rtab_service_group)
        # Costmap clear clients — wipe stale obstacle marks (e.g. from the parked/docked
        # pose) before a new goal so the planner/MPPI start from a clean slate.
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap',
            callback_group=self.rtab_service_group)
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap',
            callback_group=self.rtab_service_group)

        self.lidar_msg = None
        self.check_door_srv = self.create_service(CheckDoor, CHECK_DOOR_SERVICE, self.check_door, callback_group=self.service_group)
        self.map_areas_srv = self.create_service(MapAreas, AREAS_SERVICE, self.map_areas_callback, callback_group=self.service_group)
        self.get_area_for_point_srv = self.create_service(
            GetAreaForPoint, GET_AREA_FOR_POINT_SERVICE, self.get_area_for_point_callback,
            callback_group=self.service_group)
        
        self.move_location_srv = self.create_service(MoveLocation, MOVE_LOCATION_SERVICE, self.go_to_area, callback_group=self.service_group)
        self.dock_table_srv = self.create_service(DockTable, DOCK_TABLE_SERVICE, self.dock_table_callback, callback_group=self.service_group)
        self.goal_action_client = ActionClient(self,NavigateToPose ,GOAL_NAV_ACTION_SERVER)

        # Path query service — distance/time between two areas without navigating
        self.nav_query_srv = self.create_service(NavQuery, NAV_QUERY_SERVICE, self.query_path, callback_group=self.service_group)
        self.compute_path_client = ActionClient(self, ComputePathToPose, COMPUTE_PATH_ACTION_SERVER)

        # Manual resume service — lets the UI unpause nav2 + RTABMap on demand
        self.resume_nav_srv = self.create_service(
            Empty, RESUME_NAV_SERVICE, self._resume_nav_callback,
            callback_group=self.service_group)

        # Follow person: orchestrates person_goal_smoother + initial NavigateToPose goal
        self.follow_person_srv = self.create_service(
            SetBool, FOLLOW_PERSON_NAV_SERVICE, self._follow_person_callback,
            callback_group=self.service_group)
        self.follow_mode_client = self.create_client(
            SetBool, FOLLOW_MODE_SERVICE, callback_group=self.service_group)
        self._follow_goal_handle = None  # Track active follow goal for cancellation
        self._follow_bt_xml = os.path.join(
            get_package_share_directory("nav_main"), "bt", "follow_dynamic_point.xml"
        )
        self._latest_goal_update = None  # Cached latest PoseStamped from person_goal_smoother
        # Dedicated reentrant group: _start_follow_person() (on service_group, a
        # MutuallyExclusive group) BLOCKS waiting for the first goal update. This
        # sub MUST live in a different group so _goal_update_cb can run on another
        # thread of the MultiThreadedExecutor and unblock it — otherwise deadlock.
        self._follow_cb_group = ReentrantCallbackGroup()
        self._goal_update_sub = self.create_subscription(
            PoseStamped, GOAL_UPDATE_TOPIC, self._goal_update_cb, 10,
            callback_group=self._follow_cb_group,
        )

        # Point-based navigation services
        self.go_to_pose_srv = self.create_service(
            GoToPose, GO_TO_POSE_SERVICE, self.go_to_pose_callback,
            callback_group=self.service_group)
        self.get_robot_pose_srv = self.create_service(
            GetRobotPose, GET_ROBOT_POSE_SERVICE, self.get_robot_pose_callback,
            callback_group=self.service_group)
        self.approach_point_srv = self.create_service(
            ApproachPoint, APPROACH_POINT_SERVICE, self.approach_point_callback,
            callback_group=self.service_group)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Occupancy grids for approach_point free-space checks: prefer the
        # global costmap (static map + inflation + live obstacles, latched by
        # Nav2 even while its lifecycle is paused); fall back to the SLAM map.
        self._costmap_grid = None
        self._static_grid = None
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, GLOBAL_COSTMAP_TOPIC,
            lambda msg: setattr(self, "_costmap_grid", msg), costmap_qos,
            callback_group=self.service_group)
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC,
            lambda msg: setattr(self, "_static_grid", msg), 1,
            callback_group=self.service_group)

        # Initial pose tracking
        self._initial_pose_set = False
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            INITIAL_POSE_TOPIC,
            self._initialpose_callback,
            10
        )

        #Setup and Configuration
        self._setup_done = False
        self._setup_timer = self.create_timer(2.0, self._setup, callback_group=ReentrantCallbackGroup())
        self.no_topics_count = 0
        self.no_tf_count = 0
        self.nodes_status = False
        self.nav2_paused = False
        self.rtabmap_loaded = False
        self.rtabmap_reloading = False
        self.baseline_tf_static_publishers = None

    def _setup(self):
        """Setup of all navigation environment"""
        if self._setup_done:
            return
        self._setup_done = True
        self.destroy_timer(self._setup_timer)
        if self.use_nav2:
            # Create lifecycle client early so DDS has time to match endpoints
            self._lifecycle_cb_group = ReentrantCallbackGroup()
            self.lifecycle_client = self.create_client(
                ManageLifecycleNodes,
                NAV2_LIFECYCLE_SERVICE,
                callback_group=self._lifecycle_cb_group
            )
        if not self.mapping:
            self.load_map_areas()
        self.nav_logger("info", "Starting Setup, waiting for requirements ...")
        self.wait_for_requirements()
        self.nav_logger("info", "Requirements Completed, Starting Slam ...")
        self.start_slam()
        self.rtabmap_loaded = True
        if self.use_nav2:
            self.nav_logger("info", "Slam completed, Starting nav2 ...")
            self.load_nav2()
            self.nav_logger("info", "Nav2 completed")
            if not self.mapping:
                self._wait_for_initial_pose()
        else:
            self.nav_logger("info", "Nav2 not enabled, skipped")
        self.nav_logger("info", "Finished Setup, Starting monitoring ...")
        self.nodes_status = True
        self.baseline_tf_static_publishers = len(self.get_publishers_info_by_topic('/tf_static'))
        self.nav_logger("info", f"Baseline /tf_static publishers: {self.baseline_tf_static_publishers}")
        self._monitor_timer = self.create_timer(MONITOR_RATE, self._monitoring, callback_group=ReentrantCallbackGroup())
        self.nav_logger("info", "Monitor Started")

    def _monitoring(self):
        """General topics and tf monitor """
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Check if the SLAM backend crashed (was loaded but its topic disappeared)
        if self.rtabmap_loaded and not self.check_for_topics({self.slam_check_topic}):
            self.rtabmap_loaded = False
            self.nav_logger("warn", "Monitor -> SLAM backend crashed, will reload when requirements are met")

        # Reload rtabmap if it crashed and requirements are available
        if not self.rtabmap_loaded and not self.rtabmap_reloading:
            if self.check_for_topics(self.required_topics):
                self.rtabmap_reloading = True
                self.nav_logger("info", "Monitor -> Reloading rtabmap into respawned container ...")
                self.start_slam()
                self.rtabmap_loaded = True
                self.rtabmap_reloading = False
                self.nav_logger("info", "Monitor -> Rtabmap reloaded successfully")

        #Get Available Topics
        topics_ready = self.check_for_topics(self.required_topics)
        if not topics_ready:
            self.no_topics_count += 1
        else:
            self.no_topics_count = 0
        #Get available tf
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            current_time = self.get_clock().now().nanoseconds / 1e9
            static_publishers = len(self.get_publishers_info_by_topic('/tf_static'))
            tf_ready = True
            for frame in self.required_frames:
                if frame not in frames_yaml:
                    tf_ready = False
                    break
                match = re.search(rf'{re.escape(frame)}.*?most_recent_transform:\s*([\d.]+)', frames_yaml, re.DOTALL)
                if not match:
                    tf_ready = False
                    break
                last_time = float(match.group(1))
                if last_time == 0.0:
                    # Static transform — detect if any publisher dropped since startup
                    if self.baseline_tf_static_publishers and static_publishers < self.baseline_tf_static_publishers:
                        tf_ready = False
                        break
                else:
                    # Dynamic transform — check freshness
                    if (current_time - last_time) > 2.0:
                        tf_ready = False
                        break
        except Exception:
            tf_ready = False
            self.nav_logger("error", "Monitoring -> Failed to get tf")
        if not tf_ready:
            self.no_tf_count += 1
        else:
            self.no_tf_count = 0

       # self.nav_logger("info", f"Monitoring -> Topics = {topics_ready} TF = {tf_ready}, ntoc = {self.no_topics_count} , ntfc = {self.no_tf_count}")
        #Check count limit
        if (self.no_topics_count >= NO_TOPICS_LIMIT or self.no_tf_count >= NO_TF_LIMIT) and self.nodes_status:
            self.nodes_status = False
            self.nav_logger("warn", f"Monitor -> {'TF not available' if self.no_tf_count >= NO_TF_LIMIT else ''}, {'Topics not available' if self.no_topics_count >= NO_TOPICS_LIMIT else ''}, pausing nodes ...")
            self._set_paused(True)
        elif self.no_topics_count == 0 and self.no_tf_count == 0 and not self.nodes_status:
            self.nodes_status = True
            self.nav_logger("info", "Monitor -> Requirements available, Activating nodes ...")
            self._set_paused(False)
        

    def nav_logger(self, status, data):
        # One call per level: rclpy pins a single severity to each call site.
        if status == "info":
            self.get_logger().info(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;119m {data}\033[0m")
        elif status == "warn":
            self.get_logger().warn(f"\033[35m\033[1mNav_Control: \033[22m\033[33m {data}\033[0m")
        elif status == "error":
            self.get_logger().error(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;167m {data}\033[0m")
        else:
            self.get_logger().fatal(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;88m {data}\033[0m")

    def _resume_nav_callback(self, request, response):
        """Service callback: manually resume RTABMap and nav2 from the UI."""
        self.nav_logger("info", "Resume Nav Service -> Manual resume requested")
        self._set_paused(False)
        self.nodes_status = True
        self.no_topics_count = 0
        self.no_tf_count = 0
        return response

    def _follow_person_callback(self, request, response):
        """Service callback: activate/deactivate person-following navigation.

        On True:  switches person_goal_smoother to follow mode AND sends a
                  NavigateToPose goal with the follow BT so Nav2 starts
                  tracking the moving goal from /goal_update.
        On False: cancels the active goal AND switches smoother back to
                  standard mode.
        """
        self.nav_logger("info", f"Follow Person Service -> follow={request.data}")
        (self._start_follow_person if request.data else self._stop_follow_person)()
        response.success = True
        response.message = f"Follow person {'started' if request.data else 'stopped'}"
        return response

    def _goal_update_cb(self, msg):
        """Cache latest goal from person_goal_smoother for initial-goal seeding."""
        self._latest_goal_update = msg

    def _start_follow_person(self):
        """Activate smoother follow mode and send the initial goal."""
        # 0. Resume Nav2/SLAM FIRST. After a stationary phase (e.g. the HRIC
        # introduction) the idle monitor pauses Nav2, so bt_navigator is INACTIVE
        # and rejects the follow goal ("Action server is inactive"). Mirror the
        # go_to_pose/go_to_area path so the lifecycle is active before we send.
        if not self._ready_to_drive("Follow Person"):
            return

        # 1. Tell person_goal_smoother to switch Nav2 params to follow mode
        if self.follow_mode_client.wait_for_service(timeout_sec=FOLLOW_MODE_SERVICE_TIMEOUT):
            req = SetBool.Request()
            req.data = True
            self.follow_mode_client.call_async(req)
            self.nav_logger("info", "Follow Person -> smoother set to follow mode")
        else:
            self.nav_logger("warn", "Follow Person -> smoother service not available")

        # 2. Wait briefly for the goal smoother to publish the first goal
        wait_start = self.get_clock().now()
        while self._latest_goal_update is None:
            elapsed = (self.get_clock().now() - wait_start).nanoseconds / 1e9
            if elapsed > FOLLOW_GOAL_UPDATE_TIMEOUT:
                self.nav_logger("warn", "Follow Person -> no goal update yet, using dummy pose")
                break
            self.get_clock().sleep_for(Duration(seconds=0.1))

        # 3. Build initial goal: use latest smoothed goal if available, else dummy
        if self._latest_goal_update is not None:
            initial_pose = self._latest_goal_update
            self.nav_logger(
                "info",
                f"Follow Person -> seeding initial goal "
                f"({initial_pose.pose.position.x:.2f}, {initial_pose.pose.position.y:.2f})"
            )
        else:
            # No smoothed goal yet: seed with the robot's CURRENT pose (always
            # reachable) so the planner doesn't abort the follow BT before the
            # GoalUpdater swaps in the real /goal_update target. The old map-origin
            # (0,0) seed was usually unreachable -> "Failed to create plan" -> the
            # whole follow goal aborted and the base never moved.
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = "map"
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.orientation.w = 1.0
            tf = self._robot_tf()
            if tf is not None:
                initial_pose.pose.position.x = tf.transform.translation.x
                initial_pose.pose.position.y = tf.transform.translation.y
                initial_pose.pose.orientation = tf.transform.rotation
                self.nav_logger(
                    "warn",
                    f"Follow Person -> no goal update yet, seeding current pose "
                    f"({initial_pose.pose.position.x:.2f}, {initial_pose.pose.position.y:.2f})",
                )
            else:
                self.nav_logger("warn", "Follow Person -> no goal update and no TF; using origin")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = initial_pose
        goal_msg.behavior_tree = self._follow_bt_xml

        if not self.goal_action_client.wait_for_server(timeout_sec=FOLLOW_ACTION_SERVER_TIMEOUT):
            self.nav_logger("error", "Follow Person -> NavigateToPose action not available")
            return

        goal_future = self.goal_action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._follow_goal_response_cb)
        self.nav_logger("info", "Follow Person -> sent initial goal with follow BT")

    def _follow_goal_response_cb(self, future):
        """Store goal handle so we can cancel it later."""
        try:
            handle = future.result()
            if handle.accepted:
                self._follow_goal_handle = handle
                self.nav_logger("info", "Follow Person -> goal accepted")
            else:
                self.nav_logger("warn", "Follow Person -> goal rejected by Nav2")
        except Exception as e:
            self.nav_logger("error", f"Follow Person -> goal callback error: {e}")

    def _stop_follow_person(self):
        """Cancel the active follow goal and restore standard nav params."""
        # 1. Cancel the running goal (robot stops)
        if self._follow_goal_handle is not None:
            try:
                self._follow_goal_handle.cancel_goal_async()
                self.nav_logger("info", "Follow Person -> goal cancelled")
            except Exception as e:
                self.nav_logger("warn", f"Follow Person -> cancel error: {e}")
            self._follow_goal_handle = None

        # 2. Restore standard Nav2 params
        if self.follow_mode_client.wait_for_service(timeout_sec=FOLLOW_MODE_SERVICE_TIMEOUT):
            req = SetBool.Request()
            req.data = False
            self.follow_mode_client.call_async(req)
            self.nav_logger("info", "Follow Person -> smoother restored to standard mode")

    def _initialpose_callback(self, msg):
        if not self._initial_pose_set:
            self._initial_pose_set = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.nav_logger("info", f"Initial pose received at ({x:.2f}, {y:.2f})")

    def _wait_for_initial_pose(self):
        self.nav_logger("info", "Setup -> Waiting for initial pose to be set via nav_ui ...")
        while not self._initial_pose_set:
            self.get_clock().sleep_for(Duration(seconds=0.5))
        self.nav_logger("info", "Setup -> Initial pose confirmed, continuing setup")

    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def check_door(self, request, response):
        self.nav_logger("info", "Check_door -> Service called")
        rate = DOOR_CHECK.CHECKING_RATE.value
        response.status = False
        self.lidar_msg = None
        # /scan is published with SensorDataQoS (BEST_EFFORT) by laserscan_multi_merger;
        # a default RELIABLE subscription is QoS-incompatible and receives nothing.
        sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, qos_profile_sensor_data, callback_group=self.lidar_group)
        try:
            t.sleep(rate)  # let the subscription connect
            start_time = self.get_clock().now()
            while self.lidar_msg is None and (self.get_clock().now() - start_time) < Duration(seconds=DOOR_CHECK.TIMEOUT_SENSOR.value):
                self.nav_logger("warn", "Check_door ->  waiting for lidar msg...")
                t.sleep(rate)
            if self.lidar_msg is None:
                self.nav_logger("error", "Check_door -> Timeout reached lidar failed to retreive")
                return response

            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time) < Duration(seconds=DOOR_CHECK.TIMEOUT_TO_OPEN.value):
                self.nav_logger("info", "Check_door -> Waiting for door to open")
                msg = self.lidar_msg
                dist = door_window_median(
                    msg.ranges, DOOR_CHECK.LIDAR_RANGE_MIN.value, DOOR_CHECK.LIDAR_RANGE_MAX.value,
                    max(msg.range_min, 0.0), msg.range_max if msg.range_max > 0.0 else 12.0)
                if dist is not None:
                    self.nav_logger("info", f"Check_door -> Window median distance: {dist:.2f} m")
                    if dist > DOOR_CHECK.DOOR_DISTANCE.value:
                        self.nav_logger("info", "Check_door -> Door opened")
                        response.status = True
                        return response
                t.sleep(rate)
            self.nav_logger("error", "Check_door -> Timeout reached door didnt opened")
            return response
        finally:
            self.destroy_subscription(sub)
            self.lidar_msg = None

    def load_map_areas(self):
        try:
            pkg_share = get_package_share_directory('map_context')
            file_path = f'{pkg_share}/maps/areas/areas_{self.areas_map_name}.json'
            with open(file_path, 'r') as f:
                self.areas_data = json.load(f)
            self.nav_logger("info", f"Map_areas -> Loaded areas from {file_path}")
        except FileNotFoundError:
            self.nav_logger("error", f"Map_areas -> File not found: areas_{self.areas_map_name}.json")
        except Exception as e:
            self.nav_logger("error", f"Map_areas -> Error loading map areas: {e}")

    def map_areas_callback(self, request, response):
        self.nav_logger("info", "Map_areas -> Service called")
        if self.areas_data is None:
            self.nav_logger("error", "Map_areas -> No areas data loaded")
        else:
            response.areas = json.dumps(self.areas_data)
            self.nav_logger("info", "Map_areas -> Map Areas Sent")
        return response

    # ---------------------------------------------------------------- area lookup
    # The polygon maths lives in nav_main.semantic.areas so it can be tested
    # offline and reused (the repo already had three divergent copies).

    def get_area_for_point_callback(self, request, response):
        """Point (any TF frame) -> containing area + its nearest sublocations.

        Single source of truth for "where is this?": vision and task_manager call
        this instead of running point-in-polygon against their own copy of
        areas.json. Only the active map's areas_<MAP_NAME>.json is authoritative.
        """
        response.success = False
        response.area = ""
        response.sublocations = []
        response.distances = []
        response.in_house = False
        response.error = ""

        if self.areas_data is None:
            response.error = "no areas data loaded"
            self.nav_logger("error", "Get_area_for_point -> no areas data loaded")
            return response

        point = request.point
        frame = point.header.frame_id or "map"
        if frame != "map":
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(), timeout=Duration(seconds=1.0))
                point = do_transform_point(point, tf)
            except Exception as e:
                response.error = f"TF {frame}->map failed: {e}"
                self.nav_logger("error", f"Get_area_for_point -> {response.error}")
                return response

        x, y = point.point.x, point.point.y
        max_distance = (request.max_sublocation_distance
                        if request.max_sublocation_distance > 0.0
                        else SUBLOCATION_MAX_DISTANCE)

        area, names, distances, in_house = area_for_point(
            self.areas_data, x, y, max_distance)
        response.area = area
        response.sublocations = names
        response.distances = distances
        response.in_house = in_house

        # Success means the lookup ran, not that the point landed in a room:
        # the polygons do not tile the map, so "in no area" is a valid answer.
        response.success = True
        self.nav_logger(
            "info",
            f"Get_area_for_point -> ({x:.2f}, {y:.2f}) -> "
            f"area='{response.area or 'unknown'}' "
            f"sublocations={list(response.sublocations)[:3]}")
        return response

    def goal_feedback(self, feedback_msg):
        """Feedback function for nav goals"""

        feedback = feedback_msg.feedback
        self.nav_logger("info", f"Goal_handler -> feedback data = {feedback.distance_remaining}")

    def send_nav_goal(self, pose, behavior_tree=None, max_attempts=None, deadline_s=None):
        """Send a NavigateToPose goal and keep retrying until Nav2 reports the
        goal SUCCEEDED.

        A goal that is rejected, or that finishes with any status other than
        STATUS_SUCCEEDED (ABORTED/CANCELED — e.g. the controller momentarily
        loses TF and the BT exhausts its recoveries), is retried after a short
        delay instead of being reported as success. max_attempts bounds the
        retries; None (the default) means retry until the goal is reached.
        deadline_s bounds the total time: once exceeded, the active goal is
        cancelled and the call fails (None = no deadline)."""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        if behavior_tree is not None:
            goal_msg.behavior_tree = behavior_tree

        end_time = t.monotonic() + deadline_s if deadline_s else None

        def expired():
            return end_time is not None and t.monotonic() >= end_time

        attempt = 0
        while True:
            attempt += 1
            if expired():
                return (False, "Goal deadline exceeded")

            # Action server may not be back yet right after a lifecycle resume.
            if not self.goal_action_client.wait_for_server(timeout_sec=TIMEOUT_NAV2_LIFECYCLE):
                self.nav_logger("warn", f"Goal_Handler -> bt_navigator action server not available (attempt {attempt})")
                if max_attempts is not None and attempt >= max_attempts:
                    return (False, "Action server unavailable")
                self.get_clock().sleep_for(Duration(seconds=NAV_GOAL_RETRY_DELAY))
                continue

            _goal_future = self.goal_action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback)
            self._wait(_goal_future)
            goal_handle = _goal_future.result()

            if not goal_handle.accepted:
                self.nav_logger("warn", f"Goal_Handler -> Goal rejected (attempt {attempt}), retrying ...")
                if max_attempts is not None and attempt >= max_attempts:
                    return (False, "Goal Rejected")
                self.get_clock().sleep_for(Duration(seconds=NAV_GOAL_RETRY_DELAY))
                continue

            result_future = goal_handle.get_result_async()
            while not result_future.done():
                if expired():
                    self.nav_logger("warn", "Goal_Handler -> Deadline exceeded, cancelling goal")
                    goal_handle.cancel_goal_async()
                    return (False, "Goal deadline exceeded")
                self.get_clock().sleep_for(Duration(seconds=0.1))
            result = result_future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.nav_logger("info", "Goal_Handler -> Goal Reached")
                return (True, "Goal Finished")

            self.nav_logger("warn", f"Goal_Handler -> Goal did not succeed (status={result.status}, attempt {attempt}), retrying ...")
            if max_attempts is not None and attempt >= max_attempts:
                return (False, f"Goal failed (status {result.status})")
            self.get_clock().sleep_for(Duration(seconds=NAV_GOAL_RETRY_DELAY))

    def _retreat_if_docked(self):
        """Best-effort: ask the table_docker to back off if it's parked at a
        surface. No-op (returns fast) when the docker node isn't running."""
        if not self.undock_client.service_is_ready():
            return
        self.nav_logger("info", "Go_To_Area -> Undocking (retreat) before new goal ...")
        ok = self._call_service_with_timeout(
            self.undock_client, Trigger.Request(), TIMEOUT_NAV2_LIFECYCLE * 2, "Undock")
        if not ok:
            self.nav_logger("warn", "Go_To_Area -> Undock failed/timed out, continuing anyway")

    def _approach_table(self, offset):
        """Set the per-call front_offset on table_docker, then trigger the
        perpendicular approach. Returns (success, message)."""
        if not self.dock_client.service_is_ready():
            self.nav_logger("warn", "Approach -> table_docker not available")
            return (False, "table_docker not available")
        if offset is None or offset <= 0.0:
            offset = DEFAULT_DOCK_OFFSET
        if self.dock_param_client.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [make_param('front_offset', float(offset))]
            self._call_service_with_timeout(
                self.dock_param_client, req, TIMEOUT_RTAB_SERVICE, "Approach SetParam")
        self.nav_logger("info", f"Approach -> docking to surface (front_offset={offset})")
        # Bounded dock call, capturing the actual approach result.
        future = self.dock_client.call_async(Trigger.Request())
        if not self._wait(future, 90.0):
            return (False, "dock timed out")
        res = future.result()
        return (res.success, res.message)

    def dock_table_callback(self, request, response):
        """Service: dock to the table/shelf in front of the robot (with offset)."""
        self.nav_logger("info", f"Dock_Table -> Service called (offset={request.offset})")
        success, message = self._approach_table(request.offset)
        response.success = success
        response.error = message
        if success:
            self.nav_logger("info", "Dock_Table -> Docked")
        else:
            self.nav_logger("error", f"Dock_Table -> Failed: {message}")
        return response

    def _clear_costmaps(self, settle_s=0.3):
        """Clear local+global costmaps (global only if clear_global_on_goal) so the
        planner/MPPI start from a clean slate (removes stale obstacle marks left from
        the parked/docked pose that can box-in the planner), then settle so the layers repopulate from sensors before moving.
        Best-effort + bounded — skips a costmap whose clear service isn't up yet."""
        clients = [(self.clear_local_costmap_client, "Clear local costmap")]
        if self.clear_global_on_goal:
            clients.append((self.clear_global_costmap_client, "Clear global costmap"))
        for client, label in clients:
            if client.service_is_ready():
                self._call_service_with_timeout(
                    client, ClearEntireCostmap.Request(), TIMEOUT_RTAB_SERVICE, label)
            else:
                self.nav_logger("warn", f"{label} -> service not ready, skipping")
        # Let a sensor cycle repopulate before planning/driving. The omni base strafes
        # and the forward camera won't re-see side/rear obstacles instantly, so don't
        # command motion into the brief blind window right after the clear.
        self.get_clock().sleep_for(Duration(seconds=settle_s))

    def _ready_to_drive(self, tag):
        """Resume SLAM + Nav2 and clear costmaps. False if navigation isn't up."""
        self._set_paused(False)
        if self.nav2_paused or not self.rtabmap_loaded:
            self.nav_logger("error", f"{tag} -> Navigation not initialized")
            return False
        self._clear_costmaps()
        return True

    def _navigate(self, goal, tag, behavior_tree=None, after=None):
        """Shared goal pipeline: back off a docked surface (so nav2 doesn't plan
        from inside the inflation zone), resume + clear costmaps, drive, run
        `after` on success while nav2 is still up, then pause. Returns (ok, msg)."""
        self._retreat_if_docked()
        if not self._ready_to_drive(tag):
            return (False, "Navigation not initialized")
        start = t.monotonic()
        ok, msg = self.send_nav_goal(goal, behavior_tree, **self._nav_limits())
        if ok and after:
            after(self._remaining_budget(start))
        self._set_paused(True)
        return (ok, msg)

    def go_to_area(self,request,response):
        """Callback for navigate to specific area"""

        self.nav_logger("info", "Go_To_Area -> Starting navigation to area")

        if self.areas_data is None:
            self.nav_logger("error", "Go_To_Area -> Areas not loaded sending error")
            response.success = False
            response.error = "Areas not loaded"
            return response
        fetch_coords = self._fetch_area_coords(request.location, request.sublocation)
        if fetch_coords is None:
            self.nav_logger("error", "Go_To_Area -> Area not found")
            response.success = False
            response.error = "Area not found"
            return response

        response.success, response.error = self._navigate(
            self._pose_from_coords(fetch_coords), "Go_To_Area")
        return response

    def go_to_pose_callback(self, request, response):
        """Navigate to an arbitrary map-frame pose, through the same omni goal
        pipeline as go_to_area (retreat-if-docked, resume, clear costmaps, send goal)."""
        self.nav_logger("info", "Go_To_Pose -> Starting navigation to pose")
        goal = request.target_pose
        goal.header.frame_id = goal.header.frame_id or "map"
        response.success, response.error = self._navigate(
            goal, "Go_To_Pose", request.behavior_tree or None)
        return response

    def get_robot_pose_callback(self, request, response):
        """Return the current robot pose from TF (map -> base_link)."""
        tf = self._robot_tf()
        if tf is None:
            response.success = False
            response.error = "TF lookup failed"
            return response
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        response.success = True
        response.pose = pose
        response.error = ""
        return response

    def approach_point_callback(self, request, response):
        """Approach a person/object seen by vision: transform the target to the
        map frame, pick the nearest costmap-free pose `standoff` meters from it
        (robot side first, facing the target) and navigate there through the
        same goal pipeline as go_to_pose."""
        standoff = request.standoff if request.standoff > 0.0 else 0.65
        response.goal = PoseStamped()

        target = request.target
        frame = target.header.frame_id or "map"
        if frame != "map":
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(), timeout=Duration(seconds=1.0))
                target = do_transform_point(target, tf)
            except Exception as e:
                response.success = False
                response.error = f"TF {frame}->map failed: {e}"
                self.nav_logger("error", f"Approach_Point -> {response.error}")
                return response
        tx, ty = target.point.x, target.point.y

        tf = self._robot_tf()
        if tf is None:
            response.success = False
            response.error = "robot pose TF failed"
            return response
        rx, ry = tf.transform.translation.x, tf.transform.translation.y

        align = (request.align or "").strip().lower()
        dist = math.hypot(tx - rx, ty - ry)

        if align in ("left", "right") and dist <= APPROACH_DIRECT_ALIGN_RANGE:
            if not self.rtabmap_loaded:
                response.success = False
                response.error = "Navigation not initialized"
                return response
            self.nav_logger(
                "info",
                f"Approach_Point -> direct parallel approach "
                f"({dist:.2f} m away, align={align})")
            response.goal = self._approach_pose(rx, ry, tx, ty)
            self._retreat_if_docked()
            self.resume_slam()
            ok = self._align_parallel(tx, ty, align, request.final_distance)
            response.success = ok
            response.error = "" if ok else "parallel alignment failed"
            self.pause_slam()
            return response

        if dist <= standoff + 0.05:
            # Already close enough: face the target, never drive into it.
            goal = self._approach_pose(rx, ry, tx, ty)
            self.nav_logger("info", "Approach_Point -> already close, facing target")
        else:
            goal = self._find_free_approach(tx, ty, rx, ry, standoff)
            if goal is None:
                response.success = False
                response.error = "no free approach pose found around target"
                self.nav_logger("warn", f"Approach_Point -> {response.error}")
                return response
            self.nav_logger(
                "info",
                f"Approach_Point -> goal ({goal.pose.position.x:.2f}, "
                f"{goal.pose.position.y:.2f}) for target ({tx:.2f}, {ty:.2f})")

        response.goal = goal
        # align left/right: rotate abeam of the target instead of just facing it
        # (e.g. picking the laundry basket up from the side). Runs as the
        # `_navigate` after-hook, while nav2/slam are still resumed; a failure
        # here overrides the otherwise-successful nav goal.
        align_ok = True

        def _after(budget):
            nonlocal align_ok
            if align in ("left", "right"):
                if not self._align_parallel(tx, ty, align, request.final_distance):
                    align_ok = False
            else:
                # The goal checker can succeed inside xy tolerance without settling
                # yaw — make sure the base front actually points at the person.
                self._face_target(tx, ty, budget_s=budget)

        ok, msg = self._navigate(goal, "Approach_Point", after=_after)
        if ok and not align_ok:
            ok, msg = False, "parallel alignment failed"
        response.success = ok
        response.error = msg
        return response

    def _nav_limits(self):
        """Keyword args for send_nav_goal from the nav_goal_* parameters
        (0 means unlimited, mapped to None)."""
        return {
            "max_attempts": self.nav_goal_max_attempts or None,
            "deadline_s": self.nav_goal_deadline_s or None,
        }

    def _remaining_budget(self, start):
        """Seconds left of nav_goal_deadline_s since `start` (None if unbounded)."""
        if not self.nav_goal_deadline_s:
            return None
        return max(self.nav_goal_deadline_s - (t.monotonic() - start), 0.0)

    def _face_target(self, tx, ty, tol=0.25, budget_s=None):
        """Final in-place rotation toward (tx, ty) if the base ended up facing
        away after the approach goal (runs while nav2 is still resumed)."""
        tf = self._robot_tf(0.5)
        if tf is None:
            return
        rx, ry = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        bearing = math.atan2(ty - ry, tx - rx)
        err = (bearing - yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(err) <= tol:
            return
        self.nav_logger(
            "info", f"Approach_Point -> correcting final yaw by {math.degrees(err):.0f} deg")
        if budget_s is not None and budget_s < 5.0:
            return
        self.send_nav_goal(
            self._approach_pose(rx, ry, tx, ty), max_attempts=1, deadline_s=budget_s)

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(t)

    def _align_parallel(self, tx, ty, side, final_distance, timeout=35.0, rate=0.05):
        """Rotate to abeam (±90°) of the target, then strafe to final_distance.
        Direct cmd_vel on map->base_link TF bypasses costmap vetoes on targets
        marked as obstacles (e.g. the basket); strafe no-ops on diffdrive."""
        offset = -math.pi / 2.0 if side == "right" else math.pi / 2.0
        want_y = -final_distance if side == "right" else final_distance
        MAX_V, MAX_W = 0.12, 0.5
        K_LIN, K_ANG = 0.9, 1.5
        TOL_LIN, TOL_ANG = 0.04, 0.06

        self.nav_logger(
            "info",
            f"Approach_Point -> aligning parallel, target on the {side} "
            f"(final_distance={final_distance:.2f})")
        elapsed = 0.0
        rotated = False
        try:
            while elapsed < timeout:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "map", "base_link", rclpy.time.Time(),
                        timeout=Duration(seconds=0.5))
                except Exception as e:
                    self.nav_logger("warn", f"Approach_Point -> align TF failed: {e}")
                    return False
                rx, ry = tf.transform.translation.x, tf.transform.translation.y
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                dx, dy = tx - rx, ty - ry
                xb = math.cos(yaw) * dx + math.sin(yaw) * dy
                yb = -math.sin(yaw) * dx + math.cos(yaw) * dy
                yaw_err = (math.atan2(dy, dx) - offset - yaw + math.pi) % (2.0 * math.pi) - math.pi

                if not rotated:
                    if abs(yaw_err) <= TOL_ANG:
                        rotated = True
                        if final_distance <= 0.0:
                            break
                        continue
                    self._publish_cmd(wz=max(-MAX_W, min(MAX_W, K_ANG * yaw_err)))
                else:
                    ex, ey = xb, yb - want_y
                    if abs(ex) <= TOL_LIN and abs(ey) <= TOL_LIN and abs(yaw_err) <= TOL_ANG:
                        break
                    self._publish_cmd(
                        vx=max(-MAX_V, min(MAX_V, K_LIN * ex)),
                        vy=max(-MAX_V, min(MAX_V, K_LIN * ey)),
                        wz=max(-MAX_W, min(MAX_W, K_ANG * yaw_err)))
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=rate))
                elapsed += rate
            else:
                self.nav_logger("warn", "Approach_Point -> parallel alignment timed out")
                return False
        finally:
            self._publish_cmd()
        self.nav_logger("info", f"Approach_Point -> parallel alignment done (target abeam {side})")
        return True

    def _approach_pose(self, gx, gy, tx, ty):
        """Map-frame PoseStamped at (gx, gy) facing (tx, ty)."""
        yaw = math.atan2(ty - gy, tx - gx)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(gx)
        pose.pose.position.y = float(gy)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _grid_cell_free(self, grid, wx, wy, max_cost=50, allow_unknown=False):
        """Whether world (wx, wy) lies on a known grid cell with cost < max_cost.
        With allow_unknown, unknown (-1) and off-grid cells are accepted too."""
        info = grid.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return allow_unknown
        cost = grid.data[my * info.width + mx]
        if cost < 0:
            return allow_unknown
        return cost < max_cost

    def _near_lethal(self, grid, wx, wy, radius=0.3, lethal=65):
        """Whether any known cell within `radius` of (wx, wy) has cost >= lethal."""
        info = grid.info
        cx = int((wx - info.origin.position.x) / info.resolution)
        cy = int((wy - info.origin.position.y) / info.resolution)
        r = math.ceil(radius / info.resolution)
        for my in range(max(cy - r, 0), min(cy + r + 1, info.height)):
            for mx in range(max(cx - r, 0), min(cx + r + 1, info.width)):
                if grid.data[my * info.width + mx] >= lethal:
                    return True
        return False

    def _find_free_approach(self, tx, ty, rx, ry, standoff):
        """Nearest free approach pose: walk a ring around the target at
        `standoff` (then slightly farther), starting on the robot's side and
        fanning out. Prefers the global costmap (inflation + live obstacles);
        falls back to the SLAM map; with no grid at all, trusts the direct
        robot-side pose and lets the planner decide."""
        base = math.atan2(ry - ty, rx - tx)  # target -> robot direction
        grid = self._costmap_grid or self._static_grid
        if grid is None:
            self.nav_logger("warn", "Approach_Point -> no grid; using raw standoff pose")
            gx = tx + standoff * math.cos(base)
            gy = ty + standoff * math.sin(base)
            return self._approach_pose(gx, gy, tx, ty)
        # Pass 1: known-free cells only. Pass 2 (approach_allow_unknown): also
        # unknown cells away from lethal ones; the planner decides if reachable.
        for allow_unknown in ((False, True) if self.approach_allow_unknown else (False,)):
            for radius in (standoff, standoff + 0.15, standoff + 0.30):
                for step in range(0, 19):  # 0°, then ±10° ... ±180° around the target
                    for sign in ((1,) if step == 0 else (1, -1)):
                        ang = base + sign * math.radians(10 * step)
                        gx = tx + radius * math.cos(ang)
                        gy = ty + radius * math.sin(ang)
                        if not self._grid_cell_free(grid, gx, gy, allow_unknown=allow_unknown):
                            continue
                        if allow_unknown and self._near_lethal(grid, gx, gy):
                            continue
                        return self._approach_pose(gx, gy, tx, ty)
        return None

    def _pose_from_coords(self, coords):
        """Build a map-frame PoseStamped from an areas.json coordinate array."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        pose.pose.position.z = coords[2]
        pose.pose.orientation.x = coords[3]
        pose.pose.orientation.y = coords[4]
        pose.pose.orientation.z = coords[5]
        pose.pose.orientation.w = coords[6]
        return pose

    def _fetch_area_coords(self, location, sublocation):
        if self.areas_data is None:
            return None
        return self.areas_data.get(location, {}).get(sublocation)

    def _path_length(self, path):
        """Sum of euclidean distances between consecutive path poses."""
        total = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            total += math.hypot(p1.x - p0.x, p1.y - p0.y)
        return total

    def _compute_path(self, goal_pose, start_pose=None):
        """Call nav2 ComputePathToPose action. Returns nav_msgs/Path or None.
        If start_pose is None the planner uses the robot's current pose."""
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.planner_id = "GridBased"
        if start_pose is not None:
            goal_msg.start = start_pose
            goal_msg.use_start = True

        if not self.compute_path_client.wait_for_server(timeout_sec=TIMEOUT_NAV_QUERY):
            self.nav_logger("error", "Query_Path -> ComputePathToPose server not available")
            return None

        send_future = self.compute_path_client.send_goal_async(goal_msg)
        if not self._wait(send_future, TIMEOUT_NAV_QUERY):
            self.nav_logger("error", "Query_Path -> Timeout sending goal to planner")
            return None

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.nav_logger("error", "Query_Path -> Planner rejected the request")
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, TIMEOUT_NAV_QUERY):
            goal_handle.cancel_goal_async()
            self.nav_logger("error", "Query_Path -> Timeout waiting for planner result")
            return None

        result = result_future.result().result
        if result is None or len(result.path.poses) == 0:
            return None
        return result.path

    def query_path(self, request, response):
        """Callback: compute path distance/time between two areas without moving.
        Empty location_a means start from the robot's current pose."""
        self.nav_logger("info", f"Query_Path -> Service called ({request.location_a}/{request.sublocation_a} -> {request.location_b}/{request.sublocation_b})")
        response.success = False
        response.distance_meters = 0.0
        response.error = ""

        if self.mapping:
            response.error = "Not available in mapping mode"
            return response
        if self.areas_data is None:
            self.nav_logger("error", "Query_Path -> Areas not loaded")
            response.error = "Areas not loaded"
            return response

        goal_coords = self._fetch_area_coords(request.location_b, request.sublocation_b)
        if goal_coords is None:
            self.nav_logger("error", "Query_Path -> Destination area not found")
            response.error = f"Area not found: {request.location_b}/{request.sublocation_b}"
            return response
        goal_pose = self._pose_from_coords(goal_coords)

        start_pose = None
        if request.location_a:
            start_coords = self._fetch_area_coords(request.location_a, request.sublocation_a)
            if start_coords is None:
                self.nav_logger("error", "Query_Path -> Start area not found")
                response.error = f"Area not found: {request.location_a}/{request.sublocation_a}"
                return response
            start_pose = self._pose_from_coords(start_coords)

        if not self.rtabmap_loaded:
            self.nav_logger("error", "Query_Path -> Navigation not initialized")
            response.error = "Navigation not initialized"
            return response

        # Planner needs nav2 active; remember prior state to restore it after
        was_paused = self.nav2_paused
        self._set_paused(False)
        if self.nav2_paused:
            self.nav_logger("error", "Query_Path -> Could not resume nav2")
            response.error = "Could not resume nav2"
            return response

        try:
            path = self._compute_path(goal_pose, start_pose)
            if path is None:
                response.error = "Planner failed to compute path"
                return response
            distance = self._path_length(path)
            response.success = True
            response.distance_meters = distance
            self.nav_logger("info", f"Query_Path -> Path found: {distance:.2f} m")
        finally:
            if was_paused:
                self._set_paused(True)
        return response

    def check_for_topics(self, topics):
        # A topic counts only with a live source: /cmd_vel needs the base driver
        # subscribed, every other one a publisher. Subscribers alone keep a dead
        # topic in the graph (the costmaps keep /scan listed after the merger dies).
        return all((self.count_subscribers(t) if t == '/cmd_vel' else self.count_publishers(t)) > 0
                   for t in topics)

    def wait_for_requirements(self):
        """Function to wait until requirements are available"""
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        resources_ready = False
        while not resources_ready:
            #Get Available Topics
            topics_ready = self.check_for_topics(self.required_topics)
           #Get available tf 
            try:
                frames_dict = self.tf_buffer.all_frames_as_yaml()
                tf_ready = all(frame in frames_dict for frame in self.required_frames)
            except Exception:
                tf_ready = False
            #Check of two availables
            if topics_ready and tf_ready:
                resources_ready = True
                self.nav_logger("info", "Waiting Requirements -> Requirements complete")
            else:
                self.nav_logger("warn", f"Waiting Requirements -> {'TF not available yet' if not tf_ready else ''}, {'Topics not available yet' if not topics_ready else ''}") 
                self.get_clock().sleep_for(Duration(seconds=TIMEOUT_REQUIREMENTS))

    def start_slam(self):
        """Bring up (or confirm) the active SLAM backend."""
        if self.use_slam_toolbox:
            self.start_slam_toolbox()
        else:
            self.start_rtabmap()

    def start_slam_toolbox(self):
        """slam_toolbox runs as its own externally-launched node (see
        omni_setup/slam.launch.py). We don't load it into a container here — we
        just wait until it is publishing the map."""
        slam_topics = {self.slam_check_topic}
        self.nav_logger("info", "Loading Slam -> Waiting for slam_toolbox map ...")
        if self._wait_for_topics(slam_topics, TIMEOUT_RTABMAP):
            self.nav_logger("info", "Loading Slam -> slam_toolbox map available")
        else:
            self.nav_logger("warn", "Loading Slam -> slam_toolbox map not seen yet, continuing")

    def start_rtabmap(self):
        """Function to load slam nodes"""
        # Load a node into the container

        rtabmap_params = params_from_yaml(self.config_path, 'rtabmap')
        db_path = f'{RTAB_MAPS_PATH}{self.map_name}'
        rtabmap_params.append(make_param('database_path', db_path))
        sync_params = params_from_yaml(self.config_path, 'rgbd_sync')

        rtab_topics = {RTAB_CHECK_TOPIC}

        load_cb_group = ReentrantCallbackGroup()
        rtab_client = self.create_client(LoadNode, RTAB_CONTAINER_NODE, callback_group=load_cb_group)
        rtab_client.wait_for_service()
        self.nav_logger("info", "Loading Slam -> Started loading nodes")
        while not self.check_for_topics(rtab_topics):
            req = LoadNode.Request()                                                                                                                                                                
            req.package_name = 'rtabmap_slam'
            req.plugin_name = 'rtabmap_slam::CoreWrapper'                                                                                                                                           
            req.node_name = 'rtabmap'                                                                                                                                                               
            req.parameters = rtabmap_params  # your params
            self._wait(rtab_client.call_async(req))
            self.nav_logger("info", "Loading Slam -> RtabCore Loaded")
            req = LoadNode.Request()
            req.package_name = 'rtabmap_sync'
            req.plugin_name = 'rtabmap_sync::RGBDSync'
            req.node_name = 'rgbd_sync'
            req.parameters = sync_params
            req.remap_rules = self.rtabmap_remapping
            self._wait(rtab_client.call_async(req))
            self.nav_logger("info", "Loading Slam -> RtabSync Loaded") 
            if self._wait_for_topics(rtab_topics, TIMEOUT_RTABMAP):
                self.nav_logger("info", "Loading Slam -> Topic Founded")
            else:
                self.nav_logger("error","Loading Slam -> Topics not found trying again ...")
            
        self.destroy_client(rtab_client)
        self.nav_logger("info", "Loading Slam -> Finished Slam Loading")

    def load_nav2(self):
        """Load nav2 nodes activating lifecycle"""

        self.nav_logger("info", "Loading Nav2 -> Starting nav2 lifecycle activation ...")
        self.lifecycle_client.wait_for_service()
        self.nav_logger("info", "Loading Nav2 -> Service found, sending STARTUP")
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        self._wait(self.lifecycle_client.call_async(req))
        self.nav_logger("info", "Loading Nav2 -> Fully loaded nav2 lifecycles")
            
    def _wait(self, future, timeout=None):
        """Sleep-poll until future is done (or timeout s). Returns future.done()."""
        elapsed = 0.0
        while not future.done() and (timeout is None or elapsed < timeout):
            self.get_clock().sleep_for(Duration(seconds=0.1))
            elapsed += 0.1
        return future.done()

    def _wait_for_topics(self, topics, timeout):
        """Poll the ROS graph until all topics exist (or timeout s). Returns found."""
        elapsed = 0.0
        while not self.check_for_topics(topics) and elapsed < timeout:
            t.sleep(0.5)
            elapsed += 0.5
        return self.check_for_topics(topics)

    def _robot_tf(self, timeout=1.0):
        """map -> base_link transform, or None when TF isn't available."""
        try:
            return self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=timeout))
        except Exception as e:
            self.nav_logger("warn", f"Robot pose -> TF lookup failed: {e}")
            return None

    def _call_service_with_timeout(self, client, request, call_timeout, label):
        """Bounded service call — waits up to TIMEOUT_RTAB_SERVICE for readiness and
        call_timeout for the response. Returns True on success, False on timeout."""
        if not client.wait_for_service(timeout_sec=TIMEOUT_RTAB_SERVICE):
            self.nav_logger("warn", f"{label} -> Service not available after {TIMEOUT_RTAB_SERVICE}s")
            return False
        if not self._wait(client.call_async(request), call_timeout):
            self.nav_logger("error", f"{label} -> Call timed out after {call_timeout}s, no response")
            return False
        return True

    def _set_paused(self, paused):
        """Pause/resume SLAM (RTABMap only; slam_toolbox self-recovers), then the
        Nav2 lifecycle. No-op for whatever is disabled or already in that state."""
        verb = "Pausing" if paused else "Resuming"
        if not self.use_slam_toolbox:
            if not self.rtabmap_loaded:
                self.nav_logger("warn", f"{verb} Slam -> Rtabmap not loaded, skipping")
            elif self._call_service_with_timeout(
                    self.rtabmap_pause_client if paused else self.rtabmap_resume_client,
                    Empty.Request(), TIMEOUT_RTAB_SERVICE, f"{verb} Slam"):
                self.nav_logger("info", f"{verb} Slam -> done")
        if not self.use_nav2 or self.nav2_paused == paused:
            return
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.PAUSE if paused else ManageLifecycleNodes.Request.RESUME
        if self._call_service_with_timeout(
                self.lifecycle_client, req, TIMEOUT_NAV2_LIFECYCLE, f"{verb} Nav2"):
            self.nav2_paused = paused
            self.nav_logger("info", f"{verb} Nav2 -> done")

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = Nav_Central('nav_central')
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

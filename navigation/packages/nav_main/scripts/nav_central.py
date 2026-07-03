#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from nav2_msgs.srv import ManageLifecycleNodes, ClearEntireCostmap
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.srv import GetMap
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers PointStamped transform)
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
        APPROACH_POINT_SERVICE,
        SET_OBSTACLE_AVOIDANCE_SERVICE,
        MOVE_RELATIVE_SERVICE,
        GLOBAL_COSTMAP_TOPIC,
        MAP_TOPIC,
        CHECK_DOOR_SERVICE,
        DOOR_CHECK,
        AREAS_SERVICE,
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
from std_srvs.srv import SetBool
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
        MoveRelative,
        )
from ament_index_python.packages import get_package_share_directory
import tf2_ros
import json
import time as t
import math
import yaml
import re


# Seconds to wait between NavigateToPose retries when a goal is rejected or
# aborted. send_nav_goal keeps retrying (by default forever) until Nav2 reports
# the goal SUCCEEDED, so a transient abort no longer leaves the robot stranded.
NAV_GOAL_RETRY_DELAY = 2.0
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
        #   default_base: 'dashgo' (RTABMap RGBD SLAM) or 'omnibase' (slam_toolbox)
        #   nav_type:     '2d' (slam_toolbox / lidar) or '3d' (RTABMap)
        # Params absent -> legacy dashgo / RTABMap behaviour.
        self.default_base = self.declare_parameter('default_base', 'dashgo').value
        self.nav_type = self.declare_parameter('nav_type', '2d').value
        self.use_slam_toolbox = (self.default_base == 'omnibase' and self.nav_type == '2d')
        # Topic that proves the active SLAM backend is alive (setup + monitor use it).
        self.slam_check_topic = '/map' if self.use_slam_toolbox else RTAB_CHECK_TOPIC

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None
        if self.use_slam_toolbox:
            # slam_toolbox is lidar-only: no RGBD camera, and the omnibase has no arm.
            self.required_topics = {'/cmd_vel', '/scan'}
            self.required_frames = {'link_eef'}
        else:
            self.required_topics = {'/zed/zed_node/rgb/camera_info', '/cmd_vel', '/scan'}
            self.required_frames = {'link_eef'}
        self.requirements_timeout = TIMEOUT_REQUIREMENTS

        self.rtabmap_remapping = [
            f'rgb/image:={CAMERA_RGB_TOPIC}',
            f'rgb/camera_info:={CAMERA_INFO_TOPIC}',
            f'depth/image:={CAMERA_DEPTH_TOPIC}'
                ]         

        self.config_path = self.localization_config if self.localization else self.mapping_config
        self.rtab_load_timeout = TIMEOUT_RTABMAP


        self.areas_map_name = self.declare_parameter('areas_map_name', 'default_map').value
        self.areas_data = None

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
        # Costmap parameter clients — toggle the live obstacle layers (lidar +
        # ZED cloud) when the arm carries something the sensors would otherwise
        # mark as a wall right on top of the robot (set_obstacle_avoidance).
        self.local_costmap_param_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters',
            callback_group=self.rtab_service_group)
        self.global_costmap_param_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters',
            callback_group=self.rtab_service_group)

        self.lidar_msg = None
        self.lidar_reciever = None
        self.check_door_srv = self.create_service(CheckDoor, CHECK_DOOR_SERVICE, self.check_door, callback_group=self.service_group)
        self.map_areas_srv = self.create_service(MapAreas, AREAS_SERVICE, self.map_areas_callback, callback_group=self.service_group)
        self.range_min = DOOR_CHECK.LIDAR_RANGE_MIN.value  
        self.range_max = DOOR_CHECK.LIDAR_RANGE_MAX.value
        self.door_rate = DOOR_CHECK.CHECKING_RATE.value
        self.door_distance = DOOR_CHECK.DOOR_DISTANCE.value 
        self.sensor_timeout = Duration(seconds=DOOR_CHECK.TIMEOUT_SENSOR.value) # Timeout in seconds to wait for sensors
        self.door_timeout = Duration(seconds=DOOR_CHECK.TIMEOUT_TO_OPEN.value) # Timeout in seconds to wait for sensors
        
        self.move_location_srv = self.create_service(MoveLocation, MOVE_LOCATION_SERVICE, self.go_to_area, callback_group=self.service_group)
        self.dock_table_srv = self.create_service(DockTable, DOCK_TABLE_SERVICE, self.dock_table_callback, callback_group=self.service_group)
        self.goal_action_client = ActionClient(self,NavigateToPose ,GOAL_NAV_ACTION_SERVER)

        # Expose the current nav goal + active flag so the arm pointer (manipulation)
        # can aim the camera at the destination. transient_local so a late-joining
        # subscriber still gets the last goal. ponytail: plain topics, no constants
        # file — two strings, one consumer.
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.current_goal_pub = self.create_publisher(PoseStamped, "/nav/current_goal", _latched)
        self.goal_active_pub = self.create_publisher(Bool, "/nav/goal_active", _latched)
        # Arm pointer reports when it's back to its normal pose after the goal ends.
        # Default True so nav never blocks when the arm pointer node isn't running.
        self.arm_ready = True
        self.create_subscription(
            Bool, "/nav/arm_ready",
            lambda m: setattr(self, "arm_ready", m.data),
            _latched, callback_group=self.lidar_group)

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

        # Short relative sidesteps (direct cmd_vel on odom TF, no Nav2/costmaps)
        self.move_relative_srv = self.create_service(
            MoveRelative, MOVE_RELATIVE_SERVICE, self.move_relative_callback,
            callback_group=self.service_group)

        # Live-obstacle toggle: SetBool(False) disables obstacle_layer +
        # rgbd_obstacle_layer on both costmaps (static map keeps applying) so a
        # carried bag/basket can't box the planner in; SetBool(True) restores.
        self._obstacle_avoidance_enabled = True
        self.set_obstacle_avoidance_srv = self.create_service(
            SetBool, SET_OBSTACLE_AVOIDANCE_SERVICE, self.set_obstacle_avoidance_callback,
            callback_group=self.service_group)
        # Direct base commands for the approach_point parallel alignment (same
        # TwistStamped-on-/cmd_vel convention as table_docker; only published
        # while no Nav2 goal is active, so there is no contention with MPPI).
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
        self._montitor_timer = None 
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
            self.pause_slam()
            if self.use_nav2:
                self.pause_nav2()
        elif (self.no_topics_count == 0) and (self.no_tf_count == 0):
            if self.nodes_status == False:
                self.nodes_status = True
                self.nav_logger("info", "Monitor -> Requirements available, Activating nodes ...")
                self.resume_slam()
                if self.use_nav2:
                    self.resume_nav2()
        

    def nav_logger(self,status, data):
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
        self.resume_slam()
        if self.use_nav2:
            self.resume_nav2()
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
        if request.data:
            self._start_follow_person()
            response.success = True
            response.message = "Follow person started"
        else:
            self._stop_follow_person()
            response.success = True
            response.message = "Follow person stopped"
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
        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            self.nav_logger("error", "Follow Person -> Navigation not initialized; cannot follow")
            return
        self._clear_costmaps()

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
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

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
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0)
                )
                initial_pose.pose.position.x = tf.transform.translation.x
                initial_pose.pose.position.y = tf.transform.translation.y
                initial_pose.pose.orientation = tf.transform.rotation
                self.nav_logger(
                    "warn",
                    f"Follow Person -> no goal update yet, seeding current pose "
                    f"({initial_pose.pose.position.x:.2f}, {initial_pose.pose.position.y:.2f})",
                )
            except Exception as e:
                self.nav_logger(
                    "warn", f"Follow Person -> no goal update and no TF ({e}); using origin"
                )

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
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        self.nav_logger("info", "Setup -> Initial pose confirmed, continuing setup")

    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def check_door(self,request, response):
        self.nav_logger("info","Check_door -> Service called")

        # /scan is published with SensorDataQoS (BEST_EFFORT) by laserscan_multi_merger;
        # a default RELIABLE subscription is QoS-incompatible and receives nothing.
        self.lidar_reciever = self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, qos_profile_sensor_data, callback_group=self.lidar_group)
        self.lidar_msg = None  #Clean for cache msgs
        t.sleep(self.door_rate) #Wait for suscription to start

        start_time = self.get_clock().now()
        while self.lidar_msg is None and (self.get_clock().now() - start_time) < self.sensor_timeout:
            self.nav_logger("warn","Check_door ->  waiting for lidar msg...")
            t.sleep(self.door_rate)

        if self.lidar_msg is None:
            self.destroy_subscription(self.lidar_reciever)
            self.lidar_msg = None
            self.nav_logger("error", "Check_door -> Timeout reached lidar failed to retreive")
            response.status = False
            return response

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < self.door_timeout: #Timeout in case of absolute failure 
            self.nav_logger("info","Check_door -> Waiting for door to open")
            # Beams that find nothing (door open -> beam passes through) come back
            # as inf; clamp them to the sensor max range so they always count as
            # "far". Average every valid beam in the window each cycle instead of
            # gating on the inf count.
            far_value = self.lidar_msg.range_max if self.lidar_msg.range_max > 0.0 else 12.0
            sensor_min = self.lidar_msg.range_min if self.lidar_msg.range_min > 0.0 else 0.0

            door_points = []
            for count, r in enumerate(self.lidar_msg.ranges):
                # Select only the beams pointing at the door (index window, with
                # wrap-around support when range_min > range_max).
                if self.range_min > self.range_max:
                    in_window = (0 <= count <= self.range_max) or (count >= self.range_min)
                else:
                    in_window = self.range_min <= count <= self.range_max
                if not in_window:
                    continue

                if math.isinf(r) or r > far_value:
                    door_points.append(far_value)  # nothing in range -> door open
                elif math.isnan(r) or r < sensor_min:
                    continue  # invalid reading -> ignore
                else:
                    door_points.append(r)

            if door_points:
                avg_points = sum(door_points) / len(door_points)
                self.nav_logger("info", f"Check_door -> Window avg distance: {avg_points:.2f} m")

                if avg_points > self.door_distance:
                    self.nav_logger("info", "Check_door -> Door opened")
                    self.destroy_subscription(self.lidar_reciever)
                    self.lidar_msg = None
                    response.status = True
                    return response

            t.sleep(self.door_rate)

        self.destroy_subscription(self.lidar_reciever)
        self.lidar_msg = None
        self.nav_logger("error", "Check_door -> Timeout reached door didnt opened")
        response.status = False
        return response
             
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

    def goal_feedback(self, feedback_msg):
        """Feedback function for nav goals"""

        feedback = feedback_msg.feedback
        self.nav_logger("info", f"Goal_handler -> feedback data = {feedback.distance_remaining}")

    def send_nav_goal(self, pose, behaivor_tree = None, max_attempts = None):
        """Send a NavigateToPose goal and keep retrying until Nav2 reports the
        goal SUCCEEDED.

        A goal that is rejected, or that finishes with any status other than
        STATUS_SUCCEEDED (ABORTED/CANCELED — e.g. the controller momentarily
        loses TF and the BT exhausts its recoveries), is retried after a short
        delay instead of being reported as success. max_attempts bounds the
        retries; None (the default) means retry until the goal is reached."""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        if behaivor_tree is not None:
            goal_msg.behaivor_tree = behaivor_tree

        # Publish destination + active flag for the arm pointer; finally clears the
        # flag on any exit (success, failure, or exception).
        self.current_goal_pub.publish(pose)
        self.goal_active_pub.publish(Bool(data=True))

        attempt = 0
        try:
            while True:
                attempt += 1

                # Action server may not be back yet right after a lifecycle resume.
                if not self.goal_action_client.wait_for_server(timeout_sec=TIMEOUT_NAV2_LIFECYCLE):
                    self.nav_logger("warn", f"Goal_Handler -> bt_navigator action server not available (attempt {attempt})")
                    if max_attempts is not None and attempt >= max_attempts:
                        return (False, "Action server unavailable")
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=NAV_GOAL_RETRY_DELAY))
                    continue

                _goal_future = self.goal_action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback)
                while not _goal_future.done():
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
                goal_handle = _goal_future.result()

                if not goal_handle.accepted:
                    self.nav_logger("warn", f"Goal_Handler -> Goal rejected (attempt {attempt}), retrying ...")
                    if max_attempts is not None and attempt >= max_attempts:
                        return (False, "Goal Rejected")
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=NAV_GOAL_RETRY_DELAY))
                    continue

                result_future = goal_handle.get_result_async()
                while not result_future.done():
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
                result = result_future.result()

                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.nav_logger("info", "Goal_Handler -> Goal Reached")
                    return (True, "Goal Finished")

                self.nav_logger("warn", f"Goal_Handler -> Goal did not succeed (status={result.status}, attempt {attempt}), retrying ...")
                if max_attempts is not None and attempt >= max_attempts:
                    return (False, f"Goal failed (status {result.status})")
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=NAV_GOAL_RETRY_DELAY))
        finally:
            self.goal_active_pub.publish(Bool(data=False))
            # Hold the result until the arm has homed back to its normal pose
            # (arm_ready True), so callers don't act while the arm is still moving.
            waited = 0.0
            while not self.arm_ready and waited < ARM_HOME_TIMEOUT:
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
                waited += 0.1
            if not self.arm_ready:
                self.nav_logger("warn", "Goal_Handler -> arm did not report ready before timeout")

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
        elapsed = 0.0
        while not future.done() and elapsed < 90.0:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not future.done():
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
        """Clear local+global costmaps so the planner/MPPI start from a clean slate
        (removes stale obstacle marks left from the parked/docked pose that can box-in
        the planner), then settle so the layers repopulate from sensors before moving.
        Best-effort + bounded — skips a costmap whose clear service isn't up yet."""
        for client, label in ((self.clear_local_costmap_client,  "Clear local costmap"),
                              (self.clear_global_costmap_client, "Clear global costmap")):
            if client.service_is_ready():
                self._call_service_with_timeout(
                    client, ClearEntireCostmap.Request(), TIMEOUT_RTAB_SERVICE, label)
            else:
                self.nav_logger("warn", f"{label} -> service not ready, skipping")
        # Let a sensor cycle repopulate before planning/driving. The omni base strafes
        # and the forward camera won't re-see side/rear obstacles instantly, so don't
        # command motion into the brief blind window right after the clear.
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=settle_s))

    def go_to_area(self,request,response):
        """Callback for navigate to specific area"""

        self.nav_logger("info", "Go_To_Area -> Starting navigation to area")

        if self.areas_data is None:
            self.nav_logger("error", "Go_To_Area -> Areas not loaded sending error")
            response.success = False
            response.error = "Areas not loaded"
            return response
        fetch_coords = self.areas_data.get(request.location, {}).get(request.sublocation)
        if fetch_coords is None:
            self.nav_logger("error", "Go_To_Area -> Area not found")
            response.success = False
            response.error = "Area not found"
            return response

        # If parked at a surface, back off first so nav2 plans from a clear pose
        # (avoids the planner starting inside the inflation/lethal zone).
        self._retreat_if_docked()

        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            self.nav_logger("error", "Go_To_Area -> Navigation not initialized")
            response.success = False
            response.error = "Navigation not initialized"
            return response 
        # Fresh costmaps before planning/driving (clears stale marks from the
        # docked/retreated pose, then settles for sensor repopulation).
        self._clear_costmaps()

        goal_coord = PoseStamped()
        goal_coord.header.frame_id = "map"
        goal_coord.pose.position.x = fetch_coords[0]
        goal_coord.pose.position.y = fetch_coords[1]
        goal_coord.pose.position.z = fetch_coords[2]
        goal_coord.pose.orientation.x = fetch_coords[3]
        goal_coord.pose.orientation.y = fetch_coords[4]
        goal_coord.pose.orientation.z = fetch_coords[5]
        goal_coord.pose.orientation.w = fetch_coords[6]

        future = self.send_nav_goal(goal_coord)
        response.success = future[0]
        response.error = future[1]
        self.pause_slam()
        self.pause_nav2()
        return response

    def go_to_pose_callback(self, request, response):
        """Navigate to an arbitrary map-frame pose, through the same omni goal
        pipeline as go_to_area (retreat-if-docked, resume, clear costmaps, send goal)."""
        self.nav_logger("info", "Go_To_Pose -> Starting navigation to pose")
        self._retreat_if_docked()
        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            self.nav_logger("error", "Go_To_Pose -> Navigation not initialized")
            response.success = False
            response.error = "Navigation not initialized"
            return response
        self._clear_costmaps()
        goal = request.target_pose
        if not goal.header.frame_id:
            goal.header.frame_id = "map"
        bt = request.behavior_tree if request.behavior_tree else None
        ok, msg = self.send_nav_goal(goal, behaivor_tree=bt)
        response.success = ok
        response.error = msg
        self.pause_slam()
        self.pause_nav2()
        return response

    def get_robot_pose_callback(self, request, response):
        """Return the current robot pose from TF (map -> base_link)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception as e:
            self.nav_logger("warn", f"Get_Robot_Pose -> TF lookup failed: {e}")
            response.success = False
            response.error = f"TF lookup failed: {e}"
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

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0))
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            response.success = False
            response.error = f"robot pose TF failed: {e}"
            self.nav_logger("error", f"Approach_Point -> {response.error}")
            return response

        if math.hypot(tx - rx, ty - ry) <= standoff + 0.05:
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
        align = (request.align or "").strip().lower()
        self._retreat_if_docked()
        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            response.success = False
            response.error = "Navigation not initialized"
            return response
        self._clear_costmaps()
        ok, msg = self.send_nav_goal(goal)
        if ok:
            if align in ("left", "right"):
                # Finish parallel to the target instead of facing it, target
                # abeam on the requested side, optionally strafing in until
                # base_link sits final_distance meters from it.
                if not self._align_parallel(tx, ty, align, request.final_distance):
                    ok, msg = False, "parallel alignment failed"
            else:
                # The goal checker can succeed inside xy tolerance without settling
                # yaw — make sure the base front actually points at the person.
                self._face_target(tx, ty)
        response.success = ok
        response.error = msg
        self.pause_slam()
        self.pause_nav2()
        return response

    def _face_target(self, tx, ty, tol=0.25):
        """Final in-place rotation toward (tx, ty) if the base ended up facing
        away after the approach goal (runs while nav2 is still resumed)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:
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
        self.send_nav_goal(self._approach_pose(rx, ry, tx, ty))

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        """Direct base command (TwistStamped on /cmd_vel, same convention as
        table_docker). Only used while no Nav2 goal is active."""
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(t)

    def _align_parallel(self, tx, ty, side, final_distance, timeout=20.0, rate=0.05):
        """Final alignment for approach_point(align=left/right): rotate in
        place until the base is PARALLEL to the target (target abeam at ±90°),
        then, if final_distance > 0, strafe until base_link sits that many
        meters from the target, still abeam. Closed loop on map->base_link TF
        with direct cmd_vel, so the costmaps cannot veto getting close to a
        target that is itself marked as an obstacle (e.g. the laundry basket).
        Holonomic base only — the strafe is a no-op on a diffdrive."""
        # Desired bearing of the target in the base frame: -90° (right) / +90° (left)
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
                # Target in the base frame + yaw error vs the parallel heading.
                # Both recomputed every cycle, so the base stays parallel while
                # it strafes (the bearing changes as the robot moves).
                xb = math.cos(yaw) * dx + math.sin(yaw) * dy
                yb = -math.sin(yaw) * dx + math.cos(yaw) * dy
                yaw_err = (math.atan2(dy, dx) - offset - yaw + math.pi) % (2.0 * math.pi) - math.pi

                if not rotated:
                    # Phase 1: pure in-place rotation until parallel.
                    if abs(yaw_err) <= TOL_ANG:
                        rotated = True
                        if final_distance <= 0.0:
                            break
                        continue
                    self._publish_cmd(wz=max(-MAX_W, min(MAX_W, K_ANG * yaw_err)))
                else:
                    # Phase 2: drive the target to (0, want_y) in the base
                    # frame (abeam at the requested gap), holding yaw.
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
            self._publish_cmd()  # always leave the base stopped
        self.nav_logger("info", f"Approach_Point -> parallel alignment done (target abeam {side})")
        return True

    def move_relative_callback(self, request, response):
        """Service: short relative displacement in the current base frame
        (dx fwd+, dy left+, dyaw CCW). Direct cmd_vel — see MoveRelative.srv."""
        ok, err, traveled = self._move_relative(
            request.dx, request.dy, request.dyaw,
            stop_at_front_distance=request.stop_at_front_distance)
        response.success = ok
        response.error = err
        response.traveled = float(traveled)
        return response

    def _front_lidar_distance(self, scan, half_angle=math.radians(10.0)):
        """Min valid range in the front sector (|beam angle| <= half_angle).
        The merged /scan has 0 rad = straight ahead (see DOOR_CHECK calibration).
        None when the scan has no valid beam in the sector."""
        if scan is None:
            return None
        best = None
        sensor_min = scan.range_min if scan.range_min > 0.0 else 0.0
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            if abs(ang) > half_angle:
                continue
            if math.isnan(r) or math.isinf(r) or r < sensor_min:
                continue
            if best is None or r < best:
                best = r
        return best

    def _move_relative(self, dx, dy, dyaw, stop_at_front_distance=0.0, rate=0.05):
        """Closed-loop relative move on odom->base_link TF (odom: smooth, no
        SLAM correction jumps over a short displacement). Same direct-cmd_vel
        machinery as _align_parallel; nothing checks obstacles on the way, so
        callers keep displacements short (e.g. the 1 m basket-clearing strafe).

        With stop_at_front_distance > 0 and a forward component (dx > 0), the
        front lidar sector is monitored every cycle and the move ends EARLY
        (success) as soon as it reads at or below that distance — the
        washing-machine arm insert: dx caps the travel, the lidar reading
        against the machine's front face decides the actual depth.
        Returns (ok, error, traveled_meters)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            return False, f"odom TF failed: {e}", 0.0
        x0, y0 = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw0 = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        gx = x0 + math.cos(yaw0) * dx - math.sin(yaw0) * dy
        gy = y0 + math.sin(yaw0) * dx + math.cos(yaw0) * dy
        gyaw = yaw0 + dyaw

        MAX_V, MAX_W = 0.2, 0.5
        K_LIN, K_ANG = 1.2, 1.5
        TOL_LIN, TOL_ANG = 0.03, 0.05
        timeout = 10.0 + 10.0 * math.hypot(dx, dy)  # generous vs the 0.2 m/s cap

        # Lidar stop only makes sense while driving forward; /scan is BEST_EFFORT
        # (see check_door), so the temp subscription must use sensor QoS.
        lidar_stop = stop_at_front_distance > 0.0 and dx > 0.0
        scan_sub = None
        self._relmove_scan = None
        if lidar_stop:
            scan_sub = self.create_subscription(
                LaserScan, SCAN_TOPIC,
                lambda m: setattr(self, "_relmove_scan", m),
                qos_profile_sensor_data, callback_group=self.lidar_group)

        self.nav_logger(
            "info",
            f"Move_Relative -> dx={dx:.2f} dy={dy:.2f} dyaw={math.degrees(dyaw):.0f} deg"
            + (f" (lidar stop @ {stop_at_front_distance:.2f} m)" if lidar_stop else ""))
        elapsed = 0.0
        traveled = 0.0
        try:
            while elapsed < timeout:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "odom", "base_link", rclpy.time.Time(),
                        timeout=Duration(seconds=0.5))
                except Exception as e:
                    return False, f"odom TF failed mid-move: {e}", traveled
                rx, ry = tf.transform.translation.x, tf.transform.translation.y
                traveled = math.hypot(rx - x0, ry - y0)
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                if lidar_stop:
                    front = self._front_lidar_distance(self._relmove_scan)
                    if front is not None and front <= stop_at_front_distance:
                        self.nav_logger(
                            "info",
                            f"Move_Relative -> lidar stop (front={front:.2f} m, "
                            f"traveled={traveled:.2f} m)")
                        return True, "", traveled
                ex_o, ey_o = gx - rx, gy - ry
                # Remaining error expressed in the CURRENT base frame.
                ex = math.cos(yaw) * ex_o + math.sin(yaw) * ey_o
                ey = -math.sin(yaw) * ex_o + math.cos(yaw) * ey_o
                eyaw = (gyaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
                if abs(ex) <= TOL_LIN and abs(ey) <= TOL_LIN and abs(eyaw) <= TOL_ANG:
                    self.nav_logger("info", f"Move_Relative -> done (traveled={traveled:.2f} m)")
                    return True, "", traveled
                self._publish_cmd(
                    vx=max(-MAX_V, min(MAX_V, K_LIN * ex)),
                    vy=max(-MAX_V, min(MAX_V, K_LIN * ey)),
                    wz=max(-MAX_W, min(MAX_W, K_ANG * eyaw)))
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=rate))
                elapsed += rate
            self.nav_logger("warn", "Move_Relative -> timed out")
            return False, "move_relative timed out", traveled
        finally:
            self._publish_cmd()  # always leave the base stopped
            if scan_sub is not None:
                self.destroy_subscription(scan_sub)
                self._relmove_scan = None

    def _set_obstacle_layers(self, enabled):
        """Set `enabled` on the live obstacle layers (lidar obstacle_layer +
        ZED rgbd_obstacle_layer) of BOTH costmaps. The static layer is left
        untouched, so mapped walls/furniture still constrain the planner."""
        req = SetParameters.Request()
        req.parameters = [
            make_param("obstacle_layer.enabled", bool(enabled)),
            make_param("rgbd_obstacle_layer.enabled", bool(enabled)),
        ]
        all_ok = True
        for client, label in ((self.local_costmap_param_client, "local costmap"),
                              (self.global_costmap_param_client, "global costmap")):
            if not self._call_service_with_timeout(
                    client, req, TIMEOUT_RTAB_SERVICE, f"Set obstacle layers ({label})"):
                all_ok = False
        return all_ok

    def set_obstacle_avoidance_callback(self, request, response):
        """SetBool service: data=False stops marking live obstacles on both
        costmaps so an object carried by the arm (bag/basket hanging in the
        lidar/ZED view) doesn't wall the robot in while navigating; data=True
        restores marking. Costmaps are cleared afterwards so marks already
        laid down don't linger into the new mode."""
        enabled = bool(request.data)
        self.nav_logger(
            "info",
            f"Set_Obstacle_Avoidance -> {'enabling' if enabled else 'DISABLING'} live obstacle layers")
        ok = self._set_obstacle_layers(enabled)
        self._clear_costmaps()
        self._obstacle_avoidance_enabled = enabled
        response.success = ok
        response.message = (
            f"obstacle layers {'enabled' if enabled else 'disabled'}" if ok
            else "some costmap parameter services did not respond")
        return response

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

    def _grid_cell_free(self, grid, wx, wy, max_cost=50):
        """Whether world (wx, wy) lies on a known grid cell with cost < max_cost."""
        info = grid.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return False
        cost = grid.data[my * info.width + mx]
        return 0 <= cost < max_cost

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
        for radius in (standoff, standoff + 0.15, standoff + 0.30):
            for step in range(0, 19):  # 0°, then ±10° ... ±180° around the target
                for sign in ((1,) if step == 0 else (1, -1)):
                    ang = base + sign * math.radians(10 * step)
                    gx = tx + radius * math.cos(ang)
                    gy = ty + radius * math.sin(ang)
                    if self._grid_cell_free(grid, gx, gy):
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

        elapsed = 0.0
        while not self.compute_path_client.server_is_ready() and elapsed < TIMEOUT_NAV_QUERY:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
            elapsed += 0.2
        if not self.compute_path_client.server_is_ready():
            self.nav_logger("error", "Query_Path -> ComputePathToPose server not available")
            return None

        send_future = self.compute_path_client.send_goal_async(goal_msg)
        elapsed = 0.0
        while not send_future.done() and elapsed < TIMEOUT_NAV_QUERY:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not send_future.done():
            self.nav_logger("error", "Query_Path -> Timeout sending goal to planner")
            return None

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.nav_logger("error", "Query_Path -> Planner rejected the request")
            return None

        result_future = goal_handle.get_result_async()
        elapsed = 0.0
        while not result_future.done() and elapsed < TIMEOUT_NAV_QUERY:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not result_future.done():
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
        self.resume_slam()
        self.resume_nav2()
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
                self.pause_slam()
                self.pause_nav2()
        return response

    def check_for_topics(self, topics):
        topic_names_and_types = self.get_topic_names_and_types()
        active_topics = {t[0] for t in topic_names_and_types}
        topics_ready = topics.issubset(active_topics)
        return topics_ready

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
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.requirements_timeout))

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
        elapsed = 0.0
        while not self.check_for_topics(slam_topics) and elapsed < self.rtab_load_timeout:
            t.sleep(0.5)
            elapsed += 0.5
        if self.check_for_topics(slam_topics):
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
        while not rtab_client.service_is_ready():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        self.nav_logger("info", "Loading Slam -> Started loading nodes")
        while not self.check_for_topics(rtab_topics):
            req = LoadNode.Request()                                                                                                                                                                
            req.package_name = 'rtabmap_slam'
            req.plugin_name = 'rtabmap_slam::CoreWrapper'                                                                                                                                           
            req.node_name = 'rtabmap'                                                                                                                                                               
            req.parameters = rtabmap_params  # your params
            future = rtab_client.call_async(req)
            while not future.done():
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            self.nav_logger("info", "Loading Slam -> RtabCore Loaded")
            req = LoadNode.Request()
            req.package_name = 'rtabmap_sync'
            req.plugin_name = 'rtabmap_sync::RGBDSync'
            req.node_name = 'rgbd_sync'
            req.parameters = sync_params
            req.remap_rules = self.rtabmap_remapping
            future = rtab_client.call_async(req)
            while not future.done():
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            self.nav_logger("info", "Loading Slam -> RtabSync Loaded") 
            elapsed = 0.0
            while not self.check_for_topics(rtab_topics) and elapsed < self.rtab_load_timeout:
                t.sleep(0.5)
                elapsed += 0.5
            if self.check_for_topics(rtab_topics):
                self.nav_logger("info", "Loading Slam -> Topic Founded")
            else:
                self.nav_logger("error","Loading Slam -> Topics not found trying again ...")
            
        self.destroy_client(rtab_client)
        self.nav_logger("info", "Loading Slam -> Finished Slam Loading")

    def load_nav2(self):
        """Load nav2 nodes activating lifecycle"""

        self.nav_logger("info", "Loading Nav2 -> Starting nav2 lifecycle activation ...")
        while not self.lifecycle_client.service_is_ready():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        self.nav_logger("info", "Loading Nav2 -> Service found, sending STARTUP")
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        future = self.lifecycle_client.call_async(req)
        while not future.done():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self.nav_logger("info", "Loading Nav2 -> Fully loaded nav2 lifecycles")
            
    def _call_service_with_timeout(self, client, request, call_timeout, label):
        """Bounded service call — waits up to TIMEOUT_RTAB_SERVICE for readiness and
        call_timeout for the response. Returns True on success, False on timeout."""
        elapsed = 0.0
        while not client.service_is_ready() and elapsed < TIMEOUT_RTAB_SERVICE:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
            elapsed += 0.5
        if not client.service_is_ready():
            self.nav_logger("warn", f"{label} -> Service not available after {TIMEOUT_RTAB_SERVICE}s")
            return False
        future = client.call_async(request)
        elapsed = 0.0
        while not future.done() and elapsed < call_timeout:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not future.done():
            self.nav_logger("error", f"{label} -> Call timed out after {call_timeout}s, no response")
            return False
        return True

    def pause_slam(self):
        """Pause Slam function"""
        if self.use_slam_toolbox:
            # slam_toolbox self-recovers (respawn) — nothing to pause.
            return
        if not self.rtabmap_loaded:
            self.nav_logger("warn", "Pausing Slam -> Rtabmap not loaded, skipping")
            return
        self.nav_logger("info", "Pausing Slam -> Starting pause slam..")
        ok = self._call_service_with_timeout(
            self.rtabmap_pause_client, Empty.Request(),
            TIMEOUT_RTAB_SERVICE, "Pausing Slam")
        if ok:
            self.nav_logger("info", "Pausing Slam -> Finished pausing slam")

    def resume_slam(self):
        """Resuming Slam function"""
        if self.use_slam_toolbox:
            return
        if not self.rtabmap_loaded:
            self.nav_logger("warn", "Resuming Slam -> Rtabmap not loaded, skipping")
            return
        self.nav_logger("info", "Resuming Slam -> Starting resume slam..")
        ok = self._call_service_with_timeout(
            self.rtabmap_resume_client, Empty.Request(),
            TIMEOUT_RTAB_SERVICE, "Resuming Slam")
        if ok:
            self.nav_logger("info", "Resuming Slam -> Finished resuming slam")

    def pause_nav2(self):
        """Nav2 nodes pausing lifecycle"""

        if self.nav2_paused:
            return
        self.nav_logger("info", "Pausing Nav2 -> Starting nav2 lifecycle pausing ...")
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.PAUSE
        ok = self._call_service_with_timeout(
            self.lifecycle_client, req, TIMEOUT_NAV2_LIFECYCLE, "Pausing Nav2")
        if ok:
            self.nav2_paused = True
            self.nav_logger("info", "Pausing Nav2 -> Fully Paused nav2 lifecycles")

    def resume_nav2(self):
        """Nav2 nodes resume lifecycle"""

        if not self.nav2_paused:
            return
        self.nav_logger("info", "Resume Nav2 -> Starting nav2 lifecycle resume ...")
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.RESUME
        ok = self._call_service_with_timeout(
            self.lifecycle_client, req, TIMEOUT_NAV2_LIFECYCLE, "Resume Nav2")
        if ok:
            self.nav2_paused = False
            self.nav_logger("info", "Resume Nav2 -> Fully resumed nav2 lifecycles")

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

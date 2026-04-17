#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.action import NavigateToPose  
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.srv import GetMap
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
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
        MOVE_DISTANCE_SERVICE,
        MOVE_UNTIL_OBJECT_SERVICE,
        MOVE_LOCATION_IGNORE_OBSTACLES_SERVICE,
        CMD_VEL_TOPIC,
        BASE_FRAME,
        ODOM_FRAME,
        MAP_FRAME,
        TIMEOUT_MOVE_DISTANCE_TF,
        MOVE_DISTANCE,
        MOVE_UNTIL_OBJECT,
        SCAN_MASKED_TOPIC,
        SCAN_MASK,
        LOCAL_COSTMAP_NODE,
        GLOBAL_COSTMAP_NODE,
        CAMERA_OBSTACLE_LAYER,
        GOAL_NAV_ACTION_SERVER,
        INITIAL_POSE_TOPIC,
        RESUME_NAV_SERVICE,
        )
from frida_interfaces.srv import (
        CheckDoor,
        MapAreas,
        MoveLocation,
        MoveDistance,
        MoveUntilObject,
        MoveLocationIgnoreObstacles,
        )
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterValue as RclParameterValue
from rcl_interfaces.msg import ParameterType as RclParameterType
from ament_index_python.packages import get_package_share_directory
import tf2_ros
import json
import time as t
import math
import yaml
import re



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
        self.map_name= self.declare_parameter('map_name', 'rtabmap_map.db').value
        self.mapping_config = self.declare_parameter('rtab_mapping_config', '').value
        self.localization_config = self.declare_parameter('rtab_localization_config', '').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None 
        self.required_topics = {'/zed/zed_node/rgb/camera_info', '/cmd_vel','/scan'}
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
        self.move_distance_srv = self.create_service(MoveDistance, MOVE_DISTANCE_SERVICE, self.move_distance, callback_group=self.service_group)
        self.move_until_object_srv = self.create_service(MoveUntilObject, MOVE_UNTIL_OBJECT_SERVICE, self.move_until_object, callback_group=self.service_group)
        self.move_location_ignore_obstacles_srv = self.create_service(MoveLocationIgnoreObstacles, MOVE_LOCATION_IGNORE_OBSTACLES_SERVICE, self.move_location_ignore_obstacles, callback_group=self.service_group)

        # Cmd vel publisher for open-loop motion
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # Persistent scan masker: subscribes /scan, publishes /scan_masked.
        # When _scan_mask_active is False it is a pure pass-through.
        self._scan_mask_active = False
        self._latest_scan = None
        self._scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self._scan_callback, 10, callback_group=self.lidar_group)
        self._scan_masked_pub = self.create_publisher(LaserScan, SCAN_MASKED_TOPIC, 10)

        # Nav2 costmap param clients (created lazily before first use)
        self._local_costmap_param_client = None
        self._global_costmap_param_client = None

        self.goal_action_client = ActionClient(self,NavigateToPose ,GOAL_NAV_ACTION_SERVER)

        # Manual resume service — lets the UI unpause nav2 + RTABMap on demand
        self.resume_nav_srv = self.create_service(
            Empty, RESUME_NAV_SERVICE, self._resume_nav_callback,
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
        if not self.mapping:
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
        if not self.mapping:
            self.nav_logger("info", "Slam completed, Starting nav2 ...")
            self.load_nav2()
            self.nav_logger("info", "Nav2 completed")
            self._wait_for_initial_pose()
        else:
            self.nav_logger("info", "Mapping mode: nav2 skipped")
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

        # Check if rtabmap crashed (was loaded but topic disappeared)
        if self.rtabmap_loaded and not self.check_for_topics({RTAB_CHECK_TOPIC}):
            self.rtabmap_loaded = False
            self.nav_logger("warn", "Monitor -> Rtabmap container crashed, will reload when requirements are met")

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
            if not self.mapping:
                self.pause_nav2()
        elif (self.no_topics_count == 0) and (self.no_tf_count == 0):
            if self.nodes_status == False:
                self.nodes_status = True
                self.nav_logger("info", "Monitor -> Requirements available, Activating nodes ...")
                self.resume_slam()
                if not self.mapping:
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
        if not self.mapping:
            self.resume_nav2()
        self.nodes_status = True
        self.no_topics_count = 0
        self.no_tf_count = 0
        return response

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

        self.lidar_reciever = self.create_subscription(LaserScan,SCAN_TOPIC, self.lidar_callback, 10,callback_group=self.lidar_group )
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
            door_points = []
            inf_count = 0
            point_count = 0

            for count, r in enumerate(self.lidar_msg.ranges):
                if self.range_min > self.range_max:
                    if (count <= self.range_max and count >= 0 ) or (count >= self.range_min):
                        door_points.append(r)
                        if(math.isinf(r)):
                            inf_count += 1
                        point_count += 1
                elif self.range_min <= count <= self.range_max:
                    door_points.append(r)

            if inf_count > 0: #Filter only if there is points
                if inf_count < count / 3: #FIlter noise inf, only if is above 1/3 of the points
                    door_points = [x for x in door_points if not math.isinf(x)]  
                avg_points = sum(door_points)/ len(door_points)

                if(avg_points > self.door_distance):
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

    def send_nav_goal(self, pose, behaivor_tree = None):
        """Function to send goal to nav2 bt"""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        if behaivor_tree is not None:
            goal_msg.behaivor_tree = behaivor_tree
        self.goal_action_client.wait_for_server()

        _goal_future = self.goal_action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback)

        while not _goal_future.done():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        goal_handle = _goal_future.result()
        if goal_handle.accepted:
              result_future = goal_handle.get_result_async()                                                                                                                                                   
              while not result_future.done():
                  self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))                                                                                                                             
              result = result_future.result()
              self.nav_logger("info", f"Goal_Handler -> Goal Reached")
              return (True, "Goal Finished")
        else:
            return (False, "Goal Rejected")

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
        
        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            self.nav_logger("error", "Go_To_Area -> Navigation not initialized")
            response.success = False
            response.error = "Navigation not initialized"
            return response 
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

    # ------------------------------------------------------------------
    #   Scan masker helpers (publishes /scan_masked from /scan)
    # ------------------------------------------------------------------
    def _scan_callback(self, msg):
        """Cache latest scan and republish (optionally masked) on /scan_masked."""
        self._latest_scan = msg

        if not self._scan_mask_active:
            # Passthrough. Cheap because LaserScan is a shallow struct.
            self._scan_masked_pub.publish(msg)
            return

        # Build a masked copy: any ray whose angle falls inside the rear
        # window gets its range set to infinity (ignored by costmap).
        center = math.radians(SCAN_MASK.REAR_CENTER_DEG.value)
        half = math.radians(SCAN_MASK.REAR_HALFWIDTH_DEG.value)
        low = center - half
        high = center + half

        masked = LaserScan()
        masked.header = msg.header
        masked.angle_min = msg.angle_min
        masked.angle_max = msg.angle_max
        masked.angle_increment = msg.angle_increment
        masked.time_increment = msg.time_increment
        masked.scan_time = msg.scan_time
        masked.range_min = msg.range_min
        masked.range_max = msg.range_max
        masked.intensities = msg.intensities

        def _wrap(a):
            # Wrap to [-pi, pi]
            return math.atan2(math.sin(a), math.cos(a))

        new_ranges = list(msg.ranges)
        wrapped_low = _wrap(low)
        wrapped_high = _wrap(high)
        for i in range(len(new_ranges)):
            ang = _wrap(msg.angle_min + i * msg.angle_increment)
            if wrapped_low <= wrapped_high:
                in_window = wrapped_low <= ang <= wrapped_high
            else:
                # window wraps around -pi/pi
                in_window = ang >= wrapped_low or ang <= wrapped_high
            if in_window:
                new_ranges[i] = float("inf")
        masked.ranges = new_ranges
        self._scan_masked_pub.publish(masked)

    # ------------------------------------------------------------------
    #   Open-loop base motion helpers
    # ------------------------------------------------------------------
    def _lookup_base_in_odom(self):
        """Return (x, y) of base_link in odom frame, or None on failure."""
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < Duration(seconds=TIMEOUT_MOVE_DISTANCE_TF):
            try:
                tf = self.tf_buffer.lookup_transform(
                    ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
                return (tf.transform.translation.x, tf.transform.translation.y)
            except Exception:
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        return None

    def _wait_for_scan(self, timeout_sec):
        """Block until at least one LaserScan has been received."""
        start = self.get_clock().now()
        while self._latest_scan is None and (
            (self.get_clock().now() - start) < Duration(seconds=timeout_sec)):
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        return self._latest_scan is not None

    def _min_range_in_window(self, scan, center_deg, halfwidth_deg):
        """Return min finite range in [center-half, center+half] degrees.

        Angle 0 is assumed to be the robot's forward direction. Wrapping
        around ±pi is handled.
        """
        if scan is None or len(scan.ranges) == 0:
            return float("inf")
        low = math.radians(center_deg - halfwidth_deg)
        high = math.radians(center_deg + halfwidth_deg)
        # Wrap to [-pi, pi]
        def _wrap(a):
            return math.atan2(math.sin(a), math.cos(a))
        wl, wh = _wrap(low), _wrap(high)
        best = float("inf")
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min:
                continue
            ang = _wrap(scan.angle_min + i * scan.angle_increment)
            if wl <= wh:
                in_window = wl <= ang <= wh
            else:
                in_window = ang >= wl or ang <= wh
            if in_window and r < best:
                best = r
        return best

    # ------------------------------------------------------------------
    #   Service: move a fixed distance (open-loop, no obstacle checks)
    # ------------------------------------------------------------------
    def move_distance(self, request, response):
        """Publishes /cmd_vel and integrates distance from odom TF. Ignores
        obstacles entirely — the caller is responsible for path clearance."""

        distance = float(request.distance)
        self.nav_logger("info", f"Move_Distance -> Moving {distance:.3f} m (open-loop)")

        start_xy = self._lookup_base_in_odom()
        if start_xy is None:
            self.nav_logger("error", "Move_Distance -> odom->base_link TF not available")
            response.success = False
            response.error = "TF odom->base_link not available"
            return response

        target = abs(distance)
        if target < MOVE_DISTANCE.DISTANCE_TOLERANCE.value:
            response.success = True
            response.error = ""
            return response

        direction = 1.0 if distance >= 0.0 else -1.0
        speed = MOVE_DISTANCE.LINEAR_SPEED.value
        period = 1.0 / MOVE_DISTANCE.CONTROL_RATE.value
        tolerance = MOVE_DISTANCE.DISTANCE_TOLERANCE.value
        deadline = Duration(seconds=(target / speed) + MOVE_DISTANCE.MAX_EXTRA_TIME.value)

        twist = Twist()
        twist.linear.x = direction * speed

        loop_start = self.get_clock().now()
        travelled = 0.0
        while (self.get_clock().now() - loop_start) < deadline:
            cur = self._lookup_base_in_odom()
            if cur is None:
                self.nav_logger("warn", "Move_Distance -> Lost odom TF during motion")
                break
            travelled = math.hypot(cur[0] - start_xy[0], cur[1] - start_xy[1])
            if (target - travelled) <= tolerance:
                break
            self.cmd_vel_pub.publish(twist)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=period))

        # Stop
        self.cmd_vel_pub.publish(Twist())

        if (target - travelled) > tolerance:
            msg = f"Timed out after {travelled:.3f} m of {target:.3f} m"
            self.nav_logger("error", f"Move_Distance -> {msg}")
            response.success = False
            response.error = msg
            return response

        self.nav_logger("info", f"Move_Distance -> Completed, travelled {travelled:.3f} m")
        response.success = True
        response.error = ""
        return response

    # ------------------------------------------------------------------
    #   Service: drive until an obstacle is detected by the lidar
    # ------------------------------------------------------------------
    def move_until_object(self, request, response):
        """Drive open-loop at low speed until the lidar sector of interest
        reports a range <= stop_distance. Ignores camera and any other sensors.
        """
        forward = bool(request.forward)
        stop_distance = float(request.stop_distance)
        if stop_distance <= 0.0:
            stop_distance = MOVE_UNTIL_OBJECT.DEFAULT_STOP_DISTANCE.value

        self.nav_logger(
            "info",
            f"Move_Until_Object -> {'forward' if forward else 'backward'} until "
            f"object within {stop_distance:.2f} m")

        if not self._wait_for_scan(MOVE_UNTIL_OBJECT.LIDAR_TIMEOUT.value):
            self.nav_logger("error", "Move_Until_Object -> No lidar data available")
            response.success = False
            response.measured_distance = 0.0
            response.error = "No lidar data"
            return response

        start_xy = self._lookup_base_in_odom()
        if start_xy is None:
            self.nav_logger("error", "Move_Until_Object -> odom->base_link TF missing")
            response.success = False
            response.measured_distance = 0.0
            response.error = "TF odom->base_link not available"
            return response

        if forward:
            center = MOVE_UNTIL_OBJECT.FRONT_ANGLE_CENTER_DEG.value
            half = MOVE_UNTIL_OBJECT.FRONT_ANGLE_HALFWIDTH_DEG.value
            direction = 1.0
        else:
            center = MOVE_UNTIL_OBJECT.REAR_ANGLE_CENTER_DEG.value
            half = MOVE_UNTIL_OBJECT.REAR_ANGLE_HALFWIDTH_DEG.value
            direction = -1.0

        speed = MOVE_UNTIL_OBJECT.LINEAR_SPEED.value
        period = 1.0 / MOVE_UNTIL_OBJECT.CONTROL_RATE.value
        deadline = Duration(seconds=MOVE_UNTIL_OBJECT.MAX_TRAVEL_TIME.value)

        twist = Twist()
        twist.linear.x = direction * speed

        loop_start = self.get_clock().now()
        travelled = 0.0
        last_min = float("inf")

        while (self.get_clock().now() - loop_start) < deadline:
            scan = self._latest_scan
            last_min = self._min_range_in_window(scan, center, half)
            if last_min <= stop_distance:
                break

            cur = self._lookup_base_in_odom()
            if cur is not None:
                travelled = math.hypot(cur[0] - start_xy[0], cur[1] - start_xy[1])
            self.cmd_vel_pub.publish(twist)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=period))

        self.cmd_vel_pub.publish(Twist())

        if last_min > stop_distance:
            msg = f"Timed out. Nearest: {last_min:.3f} m; travelled {travelled:.3f} m"
            self.nav_logger("error", f"Move_Until_Object -> {msg}")
            response.success = False
            response.measured_distance = travelled
            response.error = msg
            return response

        self.nav_logger(
            "info",
            f"Move_Until_Object -> Stopped. Object at {last_min:.3f} m, "
            f"travelled {travelled:.3f} m")
        response.success = True
        response.measured_distance = travelled
        response.error = ""
        return response

    # ------------------------------------------------------------------
    #   Nav2 costmap parameter helpers
    # ------------------------------------------------------------------
    def _ensure_costmap_param_clients(self):
        if self._local_costmap_param_client is None:
            self._local_costmap_param_client = self.create_client(
                SetParameters,
                f"{LOCAL_COSTMAP_NODE}/set_parameters",
                callback_group=self.service_group)
        if self._global_costmap_param_client is None:
            self._global_costmap_param_client = self.create_client(
                SetParameters,
                f"{GLOBAL_COSTMAP_NODE}/set_parameters",
                callback_group=self.service_group)

    def _set_layer_enabled(self, client, layer_name, enabled, label):
        """Set {layer_name}.enabled on the given costmap's parameter server."""
        if not client.wait_for_service(timeout_sec=TIMEOUT_RTAB_SERVICE):
            self.nav_logger("warn", f"{label} -> param service not ready for {layer_name}")
            return False
        req = SetParameters.Request()
        p = RclParameter()
        p.name = f"{layer_name}.enabled"
        p.value = RclParameterValue(
            type=RclParameterType.PARAMETER_BOOL, bool_value=bool(enabled))
        req.parameters = [p]
        future = client.call_async(req)
        elapsed = 0.0
        while not future.done() and elapsed < TIMEOUT_RTAB_SERVICE:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not future.done():
            self.nav_logger("error", f"{label} -> SetParameters timed out")
            return False
        try:
            result = future.result()
            ok = result is not None and all(r.successful for r in result.results)
        except Exception:
            ok = False
        if not ok:
            self.nav_logger("error", f"{label} -> failed to set {layer_name}.enabled={enabled}")
        return ok

    # ------------------------------------------------------------------
    #   Service: go to a map area with obstacle sources temporarily muted
    # ------------------------------------------------------------------
    def move_location_ignore_obstacles(self, request, response):
        """Sends a Nav2 goal with the camera obstacle layer and / or the rear
        sector of the lidar temporarily suppressed. Previous values are
        always restored, even on failure."""

        self.nav_logger(
            "info",
            f"Move_Location_Ignore -> {request.location}/{request.sublocation} "
            f"(ignore_camera={request.ignore_camera}, "
            f"ignore_rear_lidar={request.ignore_rear_lidar})")

        if self.areas_data is None:
            response.success = False
            response.error = "Areas not loaded"
            return response

        fetch_coords = self.areas_data.get(
            request.location, {}).get(
                request.sublocation or "safe_place")
        if fetch_coords is None:
            response.success = False
            response.error = "Area not found"
            return response

        # --- apply overrides ---------------------------------------------
        self._ensure_costmap_param_clients()
        camera_changed = False
        rear_mask_changed = False

        if request.ignore_camera:
            a = self._set_layer_enabled(
                self._local_costmap_param_client,
                CAMERA_OBSTACLE_LAYER, False, "Ignore_Obstacles(local_cam)")
            b = self._set_layer_enabled(
                self._global_costmap_param_client,
                CAMERA_OBSTACLE_LAYER, False, "Ignore_Obstacles(global_cam)")
            camera_changed = a and b
            if not camera_changed:
                self.nav_logger("warn", "Move_Location_Ignore -> camera disable partially failed")

        if request.ignore_rear_lidar:
            self._scan_mask_active = True
            rear_mask_changed = True
            self.nav_logger("info", "Move_Location_Ignore -> rear scan mask ON")

        # --- build & send goal -------------------------------------------
        self.resume_slam()
        self.resume_nav2()
        if self.nav2_paused or not self.rtabmap_loaded:
            self._restore_obstacle_sources(camera_changed, rear_mask_changed)
            response.success = False
            response.error = "Navigation not initialized"
            return response

        goal_coord = PoseStamped()
        goal_coord.header.frame_id = MAP_FRAME
        goal_coord.header.stamp = self.get_clock().now().to_msg()
        goal_coord.pose.position.x = fetch_coords[0]
        goal_coord.pose.position.y = fetch_coords[1]
        goal_coord.pose.position.z = fetch_coords[2]
        goal_coord.pose.orientation.x = fetch_coords[3]
        goal_coord.pose.orientation.y = fetch_coords[4]
        goal_coord.pose.orientation.z = fetch_coords[5]
        goal_coord.pose.orientation.w = fetch_coords[6]

        try:
            result = self.send_nav_goal(goal_coord)
            response.success = result[0]
            response.error = result[1]
        finally:
            self._restore_obstacle_sources(camera_changed, rear_mask_changed)
            self.pause_slam()
            self.pause_nav2()

        return response

    def _restore_obstacle_sources(self, camera_changed, rear_mask_changed):
        """Undo any obstacle-source overrides applied during the goal."""
        if rear_mask_changed:
            self._scan_mask_active = False
            self.nav_logger("info", "Move_Location_Ignore -> rear scan mask OFF")
        if camera_changed:
            self._set_layer_enabled(
                self._local_costmap_param_client,
                CAMERA_OBSTACLE_LAYER, True, "Ignore_Obstacles(local_cam_restore)")
            self._set_layer_enabled(
                self._global_costmap_param_client,
                CAMERA_OBSTACLE_LAYER, True, "Ignore_Obstacles(global_cam_restore)")

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

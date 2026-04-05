#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.srv import ManageLifecycleNodes
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.srv import GetMap
from std_srvs.srv import Empty
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
        CHECK_DOOR_SERVICE,
        DOOR_CHECK,
        TIMEOUT_REQUIREMENTS,
        CAMERA_RGB_TOPIC,
        CAMERA_INFO_TOPIC,
        CAMERA_DEPTH_TOPIC,
        TIMEOUT_RTABMAP,
        RTAB_PAUSE_SERVICE,
        RTAB_RESUME_SERVICE,
        NAV2_LIFECYCLE_SERVICE,
        RTAB_CHECK_TOPIC,
        RTAB_MAPS_PATH,
        RTAB_CONTAINER_NODE
        ) 
from frida_interfaces.srv import (
        CheckDoor
        )
import tf2_ros
import time as t
import math
import yaml



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


        self.lidar_group = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.lidar_msg = None
        self.lidar_reciever = None
        self.check_door_srv = self.create_service(CheckDoor, CHECK_DOOR_SERVICE, self.check_door, callback_group=self.service_group)
        self.range_min = DOOR_CHECK.LIDAR_RANGE_MIN.value  
        self.range_max = DOOR_CHECK.LIDAR_RANGE_MAX.value
        self.door_rate = DOOR_CHECK.CHECKING_RATE.value
        self.door_distance = DOOR_CHECK.DOOR_DISTANCE.value 
        self.sensor_timeout = Duration(seconds=DOOR_CHECK.TIMEOUT_SENSOR.value) # Timeout in seconds to wait for sensors
        self.door_timeout = Duration(seconds=DOOR_CHECK.TIMEOUT_TO_OPEN.value) # Timeout in seconds to wait for sensors
        
        #Setup and Configuration
        self._setup_done = False
        self._setup_timer = self.create_timer(2.0, self._setup, callback_group=ReentrantCallbackGroup())
    
    def _setup(self):
        if self._setup_done:
            self.nav_logger("info", "Out of papu ")
            return
        self._setup_done = True
        self.destroy_timer(self._setup_timer)
        self.nav_logger("info", "Starting Setup, waiting for requirements ...")
        self.wait_for_requirements()
        self.nav_logger("info", "Requirements Completed, Starting Slam ...")
        self.start_slam()
        self.nav_logger("info", "Slam completed, Starting nav2 ...")
        self.load_nav2()
        self.nav_logger("info", "Nav2 completed")
        self.nav_logger("info", "Finished Setup, Starting monitoring ...")
    
    def monitoring(self):
       self.nav_logger("info", "Starting monitoring") 

    def nav_logger(self,status, data):
        if status == "info":
            self.get_logger().info(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;119m {data}\033[0m")
        elif status == "warn":
            self.get_logger().warn(f"\033[35m\033[1mNav_Control: \033[22m\033[33m {data}\033[0m")
        elif status == "error":
            self.get_logger().error(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;167m {data}\033[0m")
        else:
            self.get_logger().fatal(f"\033[35m\033[1mNav_Control: \033[22m\033[38;5;88m {data}\033[0m")

    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def check_door(self,request, response):
        self.get_logger().info("Check_door: Service called")

        self.lidar_reciever = self.create_subscription(LaserScan,SCAN_TOPIC, self.lidar_callback, 10,callback_group=self.lidar_group )
        self.lidar_msg = None  #Clean for cache msgs
        t.sleep(self.door_rate) #Wait for suscription to start

        start_time = self.get_clock().now()
        while self.lidar_msg is None and (self.get_clock().now() - start_time) < self.sensor_timeout:
            self.get_logger().warn("Check_door: waiting for lidar msg...")
            t.sleep(self.door_rate)

        if self.lidar_msg is None:
            self.destroy_subscription(self.lidar_reciever)
            self.lidar_msg = None
            self.get_logger().error("Check_door: Timeout reached lidar failed to retreive")
            response.status = False
            return response

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < self.door_timeout: #Timeout in case of absolute failure 
            self.get_logger().info("Check_door: Waiting for door to open")
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
                    self.get_logger().info("Door opened")
                    self.destroy_subscription(self.lidar_reciever)
                    self.lidar_msg = None
                    response.status = True
                    return response

            t.sleep(self.door_rate)

        self.destroy_subscription(self.lidar_reciever)
        self.lidar_msg = None
        self.get_logger().error("Check_door: Timeout reached door didnt opened")
        response.status = False
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
        load_cb_group = ReentrantCallbackGroup()
        lifecycle_client = self.create_client(                                                                                                                                                  
              ManageLifecycleNodes,                                                                                                                                                               
              NAV2_LIFECYCLE_SERVICE,
              callback_group=load_cb_group
        )
        lifecycle_client.wait_for_service()
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP                                                                                                                                      
        future = lifecycle_client.call_async(req)
        while not future.done():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self.destroy_client(lifecycle_client)
        self.nav_logger("info", "Loading Nav2 -> Fully loaded nav2 lifecycles") 
            
    def pause_slam(self):
        """Pause Slam function"""

        self.nav_logger("info" "Pausing Slam -> Starting pause slam..")
        load_cb_group = ReentrantCallbackGroup()
        rtabmap_pause = self.node.create_client(Empty, RTAB_PAUSE_SERVICE, callback_group=load_cb_group)
        rtabmap_pause.wait_for_service()
        req = Empty.Request()
        future = rtabmap_pause.call_async(req)       
        while not future.done():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self.destroy_client(rtabmap_pause)
        self.nav_logger("info", "Pausing Slam -> Finished pausing slam")

    def resume_slam(self):
        """Resuming Slam function"""

        self.nav_logger("info" "Resuming Slam -> Starting pause slam..")
        load_cb_group = ReentrantCallbackGroup()
        rtabmap_resume= self.node.create_client(Empty, RTAB_RESUME_SERVICE , callback_group=load_cb_group)
        rtabmap_resume.wait_for_service()
        req = Empty.Request()
        future = rtabmap_resume.call_async(req)       
        while not future.done():
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self.destroy_client(rtabmap_resume)
        self.nav_logger("info", "Resuming Slam -> Finished pausing slam")

    

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()

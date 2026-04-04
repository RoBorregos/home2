#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.srv import GetMap
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
        CHECK_DOOR_SERVICE,
        DOOR_CHECK,
        TIMEOUT_REQUIREMENTS
        ) 
from frida_interfaces.srv import (
        CheckDoor
        )
import tf2_ros
import time as t
import math
import sys

class Nav_Central(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self.get_logger.info("Starting Nav_Central ... ")
        self.localization = self.declare_parameter('localization', False).value
        self.map_name= self.declare_parameter('map_name', 'rtabmap_map.db').value
        self.localization_config = self.declare_parameter('rtab_mapping_config', '').value
        self.mapping_config = self.declare_parameter('rtab_localization_config', '').value

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
        self.get_logger.info("Checking Requirements ....")
        self.wait_for_requirements()
        self.get_logger.info("Requirements obtained")
        self.get_logger.info("Loading Rtabmap")
        self.start_slam()
        self.get_logger.info("Finish")
    
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

    def check_for_topics(self, topics):
        topic_names_and_types = self.get_topic_names_and_types()
        active_topics = {t[0] for t in topic_names_and_types}
        topics_ready = self.topics.issubset(active_topics)
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
            else:
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.requirements_timeout))

    def start_slam(self):
        """Function to load slam nodes"""                                                                                                                                                                                        
        # Load a node into the container

        rtabmap_params = params_from_yaml(self.config_path, 'rtabmap')
        db_path = f'/workspace/src/navigation/rtabmapdbs/{self.map_name}'
        rtabmap_params.append(make_param('database_path', db_path))
        sync_params = params_from_yaml(config_path, 'rgbd_sync')

        rtab_topics = {'/rtabmap/republish_node_data'}

        rtab_client = self.create_client(LoadNode, '/rtabmap_container/_container/load_node')                                                                                                        
        sync_client = self.create_client(LoadNode, '/rtabmap_container/_container/load_node')                                                                                                        
        self.client.wait_for_service()
        while not self.check_for_topics(rtab_topics):
            req = LoadNode.Request()                                                                                                                                                                
            req.package_name = 'rtabmap_slam'
            req.plugin_name = 'rtabmap_slam::CoreWrapper'                                                                                                                                           
            req.node_name = 'rtabmap'                                                                                                                                                               
            req.parameters = rtabmap_params  # your params
            future = rtab_client.call_async(req)
            rclpy.spin_until_future_complete(self,future)
            
            req = LoadNode.Request()                                                                                                                                                                
            req.package_name = 'rtabmap_sync'
            req.plugin_name = 'rtabmap_sync::RGBDSync'                                                                                                                                           
            req.node_name = 'rgbd_sync'                                                                                                                                                               
            req.parameters = sync_params  # your params
            future = sync_client.call_async(req)
            rclpy.spin_until_future_complete(self,future)
            service_check = self.create_client(GetMap, 'rtabmap_msgs/srv/GetMap') 
            service_check.wait_for_service(timeout_sec=self.rtab_load_timeout)
                
            
        self.destroy_client(rtab_client)
        self.destroy_client(sync_client)
    
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

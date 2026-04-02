#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition as MsgTransition
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import LaserScan
from frida_constants.navigation_constants import(
        SCAN_TOPIC,
        CHECK_DOOR_SERVICE
        ) 
from frida_interfaces.srv import (
        LaserGet
        )
import tf2_ros
import time as t
import math
import sys

TIMEOUT_SENSOR = 5.0

class NavDependencyLifecycleManager(LifecycleNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None 
        
        self.required_topics = {'/zed/zed_node/rgb/camera_info'}
        self.required_frames = {'base_link'}
        
        self.ready_to_activate = False
        self.timer = None
       
        self.lidar_group = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.lidar_reciever = self.create_subscription(LaserScan,SCAN_TOPIC, self.lidar_callback, 10,callback_group=self.lidar_group )
        self.lidar_msg = None
        self.check_door_srv = self.create_service(LaserGet, CHECK_DOOR_SERVICE, self.check_door, callback_group=self.service_group)
        self.range_min = 670
        self.range_max = 70
        self.door_rate = 0.5
        self.door_distance = 0.6
        self.sensor_timeout = Duration(seconds=TIMEOUT_SENSOR) # Timeout in seconds to wait for sensors

        self.declare_parameter('autostart', True)
        self.declare_parameter('managed_nodes', [''])
        self.managed_nodes = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        if self.managed_nodes == ['']:
            self.managed_nodes = []

    def lidar_callback(self, msg):
        self.lidar_msg = msg

    def check_door(self,request, response):
        self.get_logger().info("Check_door: Service called")
        last_time = self.get_clock().now()
        while self.lidar_msg is None and (self.get_clock().now() - last_time) < self.sensor_timeout:
            self.get_logger().warn("Check_door: Lidar msg not found retrying ...")
            t.sleep(self.doot_rate)
        if self.lidar_msg is None:
            self.get_logger().error("Check_door: Timeout reached lidar failed to retreive")
            response.status = False
            return response
        opened = False
        while opened == False:
            self.get_logger().info("Check_door: Waiting for door to open")
            t.sleep(self.door_rate)
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
                    response.status = True
                    return response
            
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando: Iniciando monitoreo autónomo de dependencias")
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        if self.timer is not None:
            self.timer.cancel()

        self.timer = self.create_timer(2.0, self.monitor_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if not self.ready_to_activate:
            self.get_logger().warn("Intento de activación sin dependencias listas")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Sistema ACTIVO: Gestionando nodos dependientes")

        # Transition managed nodes: configure then activate
        for node_name in self.managed_nodes:
            self.transition_node(node_name, MsgTransition.TRANSITION_CONFIGURE)
            self.transition_node(node_name, MsgTransition.TRANSITION_ACTIVATE)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando: Deteniendo servicios dependientes")
        
        # Optionally deactivate managed nodes
        for node_name in self.managed_nodes:
            self.transition_node(node_name, MsgTransition.TRANSITION_DEACTIVATE)
            
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    def monitor_callback(self):
        """Callback autónomo para monitoreo de dependencias."""
        if self.ready_to_activate:
            return

        topic_names_and_types = self.get_topic_names_and_types()
        active_topics = {t[0] for t in topic_names_and_types}
        topics_ready = self.required_topics.issubset(active_topics)
        
        try:
            frames_dict = self.tf_buffer.all_frames_as_yaml()
            tf_ready = all(frame in frames_dict for frame in self.required_frames)
        except Exception:
            tf_ready = False
        
        if topics_ready and tf_ready:
            self.get_logger().info("Dependencias encontradas. Transicionando a ACTIVE...")
            self.ready_to_activate = True
            # Transición interna autónoma (sin llamar a servicio externo)
            self.trigger_activate()
        else:
            missing_topics = self.required_topics - active_topics
            self.get_logger().info(f"Esperando dependencias: Tópicos {list(missing_topics)} | TF: {tf_ready}", once=True)

    def transition_node(self, node_name, transition_id):
        """Helper para transicionar un nodo externo via servicio."""
        client = self.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Servicio de ciclo de vida para {node_name} no disponible")
            return

        req = ChangeState.Request()
        req.transition.id = transition_id
        client.call_async(req)
        self.get_logger().info(f"Transición {transition_id} enviada a {node_name}")

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = NavDependencyLifecycleManager('nav_lifecycle_manager')
    executor.add_node(node)
    
    autostart = node.get_parameter('autostart').get_parameter_value().bool_value
    if autostart:
        node.get_logger().info("Autostart habilitado")
        node.trigger_configure()
    
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

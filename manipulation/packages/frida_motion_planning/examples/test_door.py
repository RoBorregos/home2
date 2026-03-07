#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker 
from frida_interfaces.action import MoveToPose
from frida_constants.manipulation_constants import MOVE_TO_POSE_ACTION_SERVER
import math
import time

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class DoorOpenerVertical(Node):
    def __init__(self):
        super().__init__('door_opener_vertical')
        self.move_to_pose_client = ActionClient(self, MoveToPose, MOVE_TO_POSE_ACTION_SERVER)
        
        self.marker_pub = self.create_publisher(Marker, '/door_handle_marker', 10)

        # Coordenadas exactas de la manija de la puerta
        self.handle_x = 0.45
        self.handle_y = 0.0
        self.handle_z = 0.60 

        # Orientación calibrada: De frente y vertical
        self.base_roll = math.pi / 2 
        self.base_pitch = -math.pi / 4
        self.base_yaw = math.pi / 2
        self.handle_orientation = quaternion_from_euler(self.base_roll, self.base_pitch, self.base_yaw)

    def publish_target_marker(self):
        """Publica una esfera roja en RViz en la posición de la manija"""
        marker = Marker()
        marker.header.frame_id = "link_base" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "door_handle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.handle_x
        marker.pose.position.y = self.handle_y
        marker.pose.position.z = self.handle_z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 
        
        self.marker_pub.publish(marker)
        self.get_logger().info(f'Marcador visual publicado en Rviz en X={self.handle_x}, Y={self.handle_y}, Z={self.handle_z}')

    def send_pose_goal(self, pose: Pose, phase_name: str, is_cartesian: bool = False):
        self.get_logger().info(f'--- INICIANDO {phase_name} ---')
        self.move_to_pose_client.wait_for_server()

        goal_msg = MoveToPose.Goal()
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'link_base'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        goal_msg.pose = pose_stamped
        
        goal_msg.target_link = 'link_eef'  
        goal_msg.velocity = 0.3
        goal_msg.acceleration = 0.3
        goal_msg.planning_time = 2.0
        goal_msg.planning_attempts = 5
        
        if is_cartesian:
            goal_msg.planner_id = 'cartesian'

        send_goal_future = self.move_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{phase_name} rechazada por el servidor.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'{phase_name} Completada.\n')
        return result_future.result().result

    def execute_sequence(self):
        # 1. Mostrar la meta en Rviz
        time.sleep(1.0) 
        self.publish_target_marker()

    
        # FASE A: PRE-GRASP (15 cm detrás de la esfera roja)
    
        pose_a = Pose()
        pose_a.position = Point(x=self.handle_x - 0.15, y=self.handle_y, z=self.handle_z)
        pose_a.orientation = self.handle_orientation
        self.send_pose_goal(pose_a, "FASE A: PRE-GRASP")

    
        # FASE B: GRASP (Avanzar para "comerse" la esfera)
    
        pose_b = Pose()
        pose_b.position = Point(x=self.handle_x, y=self.handle_y, z=self.handle_z)
        pose_b.orientation = self.handle_orientation
        self.send_pose_goal(pose_b, "FASE B: GRASP (Insercion recta)", is_cartesian=True)

    
        # FASE C: PUSH DOWN (Bajar recto hacia el piso 8 cm)
    
        pose_c = Pose()
        # Restamos 0.08m al eje Z actual.
        pose_c.position = Point(x=self.handle_x, y=self.handle_y, z=self.handle_z - 0.08)
        pose_c.orientation = self.handle_orientation
        self.send_pose_goal(pose_c, "FASE C: PUSH DOWN (Bajar manija en Z)", is_cartesian=True)

    
        # # FASE D: PULL OPEN (Jalar la puerta retrocediendo en arco)
        
        # pose_d = Pose()
        # # Retrocedemos 25cm en X, nos movemos 15cm en Y, y MANTENEMOS la Z baja.
        # pose_d.position = Point(x=self.handle_x - 0.25, y=self.handle_y + 0.15, z=self.handle_z - 0.08)
        # pose_d.orientation = self.handle_orientation
        # self.send_pose_goal(pose_d, "FASE D: PULL OPEN (Abrir puerta)", is_cartesian=True)


def main(args=None):
    rclpy.init(args=args)
    node = DoorOpenerVertical()
    node.execute_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
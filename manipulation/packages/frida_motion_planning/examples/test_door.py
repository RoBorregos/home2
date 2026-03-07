#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from frida_interfaces.action import MoveToPose
from frida_constants.manipulation_constants import MOVE_TO_POSE_ACTION_SERVER
import math

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

class DoorOpenerFull(Node):
    def __init__(self):
        super().__init__('door_opener_full')
        self.move_to_pose_client = ActionClient(self, MoveToPose, MOVE_TO_POSE_ACTION_SERVER)

        self.handle_x = 0.45
        self.handle_y = 0.0
        self.handle_z = 0.90 

        self.base_roll = 0.0
        self.base_pitch = math.pi / 2
        self.base_yaw = 0.0
        self.handle_orientation = quaternion_from_euler(self.base_roll, self.base_pitch, self.base_yaw)

    def send_pose_goal(self, pose: Pose, phase_name: str):
        self.get_logger().info(f'--- INICIANDO {phase_name} ---')
        self.move_to_pose_client.wait_for_server()

        goal_msg = MoveToPose.Goal()
        
        # 1. Empaquetar en PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'link_base'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        goal_msg.pose = pose_stamped
        
        # 2. PARÁMETROS CRÍTICOS PARA EL SERVIDOR
        goal_msg.target_link = 'link_eef'  
        goal_msg.velocity = 0.3
        goal_msg.acceleration = 0.3
        goal_msg.planning_time = 2.0
        goal_msg.planning_attempts = 5

        self.get_logger().info(f'Enviando objetivo: X={pose.position.x}, Y={pose.position.y}, Z={pose.position.z}')
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
        pose_a = Pose()
        pose_a.position = Point(x=self.handle_x - 0.15, y=self.handle_y, z=self.handle_z)
        pose_a.orientation = self.handle_orientation
        self.send_pose_goal(pose_a, "FASE A: PRE-GRASP")

        pose_b = Pose()
        pose_b.position = Point(x=self.handle_x, y=self.handle_y, z=self.handle_z)
        pose_b.orientation = self.handle_orientation
        self.send_pose_goal(pose_b, "FASE B: GRASP")

        # Restamos 45 grados (pi/4) al Roll del end-effector
        unlatch_roll = self.base_roll - (math.pi / 4) 
        pose_c = Pose()
        pose_c.position = Point(x=self.handle_x, y=self.handle_y, z=self.handle_z) # Posición X,Y,Z intacta
        pose_c.orientation = quaternion_from_euler(unlatch_roll, self.base_pitch, self.base_yaw)
        self.send_pose_goal(pose_c, "FASE C: UNLATCH (Bajar picaporte)")

        pose_d = Pose()
        pose_d.position = Point(x=self.handle_x - 0.25, y=self.handle_y + 0.15, z=self.handle_z)
        pose_d.orientation = pose_c.orientation 
        self.send_pose_goal(pose_d, "FASE D: PULL (Abrir puerta en arco)")

def main(args=None):
    rclpy.init(args=args)
    node = DoorOpenerFull()
    node.execute_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
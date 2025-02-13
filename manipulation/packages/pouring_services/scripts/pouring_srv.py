#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian, GetFloat32List, SetDigitalIO, SetInt16ById, SetInt16
import copy
import math as m

class XArmStabilizedMovement(Node):
    def __init__(self):
        super().__init__('xarm_stabilized_movement')
        self.set_servo_angle_client = self.create_client(MoveCartesian, '/xarm/set_servo_angle')
        self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
        # self.get_position_client = self.create_client(GetFloat32List, '/xarm/get_position_rpy')
        self.set_digital_io_client = self.create_client(SetDigitalIO, '/xarm/set_digital_out')
        self.motion_ctrl_client = self.create_client(SetInt16ById, '/xarm/motion_ctrl')
        self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')

        # Wait for services to be available
        while not self.set_servo_angle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/set_servo_angle not available, waiting again...')
        while not self.set_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/set_position not available, waiting again...')
        while not self.get_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/get_position_rpy not available, waiting again...')
        while not self.set_digital_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/set_digital_out not available, waiting again...')
        while not self.motion_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/motion_ctrl not available, waiting again...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/set_mode not available, waiting again...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /xarm/set_state not available, waiting again...')

    # Returns the arm with joint movements to the default horizontal pose
    def return_to_default_pose_horizontal(self):
        req = MoveCartesian.Request()
        req.mvvelo = 0.5
        req.mvacc = 7
        req.mvtime = 0
        req.mvradii = 0
        req.pose = [-1.57, -0.7853, -0.7853, -1.57, 0, -3.9253]

        future = self.set_servo_angle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Default horizontal pose movement successful')
        else:
            self.get_logger().error('Default horizontal pose movement failed')

    # Returns the arm with joint movements to the default vertical pose
    def return_to_default_pose_vertical(self):
        req = MoveCartesian.Request()
        req.mvvelo = 0.5
        req.mvacc = 7
        req.mvtime = 0
        req.mvradii = 0
        req.pose = [-1.57, -0.7853, -1.309, 0, 2.09, -2.35]

        future = self.set_servo_angle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Default vertical pose movement successful')
        else:
            self.get_logger().error('Default vertical pose movement failed')

    # Adjust the last joint angle
    def adjust_end_effector_yaw(self, joint6_angle):
        req = MoveCartesian.Request()
        req.mvvelo = 1
        req.mvacc = 7
        req.mvtime = 0
        req.mvradii = 0

        get_angle_req = GetFloat32List.Request()
        future = self.get_position_client.call_async(get_angle_req)
        rclpy.spin_until_future_complete(self, future)
        actual_pose = future.result().datas

        yaw_angle = m.radians(45 + joint6_angle)
        yaw_angle_fixed = m.trunc(actual_pose[5] / m.radians(90))
        yaw_transformed = -yaw_angle if yaw_angle_fixed < 0 else yaw_angle

        req.pose = [actual_pose[0], actual_pose[1], actual_pose[2], actual_pose[3], actual_pose[4], yaw_transformed]

        future = self.set_servo_angle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('End effector yaw adjustment successful')
        else:
            self.get_logger().error('End effector yaw adjustment failed')

    # Stabilized movement to point and execute grasp with the last given orientation of the end effector
    def xarm_move_to_point(self, x, y, z, actual_pose):
        req = MoveCartesian.Request()
        req.pose = actual_pose
        req.mvvelo = 80
        req.mvacc = 200
        req.mvtime = 0

        req.pose[0] = x
        req.pose[1] = y
        req.pose[2] = z

        future = self.set_position_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Movement to point successful')
        else:
            self.get_logger().error('Movement to point failed')

    # Activates the gripper and waits until it's completely closed
    def xarm_grasp(self, action):
        req = SetDigitalIO.Request()
        req.io_number = 1
        req.state = action

        future = self.set_digital_io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.75)  # Hardcoded sleep for gripper action

    # Main callback
    def cartesian_movement_callback(self):
        self.get_logger().info('Starting cartesian movement callback')

        # Configs for cartesian movement
        set_mode_req = SetInt16.Request()
        set_mode_req.data = 0
        future = self.set_mode_client.call_async(set_mode_req)
        rclpy.spin_until_future_complete(self, future)

        set_state_req = SetInt16.Request()
        set_state_req.data = 0
        future = self.set_state_client.call_async(set_state_req)
        rclpy.spin_until_future_complete(self, future)

        time.sleep(2.0)

        # Example usage
        self.return_to_default_pose_vertical()
        self.adjust_end_effector_yaw(90)
        self.get_logger().info('Cartesian movement callback finished')

def main(args=None):
    rclpy.init(args=args)
    node = XArmStabilizedMovement()
    node.cartesian_movement_callback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


    
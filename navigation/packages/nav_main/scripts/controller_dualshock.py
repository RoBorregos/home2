#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class ControllerDualShock(Node):
    def __init__(self):
        super().__init__('controller_dualshock')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose/_action/cancel_goal')
        self.rotation_speed = 0.1
        self.linear_speed = 0.2 
    
    def joy_callback(self, msg):    

        twist = Twist()
        twist.linear.x = msg.axes[1] * self.linear_speed
        twist.angular.z = msg.axes[3] * self.rotation_speed
        self.publisher.publish(twist)

        if msg.buttons[3] == 1:  # Square pressed
            self.nav2_client.cancel_goal_async()

        # D-pad left increase linear speed
        if msg.axes[6] > 0.5:
            self.linear_speed = min(self.linear_speed + 0.05, 1.0)
            self.get_logger().info(f'Linear speed: {self.linear_speed:.2f}')

        # D-pad right decrease linear speed
        if msg.axes[6] < -0.5:
            self.linear_speed = max(self.linear_speed - 0.05, 0.05 )
            self.get_logger().info(f'Linear speed: {self.linear_speed:.2f}')

        # D-pad Up: increase rotation speed
        if msg.axes[7] > 0.5:
            self.rotation_speed = min(self.rotation_speed + 0.05, 1.0)
            self.get_logger().info(f'Rotation speed: {self.rotation_speed:.2f}')

        # D-pad Down: decrease rotation speed
        if msg.axes[7] < -0.5:
            self.rotation_speed = max(self.rotation_speed - 0.05, 0.05)
            self.get_logger().info(f'Rotation speed: {self.rotation_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerDualShock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

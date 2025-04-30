#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class VirtualStop(Node):
    def __init__(self):
        super().__init__('virtual_stop')

        self.get_logger().info("Virtual Stop Button started")
        # Create a subscription to the joystick topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.pub_stop = self.create_publisher(
            Bool,
            '/virtual_stop',
            10
        )
        self.trigger = False
        self.debounce_flag = False
        self.create_timer(0.1, self.stop_robot)

    def joy_callback(self, msg):
        # Check if the button is pressed
        if msg.buttons[0] == 1:
            if self.debounce_flag == False:
                self.trigger = not self.trigger    
                self.debounce_flag = True     
        else:
            self.debounce_flag = False  

    def stop_robot(self):
        stop_msg = Bool()
        stop_msg.data = self.trigger
        self.pub_stop.publish(stop_msg)
        #self.get_logger().info(f"Robot stop status: {stop_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    move_action_server = VirtualStop()
    rclpy.spin(move_action_server)


if __name__ == '__main__':
    main()
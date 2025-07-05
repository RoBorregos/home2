#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from xarm_msgs.srv import GetDigitalIO
from frida_interfaces.msg import GripperDetection

class IRGripper(Node):

    def __init__(self):
        super().__init__('ir_gripper_node')

        self.cli = self.create_client(GetDigitalIO, '/xarm/get_tgpio_digital')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Waiting for get_tgpio_digital service...')

        self.tool_gpio_1_pub = self.create_publisher(GripperDetection, 'ir_detection', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        request = GetDigitalIO.Request()
        future = self.cli.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                msg = GripperDetection()
                msg.ir_detection = bool(response.digitals[1])
                self.tool_gpio_1_pub.publish(msg)
                self.get_logger().info(f'Published GPIO states: {msg.ir_detection}')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)

    ir_gripper = IRGripper()

    rclpy.spin(ir_gripper)
    ir_gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
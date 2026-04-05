#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from xarm_msgs.srv import GetDigitalIO
from frida_interfaces.msg import GripperGraspState


class GraspDetector(Node):
    def __init__(self):
        super().__init__("grasp_detector_node")

        self.cli = self.create_client(GetDigitalIO, "/xarm/get_tgpio_digital")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_tgpio_digital service...")

        self.grasp_state_pub = self.create_publisher(
            GripperGraspState, "gripper/grasp_state", 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        request = GetDigitalIO.Request()
        future = self.cli.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                msg = GripperGraspState()
                msg.object_detected = bool(response.digitals[1])
                self.grasp_state_pub.publish(msg)
                self.get_logger().info(f"Gripper grasp state: {msg.object_detected}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {str(e)}")

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)

    grasp_detector = GraspDetector()

    rclpy.spin(grasp_detector)
    grasp_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

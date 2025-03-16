#!/usr/bin/env python3

# This node is the interface between manipulation tasks and the task managers
# This can receive tasks from ManipulationTask.action (pick, place, move, etc.) and call ManipulationCore accordingly
# It can also handle debugging tasks, such as using the rviz point publisher to try picking tasks

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_constants.manipulation_constants import (
    PICK_ACTION_SERVER,
)
from frida_interfaces.action import PickTask
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener


class ManipulationServer(Node):
    def __init__(self):
        super().__init__("manipulation_server")
        self.callback_group = ReentrantCallbackGroup()

        # This is on manipulation core node
        self._pick_action_client = ActionClient(
            self,
            PickTask,
            PICK_ACTION_SERVER,
        )

        # Server here, which is the interface between manipulation tasks and the task managers
        # Point Subscriber for debug here
        self.subs = self.create_subscription(
            Point,
            "/clicked_point",
            self.point_callback,
            10,
        )
        self.last_point = None
        self.last_process_point = None

        self.get_logger().info("Manipulation Server is ready")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, True)

        self.create_timer(0.1, self.timer_callback)

    def point_callback(self, msg):
        self.last_point = msg

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback}")

    def result_callback(self, result):
        self.get_logger().info(f"Result: {result}")

    def timer_callback(self):
        if self.last_point == self.last_process_point:
            return
        self.last_process_point = self.last_point
        if self.last_point is None:
            return
        a = PickTask.Goal()
        a.object_point = self.last_point
        a.object_name = "Clicked Point"
        self._pick_action_client.send_goal_async(
            a,
            feedback_callback=self.feedback_callback,
            result_callback=self.result_callback,
        )
        self.get_logger().info(f"Received point: {self.last_point}")
        self.get_logger().info("Sending to manipulation core")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

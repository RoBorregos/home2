#!/usr/bin/env python3

# This node is the interface between manipulation tasks and the task managers
# This can receive tasks from ManipulationTask.action (pick, place, move, etc.) and call ManipulationCore accordingly
# It can also handle debugging tasks, such as using the rviz point publisher to try picking tasks

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
)
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
from geometry_msgs.msg import PointStamped


class ManipulationClient(Node):
    def __init__(self):
        super().__init__("manipulation_client")
        self.callback_group = ReentrantCallbackGroup()

        # This is on manipulation core node
        self._manipulation_action_client = ActionClient(
            self,
            ManipulationAction,
            MANIPULATION_ACTION_SERVER,
        )
        self._manipulation_action_client.wait_for_server()

        # Server here, which is the interface between manipulation tasks and the task managers
        # Point Subscriber for debug here
        self.subs = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.point_callback,
            10,
        )
        self.point_pub_debug = self.create_publisher(
            PointStamped, "/clicked_point2", 10
        )
        self.last_point = None
        self.last_process_point = None

        self.get_logger().info("Manipulation Client is ready")

        self.pick_object_name = None
        self.active = True

        self.create_timer(0.1, self.point_timer_callback)

    def point_callback(self, msg):
        print("Received point")
        self.point_pub_debug.publish(msg)
        self.last_point = msg

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback}")

    def result_callback(self, result):
        self.get_logger().info(f"Result: {result}")

    def point_timer_callback(self):
        if self.last_point == self.last_process_point:
            return
        self.last_process_point = self.last_point
        if self.last_point is None:
            return
        print("Triggering pick task")

        manipulation_goal = ManipulationAction.Goal()
        manipulation_goal.task_type = ManipulationTask.PICK
        manipulation_goal.object_name = "clicked_object"
        manipulation_goal.object_point = self.last_point
        self._manipulation_action_client.wait_for_server()
        future = self._manipulation_action_client.send_goal_async(manipulation_goal)
        future = wait_for_future(future)
        self.get_logger().info(f"Received point: {self.last_point}")
        self.get_logger().info("Sending to manipulation core")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    manipulation_client = ManipulationClient()
    executor.add_node(manipulation_client)
    executor.spin()
    manipulation_client.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

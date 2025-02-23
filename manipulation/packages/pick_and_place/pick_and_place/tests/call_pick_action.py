#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from frida_interfaces.action import PickAction


class PickActionClient(Node):
    def __init__(self):
        super().__init__("pick_action_client")
        self._action_client = ActionClient(self, PickAction, "pick_action_server")
        self.get_logger().info("Pick Action Client has been started")

    def send_goal(self, poses):
        # Wait for action server
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = PickAction.Goal()
        goal_msg.grasping_poses = poses

        # Send goal
        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = PickActionClient()

    # Create sample poses
    sample_poses = []

    # Pose 1 - Above table
    pose1 = Pose()
    pose1.position.x = 0.4
    pose1.position.y = 0.0
    pose1.position.z = 0.3
    pose1.orientation.w = 1.0

    # Pose 2 - Left side
    pose2 = Pose()
    pose2.position.x = 0.4
    pose2.position.y = 0.2
    pose2.position.z = 0.3
    pose2.orientation.w = 1.0

    # Pose 3 - Right side
    pose3 = Pose()
    pose3.position.x = 0.4
    pose3.position.y = -0.2
    pose3.position.z = 0.3
    pose3.orientation.w = 1.0

    sample_poses = [pose1, pose2, pose3]

    # Send the goal with sample poses
    action_client.send_goal(sample_poses)

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()

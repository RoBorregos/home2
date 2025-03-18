#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from frida_interfaces.action import PickMotion


class PickMotionClient(Node):
    def __init__(self):
        super().__init__("pick_action_client")
        self._action_client = ActionClient(self, PickMotion, "pick_action_server")
        self.get_logger().info("Pick Action Client has been started")

    def send_goal(self, stamped_poses):
        # Wait for action server
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = PickMotion.Goal()
        goal_msg.grasping_poses = stamped_poses

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
    action_client = PickMotionClient()

    # Create sample stamped poses
    stamped_poses = []

    # Get current time for stamping
    stamp = action_client.get_clock().now().to_msg()

    # Pose 1 - Above table
    pose1 = PoseStamped()
    pose1.header.stamp = stamp
    pose1.header.frame_id = "link_base"
    pose1.pose.position.x = 0.4
    pose1.pose.position.y = 0.0
    pose1.pose.position.z = 0.3
    pose1.pose.orientation.w = 1.0

    # Pose 2 - Left side
    pose2 = PoseStamped()
    pose2.header.stamp = stamp
    pose2.header.frame_id = "link_base"
    pose2.pose.position.x = 0.4
    pose2.pose.position.y = 0.2
    pose2.pose.position.z = 0.3
    pose2.pose.orientation.w = 1.0

    # Pose 3 - Right side
    pose3 = PoseStamped()
    pose3.header.stamp = stamp
    pose3.header.frame_id = "link_base"
    pose3.pose.position.x = 0.35
    pose3.pose.position.y = 0.35
    pose3.pose.position.z = 0.4
    pose3.pose.orientation.x = 1.0
    pose3.pose.orientation.y = 0.0
    pose3.pose.orientation.z = 0.0
    pose3.pose.orientation.w = 0.0

    stamped_poses = [pose1, pose2, pose3]

    # Send the goal with sample stamped poses
    action_client.send_goal(stamped_poses)

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import MoveToPose
from geometry_msgs.msg import PoseStamped


class MoveToPoseClient(Node):
    def __init__(self):
        super().__init__("move_to_pose_client")
        self._action_client = ActionClient(
            self, MoveToPose, "move_to_pose_action_server"
        )

    # let the server pick the default values
    def send_goal(
        self,
        pose=PoseStamped(),
        velocity=0.0,
        acceleration=0.0,
        planner_id="",
    ):
        goal_msg = MoveToPose.Goal()
        goal_msg.pose = pose
        goal_msg.velocity = velocity
        goal_msg.acceleration = acceleration
        goal_msg.planner_id = planner_id

        self._action_client.wait_for_server()

        self.get_logger().info("Sending pose goal...")
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToPoseClient()

    # Define your pose goal here
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "link_base"  # Set appropriate frame
    pose_stamped.header.stamp = action_client.get_clock().now().to_msg()

    # Set the pose component
    pose_stamped.pose.position.x = 0.5
    pose_stamped.pose.position.y = 0.5
    pose_stamped.pose.position.z = 0.4
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    # Send goal
    future = action_client.send_goal(pose_stamped)

    # Wait for goal to be accepted
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        action_client.get_logger().error("Goal rejected")
        rclpy.shutdown()
        return

    action_client.get_logger().info("Goal accepted")

    # Get result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, result_future)

    result = result_future.result().result
    if result.success:
        action_client.get_logger().info("Goal succeeded!")
    else:
        action_client.get_logger().error("Goal failed!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()

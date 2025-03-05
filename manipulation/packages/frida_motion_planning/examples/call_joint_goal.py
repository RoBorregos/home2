#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import MoveJoints


class MoveJointsClient(Node):
    def __init__(self):
        super().__init__("move_joints_client")
        self._action_client = ActionClient(
            self, MoveJoints, "move_joints_action_server"
        )

    # let the server pick the default values
    def send_goal(
        self,
        joint_names=[],
        joint_positions=[],
        velocity=0.0,
        acceleration=0.0,
        planner_id="",
    ):
        goal_msg = MoveJoints.Goal()
        goal_msg.joint_names = joint_names
        goal_msg.joint_positions = joint_positions
        goal_msg.velocity = velocity
        goal_msg.acceleration = acceleration
        goal_msg.planner_id = planner_id

        self._action_client.wait_for_server()

        self.get_logger().info("Sending joint goal...")
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveJointsClient()

    # Define your joint goals here, by default it goes through the order of the chain
    # commented so it goes through default
    # req.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_names = []
    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Send goal
    future = action_client.send_goal(joint_names, joint_positions)

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

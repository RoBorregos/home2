#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from frida_interfaces.action import PickAction


class PickActionServer(Node):
    def __init__(self):
        super().__init__("pick_action_server")
        self._action_server = ActionServer(
            self,
            PickAction,
            "pick_action_server",
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Pick Action Server has been started")

    async def execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pick goal...")

        # Initialize result
        feedback = PickAction.Feedback()
        result = PickAction.Result()

        try:
            await self.perform_pick(goal_handle, feedback)

            goal_handle.succeed()
            result.success = True
            return result

        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    async def perform_pick(self, goal_handle, feedback):
        """Perform the pick operation."""
        self.get_logger().info("Performing pick operation...")
        time.sleep(5)  # Simulate pick operation
        self.get_logger().info("Pick operation completed")


def main(args=None):
    rclpy.init(args=args)
    pick_server = PickActionServer()
    rclpy.spin(pick_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

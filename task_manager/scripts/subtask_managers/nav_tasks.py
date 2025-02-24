#!/usr/bin/env python3

"""
Node to move to a place.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from frida_interfaces.action import Move

from utils.decorators import mockable, service_check
from utils.logger import Logger

MOVE_TOPIC = "/navigation/move"

TIMEOUT = 5.0


class NavigationTasks:
    """Class to manage the navigation tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    def __init__(self, task_manager, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data

        self.move_action_client = ActionClient(self.node, Move, MOVE_TOPIC)

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        if not self.move_action_client.wait_for_server(timeout_sec=TIMEOUT):
            self.node.get_logger().warn("Move service not initialized.")

    @mockable(return_value=True, delay=2)
    @service_check("move_action_client", False, TIMEOUT)
    def move_to_location(self, location: str) -> bool:
        """Attempts to move to the given location and returns True if successful."""

        self.node.get_logger().info(f"Sending move request to: {location}")
        goal = Move.Goal()
        goal.location = location

        try:
            goal_future = self.move_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self.node, goal_future, timeout_sec=TIMEOUT
            )

            goal_handle = goal_future.result()

            if not goal_handle.accepted:
                raise Exception("Goal rejected")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=TIMEOUT
            )
            result = result_future.result()

            if result and result.result.success:
                Logger.success(node, f"Succesfully moved to {location}.")
                return self.STATE["EXECUTION_SUCCESS"]
            else:
                self.node.get_logger().warn(f"Move to {location} failed.")
                return self.STATE["EXECUTION_ERROR"]

        except Exception as e:
            self.node.get_logger().error(f"Error moving to location: {e}")
            return self.STATE["EXECUTION_ERROR"]


if __name__ == "__main__":
    rclpy.init()
    node = Node("navigation_tasks")
    navigation_tasks = NavigationTasks(node)

    try:
        while rclpy.ok():
            location = input("Enter a location: ")
            if location.lower() == "exit":
                break  # Exit loop

            success = navigation_tasks.move_to_location(location)
            if success:
                node.get_logger().info("Move action completed successfully.")
            else:
                node.get_logger().warn("Move action failed.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

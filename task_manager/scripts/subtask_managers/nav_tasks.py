#!/usr/bin/env python3

"""
Node to move to a place.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from utils.decorators import mockable, service_check
from utils.logger import Logger
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
import os
import json


MOVE_TOPIC = "/navigate_to_pose"

TIMEOUT = 5.0


class NavigationTasks:
    """Class to manage the navigation tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    def __init__(self, task_manager, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        self.pose_client = ActionClient(self.node, NavigateToPose, MOVE_TOPIC)
        if not self.pose_client.wait_for_server(timeout_sec=TIMEOUT):
            self.node.get_logger().warn("Move service not initialized.")

    @mockable(return_value=True, delay=2)
    @service_check("pose_client", False, TIMEOUT)
    def move_to_location(self, location: str, sublocation: str) -> bool:
        """Attempts to move to the given location and returns True if successful."""

        try:
            package_share_directory = get_package_share_directory("frida_constants")
            file_path = os.path.join(package_share_directory, "map_areas/areas.json")
            with open(file_path, "r") as file:
                data = json.load(file)
            if sublocation != "":
                coordinates = data[location][sublocation]
            else:
                coordinates = data[location]["safe_place"]
                sublocation = "safe_place"
            self.node.get_logger().info(f"{coordinates}")
        except Exception as e:
            self.node.get_logger().error(f"Error fetching coordinates: {e}")
            return self.STATE["EXECUTION_ERROR"]

        try:
            self.node.get_logger().info(f"Sending move request to: {location} {sublocation}")
            client_goal = NavigateToPose.Goal()
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = coordinates[0]
            goal.pose.position.y = coordinates[1]
            goal.pose.position.z = coordinates[2]
            goal.pose.orientation.x = coordinates[3]
            goal.pose.orientation.y = coordinates[4]
            goal.pose.orientation.z = coordinates[5]
            goal.pose.orientation.w = coordinates[6]

            client_goal.pose = goal

            goal_future = self.pose_client.send_goal_async(client_goal)
            while not goal_future.done():
                rclpy.spin_once(self.node)
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                raise Exception("Goal rejected")

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result()

            if result.status == rclpy.action.ResultCode.SUCCEEDED:
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
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

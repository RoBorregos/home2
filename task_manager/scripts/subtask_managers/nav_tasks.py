#!/usr/bin/env python3

"""
Node to move to a place.
"""

import json
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from utils.decorators import mockable, service_check
from utils.status import Status
from utils.task import Task
from utils.logger import Logger
from frida_constants.navigation_constants import (
    GOAL_TOPIC,
    FOLLOWING_SERVICE,
)
from frida_interfaces.srv import LaserGet

TIMEOUT = 10.0
RETURN_LASER_DATA = "/integration/Laserscan"


class NavigationTasks:
    """Class to manage the navigation tasks"""

    def __init__(self, task_manager: Node, task: Task, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        self.goal_state = None
        # Closed door variables
        self.range_max = 870
        self.range_min = 750
        self.closed_distance = 0.7
        self.laser_sub = None
        # Action clients and services
        self.goal_client = ActionClient(self.node, NavigateToPose, GOAL_TOPIC)
        self.activate_follow = self.node.create_client(SetBool, FOLLOWING_SERVICE)
        self.laser_send = self.node.create_client(LaserGet, RETURN_LASER_DATA)

        self.services = {
            Task.RECEPTIONIST: {
                "goal_client": {"client": self.goal_client, "type": "action"},
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
            Task.HELP_ME_CARRY: {
                "activate_follow": {"client": self.activate_follow, "type": "service"},
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
            Task.GPSR: {
                "goal_client": {"client": self.goal_client, "type": "action"},
                "activate_follow": {"client": self.activate_follow, "type": "service"},
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
            Task.STORING_GROCERIES: {
                "goal_client": {"client": self.goal_client, "type": "action"},
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
            Task.DEBUG: {
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
        }

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""

        if self.task not in self.services:
            Logger.error(self.node, "Task not available")
            return

        for key, service in self.services[self.task].items():
            if service["type"] == "service":
                if not service["client"].wait_for_service(timeout_sec=TIMEOUT):
                    Logger.warn(self.node, f"{key} service not initialized. ({self.task})")
            elif service["type"] == "action":
                if not service["client"].wait_for_server(timeout_sec=TIMEOUT):
                    Logger.warn(self.node, f"{key} action server not initialized. ({self.task})")

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=10)
    @service_check("goal_client", False, TIMEOUT)
    def move_to_location(self, location: str, sublocation: str) -> Future:
        """Attempts to move to the given location and returns a Future that completes when the action finishes.
        Call the function on this way

        future = self.subtask_manager["navigation"].move_to_location("living_room", "couches")
        # Wait for the action result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        """
        future = Future()
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
            Logger.info(self.node, f"{coordinates}")
        except Exception as e:
            Logger.error(self.node, f"Error fetching coordinates: {e}")
            future.set_result(Status.EXECUTION_ERROR)
            return future

        try:
            Logger.info(self.node, f"Sending move request to: {location} {sublocation}")
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
            self.goal_state = None
            self._send_goal_future = self.goal_client.send_goal_async(client_goal)

            self._send_goal_future.add_done_callback(
                lambda future_goal: self.goal_response_callback(future_goal, future)
            )
            return future
        except Exception as e:
            Logger.error(self.node, f"Error moving to location: {e}")
            future.set_result(Status.EXECUTION_ERROR)
            return future

    @mockable(return_value=True, delay=10)
    @service_check("pose_client", False, TIMEOUT)
    def move_to_zero(self) -> Future:
        """Attempts to move to the original location and returns a Future that completes when the action finishes.
        Call the function on this way

        future = self.subtask_manager["navigation"].move_to_zero()
        # Wait for the action result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        """
        future = Future()
        try:
            coordinates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.node.get_logger().info(f"{coordinates}")
        except Exception as e:
            self.node.get_logger().error(f"Error moving to original location: {e}")
            future.set_result(self.STATE["EXECUTION_ERROR"])
            return future

        try:
            self.node.get_logger().info("Sending move request to original location")
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
            self.goal_state = None
            self._send_goal_future = self.pose_client.send_goal_async(client_goal)

            self._send_goal_future.add_done_callback(
                lambda future_goal: self.goal_response_callback(future_goal, future)
            )
            return future
        except Exception as e:
            self.node.get_logger().error(f"Error moving to original location: {e}")
            future.set_result(self.STATE["EXECUTION_ERROR"])
            return future

    def goal_response_callback(self, future, result_future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            Logger.info(self.node, "Goal rejected.")
            result_future.set_result(Status.EXECUTION_ERROR)
            return

        Logger.info(self.node, "Goal accepted! Waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future_result: self.result_callback(result_future)
        )

    def result_callback(self, result_future):
        Logger.info(self.node, "Goal execution completed!")
        goal_handle = self._get_result_future.result()
        print(f"Goal handle papu pro: {goal_handle.status}")
        if goal_handle.status != 4:
            Logger.info(self.node, "Goal execution failed")
            result_future.set_result(Status.EXECUTION_ERROR)
            return
        result_future.set_result(Status.EXECUTION_SUCCESS)

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=3)
    @service_check("activate_follow", False, TIMEOUT)
    def follow_person(self, activate: bool):
        """Activate or deactivate the follow person mode"""
        try:
            request = SetBool.Request()
            request.data = activate
            future = self.activate_follow.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()
            if not result.success:
                raise Exception("Service call failed")
        except Exception as e:
            Logger.info(self.node, f"Error sending follow person mode: {e}")
            return Status.EXECUTION_ERROR
        if activate:
            Logger.info(self.node, "Follow person activated")
        else:
            Logger.info(self.node, "Follow person deactivated")
        return Status.EXECUTION_SUCCESS

    @mockable(return_value=(Status.EXECUTION_SUCCESS, "open"), delay=3)
    @service_check("laser_send", False, TIMEOUT)
    def check_door(self) -> tuple[int, str]:
        """Check if the door is open or closed"""

        request = LaserGet.Request()
        future = self.laser_send.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        result = future.result()
        if result is not None:
            if result.status:
                self.laser_sub = result.data
            else:
                Logger.error(self.node, "Error with request")
                return (Status.EXECUTION_ERROR, "")
        else:
            Logger.error(self.node, "Error with request")
            return (Status.EXECUTION_ERROR, "")

        door_points = []
        for count, r in enumerate(self.laser_sub.ranges):
            if self.range_min <= count <= self.range_max:
                door_points.append(r)

        # Check if the door is open
        if len(door_points) > 0:
            # Calculate the average distance of the door points
            avg_distance = sum(door_points) / len(door_points)
            # Check if the average distance is less than a threshold
            Logger.info(self.node, f"Average distance: {avg_distance}")
            if avg_distance < self.closed_distance:
                Logger.info(self.node, "Door closed")
                return (Status.EXECUTION_SUCCESS, "closed")
            else:
                Logger.info(self.node, "Door open")
                return (Status.EXECUTION_SUCCESS, "open")
        else:
            Logger.error(self.node, "No points detected")
            return (Status.EXECUTION_ERROR, "")


if __name__ == "__main__":
    rclpy.init()
    node = Node("navigation_tasks")
    navigation_tasks = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

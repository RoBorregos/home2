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
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose

# from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from utils.decorators import mockable, service_check
from utils.status import Status
from utils.task import Task
from utils.logger import Logger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Empty
from frida_interfaces.srv import PointTransformation

from frida_constants.navigation_constants import (
    GOAL_TOPIC,
    FOLLOWING_SERVICE,
)
from frida_interfaces.srv import ReturnLocation, LaserGet, WaitForControllerInput

TIMEOUT_WAIT_FOR_SERVICE = 1.0
TIMEOUT = 4
RETURN_LASER_DATA = "/integration/Laserscan"
BT_LIFE_CYCLE_SERVICE = "/bt_navigator/change_state"
BT_PARAM_SERVICE = "/bt_navigator/set_parameters"
RETURN_LOCATION = "/integration/ReturnLocation"
RTAB_PAUSE_SERVICE = "/rtabmap/pause"
RTAB_RESUME_SERVICE = "/rtabmap/resume"
DEFAULT_BT_PATH = (
    "/workspace/src/navigation/packages/nav_main/bt/navigate_to_pose_w_replanning_and_recovery.xml"
)
FOLLOW_BT_PATH = (
    "/workspace/src/navigation/packages/nav_main/bt/navigate_to_pose_w_replanning_and_recovery.xml"
)

GOAL_POSE_TOPIC = "/goal_pose"


class NavigationTasks:
    """Class to manage the navigation tasks"""

    def __init__(self, task_manager: Node, task: Task, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        self.goal_state = None
        # Closed door variables
        self.range_max = 260
        self.range_min = 225
        self.closed_distance = 0.7
        self.laser_sub = None
        # Action clients and services
        self.goal_client = ActionClient(self.node, NavigateToPose, GOAL_TOPIC)
        self.activate_follow = self.node.create_client(SetBool, FOLLOWING_SERVICE)
        self.bt_params = self.node.create_client(SetParameters, BT_PARAM_SERVICE)
        self.bt_lifecycle = self.node.create_client(ChangeState, BT_LIFE_CYCLE_SERVICE)
        self.laser_send = self.node.create_client(LaserGet, RETURN_LASER_DATA)
        self.rtabmap_pause = self.node.create_client(Empty, RTAB_PAUSE_SERVICE)
        self.rtabmap_continue = self.node.create_client(Empty, RTAB_RESUME_SERVICE)
        self.ReturnLocation_client = self.node.create_client(ReturnLocation, RETURN_LOCATION)
        self.convert_point = (
            self.node.create_client(PointTransformation, "/integration/point_transformer"),
        )
        self.zero_publisher = self.node.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)
        self.wait_for_controller_input = self.node.create_client(
            WaitForControllerInput, "wait_for_controller_input"
        )
        self.services = {
            Task.RECEPTIONIST: {
                "goal_client": {"client": self.goal_client, "type": "action"},
            },
            Task.HELP_ME_CARRY: {
                "activate_follow": {"client": self.activate_follow, "type": "service"},
                "bt_params": {"client": self.bt_params, "type": "service"},
                "bt_lifecycle": {"client": self.bt_lifecycle, "type": "service"},
            },
            Task.GPSR: {
                "goal_client": {"client": self.goal_client, "type": "action"},
                "activate_follow": {"client": self.activate_follow, "type": "service"},
                "laser_send": {"client": self.laser_send, "type": "service"},
            },
            Task.STORING_GROCERIES: {
                "goal_client": {"client": self.goal_client, "type": "action"},
                "laser_send": {"client": self.laser_send, "type": "service"},
                "pause_rtab": {"client": self.rtabmap_pause, "type": "service"},
                "resume_rtab": {"client": self.rtabmap_continue, "type": "service"},
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
                if not service["client"].wait_for_service(timeout_sec=TIMEOUT_WAIT_FOR_SERVICE):
                    Logger.warn(self.node, f"{key} service not initialized. ({self.task})")
            elif service["type"] == "action":
                if not service["client"].wait_for_server(timeout_sec=TIMEOUT_WAIT_FOR_SERVICE):
                    Logger.warn(self.node, f"{key} action server not initialized. ({self.task})")

    def mock_to_location_controller(self, timeout=10):
        """Mock the controller to move to a location"""
        if not self.mock_data:
            Logger.error(self.node, "Mock data is not enabled")
            return

        try:
            Logger.info(self.node, "Waiting for controller input...")
            request = WaitForControllerInput.Request()
            request.button = "x"
            request.timeout = timeout
            future = self.wait_for_controller_input.call_async(request)
            return future
        except Exception as e:
            Logger.error(self.node, f"Error waiting for controller input: {e}")
            return Future().set_result(Status.EXECUTION_ERROR)

    # def get_distance_to_zero(self):
    #     self.getdist
    @mockable(return_value=True, delay=10)
    @service_check("pause_nav", False, timeout=3)
    def pause_nav(self):
        req = Empty.Request()
        future = self.rtabmap_pause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is not None:
            Logger.error(self.node, "Service call failed")
            return Status.EXECUTION_ERROR

        else:
            Logger.info(self.node, "Service call successfull")
            return Status.EXECUTION_SUCCESS

    @mockable(return_value=True, delay=10)
    @service_check("resume_nav", False, timeout=3)
    def resume_nav(self):
        req = Empty.Request()
        future = self.rtabmap_continue.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is not None:
            Logger.error(self.node, "Service call failed")
            return Status.EXECUTION_ERROR

        else:
            Logger.info(self.node, "Service call successfull")
            return Status.EXECUTION_SUCCESS

    @mockable(_mock_callback=mock_to_location_controller)
    @service_check("goal_client", False, timeout=3)
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
            return future  # = self.subtask_manager.nav.move_to_location(location, sublocation)

    def move_to_pose(self, pose: PoseStamped) -> Future:
        future = Future()
        client_goal = NavigateToPose.Goal()
        client_goal.pose = pose
        self.goal_state = None
        self._send_goal_future = self.goal_client.send_goal_async(client_goal)
        self._send_goal_future.add_done_callback(
            lambda future_goal: self.goal_response_callback(future_goal, future)
        )
        return future

    def move_to_point(self, point_s: PointStamped) -> Future:
        """Attempts to move to the original location and returns a Future that completes when the action finishes.
        Call the function on this way

        future = self.subtask_manager["navigation"].move_to_zero()
        # Wait for the action result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        """
        # try:
        #     goal = PoseStamped()
        #     goal.header.frame_id = "map"
        #     goal.pose.position.x = 0.0
        #     goal.pose.position.y = 0.0
        #     goal.pose.position.z = 0.0
        #     goal.pose.orientation.x = 0.0
        #     goal.pose.orientation.y = 0.0
        #     goal.pose.orientation.z = 0.0
        #     goal.pose.orientation.w = 0.0
        #     self.zero_publisher.publish(goal)
        #     return Status.EXECUTION_SUCCESS

        # except Exception as e:
        # #     Logger.error(self.node, f"Error moving to location: {e}")
        # #     future.set_result(Status.EXECUTION_ERROR)
        #     return Status.EXECUTION_SUCCESS

        future = Future()
        try:
            client_goal = NavigateToPose.Goal()
            request = PointTransformation.Request()
            request.point = point_s
            request.frame = "map"
            future = self.convert_point.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            results = future.result()
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = results.transformed_point.point.x
            goal.pose.position.y = results.transformed_point.point.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

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
    @service_check("goal_client", False, TIMEOUT)
    def move_to_zero(self) -> Future:
        """Attempts to move to the original location and returns a Future that completes when the action finishes.
        Call the function on this way

        future = self.subtask_manager["navigation"].move_to_zero()
        # Wait for the action result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        """
        # try:
        #     goal = PoseStamped()
        #     goal.header.frame_id = "map"
        #     goal.pose.position.x = 0.0
        #     goal.pose.position.y = 0.0
        #     goal.pose.position.z = 0.0
        #     goal.pose.orientation.x = 0.0
        #     goal.pose.orientation.y = 0.0
        #     goal.pose.orientation.z = 0.0
        #     goal.pose.orientation.w = 0.0
        #     self.zero_publisher.publish(goal)
        #     return Status.EXECUTION_SUCCESS

        # except Exception as e:
        # #     Logger.error(self.node, f"Error moving to location: {e}")
        # #     future.set_result(Status.EXECUTION_ERROR)
        #     return Status.EXECUTION_SUCCESS

        future = Future()
        try:
            client_goal = NavigateToPose.Goal()
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

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
            future = None
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

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=3)
    @service_check("bt_params", False, TIMEOUT)
    def change_bt(self, bt_str: str):
        """Change the behavior tree and return a Future"""
        future = Future()

        try:
            param_name = "default_nav_to_pose_bt_xml"
            if bt_str == "follow":
                param_value = "/workspace/src/navigation/packages/nav_main/bt/follow_dynamic.xml"
            else:
                param_value = "/workspace/src/navigation/packages/nav_main/bt/navigate_to_pose_w_replanning_and_recovery.xml"

            param = Parameter()
            param.name = param_name
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=param_value
            )

            param_request = SetParameters.Request()
            param_request.parameters = [param]

            # Set the parameter
            param_future = self.bt_params.call_async(param_request)
            rclpy.spin_until_future_complete(self.node, param_future)

            if param_future.result() is None:
                raise Exception("Parameter service call failed")

            # Deactivate the bt_navigator
            deactivate_request = ChangeState.Request()
            deactivate_request.transition.id = Transition.TRANSITION_DEACTIVATE  # 4

            deactivate_future = self.bt_lifecycle.call_async(deactivate_request)
            rclpy.spin_until_future_complete(self.node, deactivate_future)

            if deactivate_future.result() is None:
                raise Exception("Failed to deactivate bt_navigator")

            # Reactivate the bt_navigator
            activate_request = ChangeState.Request()
            activate_request.transition.id = Transition.TRANSITION_ACTIVATE  # 3

            activate_future = self.bt_lifecycle.call_async(activate_request)
            rclpy.spin_until_future_complete(self.node, activate_future)

            if activate_future.result() is None:
                raise Exception("Failed to reactivate bt_navigator")

            Logger.info(self.node, f"Successfully changed BT to: {bt_str}")
            future.set_result(Status.EXECUTION_SUCCESS)

        except Exception as e:
            Logger.error(self.node, f"Error changing behavior tree: {e}")
            future.set_result(Status.EXECUTION_ERROR)

        return future

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
        # print(self.laser_sub.ranges)
        door_points = []
        for count, r in enumerate(self.laser_sub.ranges):
            print(f"distance={r}, number = {count}")
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

    def ReturnLocation_callback(self):
        try:
            request = ReturnLocation.Request()

            future = self.ReturnLocation_client.call_async(request)

            rclpy.spin_until_future_complete(self.node, future)

            results = future.result()

            if results is not None:
                return (Status.EXECUTION_SUCCESS, results)
            else:
                Logger.error(self.node, "Error getting location")
                return (Status.EXECUTION_ERROR, None)
        except Exception as e:
            Logger.error(self.node, f"Error getting location: {e}")
            return (Status.EXECUTION_ERROR, None)


if __name__ == "__main__":
    rclpy.init()
    node = Node("navigation_tasks")
    navigation_tasks = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

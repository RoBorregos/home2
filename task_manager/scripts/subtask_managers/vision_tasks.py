#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import FindSeat, SaveName, PersonDescription, BeverageLocation
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from utils.decorators import mockable, service_check
from utils.logger import Logger
from utils.task import Task

SAVE_NAME_TOPIC = "/vision/new_name"
FIND_SEAT_TOPIC = "/vision/find_seat"
DETECT_PERSON_TOPIC = "/vision/detect_person"
FOLLOW_TOPIC = "/vision/follow_face"
FOLLOW_BY_TOPIC = "/vision/follow_by_name"
DESCRIPTION_TOPIC = "/vision/person_description"
BEVERAGE_TOPIC = "/vision/beverage_location"
TIMEOUT = 5.0


class VisionTasks:
    """Class to manage the vision tasks"""

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1,
        "TARGET_NOT_FOUND": 2,
    }

    def __init__(self, task_manager, task: Task, mock_data=False) -> None:
        """Initialize the class"""
        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        self.follow_face = {"x": None, "y": None}
        self.flag_active_face = False

        self.face_subscriber = self.node.create_subscription(
            Point, FOLLOW_TOPIC, self.follow_callback, 10
        )
        self.save_name_client = self.node.create_client(SaveName, SAVE_NAME_TOPIC)
        self.find_seat_client = self.node.create_client(FindSeat, FIND_SEAT_TOPIC)
        self.follow_by_name_client = self.node.create_client(SaveName, FOLLOW_BY_TOPIC)
        self.person_description_client = self.node.create_client(
            PersonDescription, DESCRIPTION_TOPIC
        )
        self.beverage_location_client = self.node.create_client(BeverageLocation, BEVERAGE_TOPIC)
        self.detect_person_action_client = ActionClient(
            self.node, DetectPerson, DETECT_PERSON_TOPIC
        )

        self.services = {
            Task.RECEPTIONIST: {
                "detect_person": {
                    "client": self.detect_person_action_client,
                    "type": "action",
                },
                "find_seat": {
                    "client": self.find_seat_client,
                    "type": "service",
                },
                "save_face_name": {
                    "client": self.save_name_client,
                    "type": "service",
                },
                "person_description": {"client": self.person_description_client, "type": "service"},
                "beverage_location": {"client": self.beverage_location_client, "type": "service"},
                "follow_by_name": {"client": self.follow_by_name_client, "type": "service"},
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

    def follow_callback(self, msg: Point):
        """Callback for the face following subscriber"""
        self.follow_face["x"] = msg.x
        self.follow_face["y"] = msg.y
        self.flag_active_face = True

    @mockable(return_value=100)
    @service_check("save_name_client", -1, TIMEOUT)
    def save_face_name(self, name: str) -> int:
        """Save the name of the person detected"""

        Logger.info(self.node, f"Saving name: {name}")
        request = SaveName.Request()
        request.name = name

        try:
            future = self.save_name_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error saving name: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, f"Name saved: {name}")
        return self.STATE["EXECUTION_SUCCESS"]

    @mockable(return_value=100)
    @service_check("find_seat_client", 300, TIMEOUT)
    def find_seat(self) -> tuple[int, float]:
        """Find an available seat and get the angle for the camera to point at"""

        Logger.info(self.node, "Finding available seat")
        request = FindSeat.Request()
        request.request = True

        try:
            future = self.find_seat_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No seat found")
                return self.STATE["TARGET_NOT_FOUND"], 300

        except Exception as e:
            Logger.error(self.node, f"Error finding seat: {e}")
            return self.STATE["EXECUTION_ERROR"], 300

        Logger.success(self.node, f"Seat found: {result.angle}")
        return self.STATE["EXECUTION_SUCCESS"], result.angle

    @mockable(return_value=True, delay=2, mock=False)
    @service_check("detect_person_action_client", False, TIMEOUT)
    def detect_person(self, timeout: float = TIMEOUT) -> int:
        """Returns true when a person is detected"""

        Logger.info(self.node, "Waiting for person detection")
        goal = DetectPerson.Goal()
        goal.request = True

        try:
            goal_future = self.detect_person_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=timeout)

            goal_handle = goal_future.result()
            Logger.info(self.node, f"Goal future result: {goal_handle}")

            if goal_handle is None:
                raise Exception("Failed to get a valid goal handle")

            if not goal_handle.accepted:
                raise Exception("Goal rejected")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
            result = result_future.result()

            if result and result.result.success:
                Logger.success(self.node, "Person detected")
                return self.STATE["EXECUTION_SUCCESS"]
            else:
                Logger.warn(self.node, "No person detected")
                return self.STATE["EXECUTION_ERROR"]
        except Exception as e:
            Logger.error(self.node, f"Error detecting person: {e}")
            return self.STATE["EXECUTION_ERROR"]

    @mockable(return_value=True, delay=2)
    def detect_guest(self, name: str, timeout: float = TIMEOUT):
        """Returns true when the guest is detected"""
        pass

    @mockable(return_value=True, delay=2)
    def find_drink(self, drink: str, timeout: float = TIMEOUT):
        """Returns true when a person is detected"""
        Logger.info(self.node, f"Finding drink: {drink}")
        request = BeverageLocation.Request()
        request.beverage = drink

        try:
            future = self.beverage_location_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No drink found")
                return "not found", self.STATE["TARGET_NOT_FOUND"]

        except Exception as e:
            Logger.error(self.node, f"Error finding drink: {e}")
            return "not found", self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, f"Found drink: {drink}")
        return "center", self.STATE["EXECUTION_SUCCESS"]

    @mockable(return_value="tall person", delay=5, mock=False)
    @service_check("person_description_client", "No description generated", TIMEOUT)
    def describe_person(self):
        """Requests a description of a person and handles the response asynchronously."""
        Logger.info(self.node, "Requesting description of a person")
        request = PersonDescription.Request()
        request.request = True

        try:
            future = self.person_description_client.call_async(request)
            future.add_done_callback(self._handle_description_response)
        except Exception as e:
            Logger.error(self.node, f"Error requesting description: {e}")
            return self.STATE["EXECUTION_ERROR"]

    @mockable(return_value=None, delay=2)
    def follow_by_name(self, name: str):
        """Follow a person by name or area"""
        Logger.info(self.node, f"Following face by: {name}")
        request = SaveName.Request()
        request.name = name

        try:
            future = self.follow_by_name_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error following face by: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, f"Following face success: {name}")
        return self.STATE["EXECUTION_SUCCESS"]

    def _handle_description_response(self, future):
        """Callback to handle the response from the description service."""
        try:
            result = future.result()
            if result.success:
                Logger.success(self.node, f"Description: {result.description}")
                self.node.get_guest().description = result.description
            else:
                Logger.warn(self.node, "No description generated")
        except Exception as e:
            Logger.error(self.node, f"Error processing description response: {e}")

    def get_follow_face(self):
        """Get the face to follow"""
        if self.flag_active_face:
            self.flag_active_face = False
            return self.follow_face["x"], self.follow_face["y"]
        else:
            return None, None


if __name__ == "__main__":
    rclpy.init()
    node = Node("vision_tasks")
    vision_tasks = VisionTasks(node, task="RECEPTIONIST")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

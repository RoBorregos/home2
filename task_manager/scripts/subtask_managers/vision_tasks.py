#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from frida_interfaces.srv import SaveName
from frida_interfaces.srv import FindSeat
from frida_interfaces.action import DetectPerson

from utils.decorators import mockable, service_check

SAVE_NAME_TOPIC = "/vision/new_name"
FIND_SEAT_TOPIC = "/vision/find_seat"
DETECT_PERSON_TOPIC = "/vision/detect_person"

TIMEOUT = 5.0


class VisionTasks:
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    def __init__(self, task_manager, mock_data=True) -> None:
        self.node = task_manager
        self.mock_data = mock_data

        self.save_name_client = self.node.create_client(SaveName, SAVE_NAME_TOPIC)
        self.find_seat_client = self.node.create_client(FindSeat, FIND_SEAT_TOPIC)
        self.detect_person_action_client = ActionClient(
            self.node, DetectPerson, DETECT_PERSON_TOPIC
        )

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""
        if not self.save_name_client.wait_for_service(timeout_sec=TIMEOUT):
            self.node.get_logger().warn(
                "Save name service not initialized. (face_recognition)"
            )

        if not self.find_seat_client.wait_for_service(timeout_sec=TIMEOUT):
            self.node.get_logger().warn(
                "Find seat service not initialized. (receptionist_commands)"
            )

        if not self.detect_person_action_client.wait_for_server(timeout_sec=TIMEOUT):
            self.node.get_logger().warn(
                "Detect person action server not initialized. (face_recognition)"
            )

    def success(self, message) -> None:
        """Print success message"""
        self.node.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    @mockable(return_value=100)
    @service_check("save_name_client", -1, TIMEOUT)
    def save_face_name(self, name: str) -> int:
        """Save the name of the person detected"""

        self.node.get_logger().info(f"Saving name: {name}")
        request = SaveName.Request()
        request.name = name

        try:
            future = self.save_name_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            self.node.get_logger().error(f"Error saving name: {e}")
            return self.STATE["EXECUTION_ERROR"]

        self.success(f"Name saved: {name}")
        return self.STATE["EXECUTION_SUCCESS"]

    @mockable(return_value=100)
    @service_check("find_seat_client", 300, TIMEOUT)
    def find_seat(self) -> int:
        """Find an available seat and get the angle for the camera to point at"""

        self.node.get_logger().info("Finding available seat")
        request = FindSeat.Request()
        request.request = True

        try:
            future = self.find_seat_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if result.success:
                self.success(f"Seat found at angle: {result.angle}")
                return result.angle
            else:
                self.node.get_logger().warn("No seat found")
        except Exception as e:
            self.node.get_logger().error(f"Error finding seat: {e}")

        return 300

    @mockable(return_value=True, delay=2)
    @service_check("detect_person_action_client", False, TIMEOUT)
    def detect_person(self) -> bool:
        """Returns true when a person is detected"""

        self.node.get_logger().info("Waiting for person detection")
        goal = DetectPerson.Goal()
        goal.request = True

        try:
            goal_future = self.detect_person_action_client.send_goal_async(goal)
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
                self.success("Person detected")
                return self.STATE["EXECUTION_SUCCESS"]
            else:
                self.node.get_logger().warn("No person detected")
                return self.STATE["EXECUTION_ERROR"]
        except Exception as e:
            self.node.get_logger().error(f"Error detecting person: {e}")
            return self.STATE["EXECUTION_ERROR"]


if __name__ == "__main__":
    rclpy.init()
    node = Node("vision_tasks")
    vision_tasks = VisionTasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

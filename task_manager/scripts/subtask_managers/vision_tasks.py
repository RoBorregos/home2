#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import FindSeat, SaveName
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from utils.decorators import mockable, service_check
from utils.logger import Logger


SAVE_NAME_TOPIC = "/vision/new_name"
FIND_SEAT_TOPIC = "/vision/find_seat"
DETECT_PERSON_TOPIC = "/vision/detect_person"
FOLLOW_TOPIC = "/vision/follow_face"

TIMEOUT = 5.0


class VisionTasks:
    """Class to manage the vision tasks"""

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1,
        "TARGET_NOT_FOUND": 2,
    }
    SERVICES = {"detect_person": 0, "save_face_name": 1, "find_seat": 2}
    SUBTASKS = {
        "RECEPTIONIST": [
            SERVICES["detect_person"],
            SERVICES["find_seat"],
            SERVICES["save_face_name"],
        ],
        "DEMO": [
            SERVICES["detect_person"],
        ],
    }

    def __init__(self, task_manager, task, mock_data=False) -> None:
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
        self.detect_person_action_client = ActionClient(
            self.node, DetectPerson, DETECT_PERSON_TOPIC
        )

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""
        if self.task not in VisionTasks.SUBTASKS:
            Logger.error(self.node, "Task not available")
            return

        if VisionTasks.SERVICES["save_face_name"] in VisionTasks.SUBTASKS[self.task]:
            if not self.save_name_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Save name service not initialized. (face_recognition)")

        if VisionTasks.SERVICES["find_seat"] in VisionTasks.SUBTASKS[self.task]:
            if not self.find_seat_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(
                    self.node,
                    "Find seat service not initialized. (receptionist_commands)",
                )

        if VisionTasks.SERVICES["detect_person"] in VisionTasks.SUBTASKS[self.task]:
            if not self.detect_person_action_client.wait_for_server(timeout_sec=TIMEOUT):
                Logger.warn(
                    self.node,
                    "Detect person action server not initialized. (face_recognition)",
                )

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
    def find_seat(self) -> int:
        """Find an available seat and get the angle for the camera to point at"""

        Logger.info(self.node, "Finding available seat")
        request = FindSeat.Request()
        request.request = True

        try:
            future = self.find_seat_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if result.success:
                Logger.success(self.node, f"Seat found at angle: {result.angle}")
                return result.angle
            else:
                Logger.warn(self.node, "No seat found")
        except Exception as e:
            Logger.error(self.node, f"Error finding seat: {e}")

        return 300

    @mockable(return_value=True, delay=2)
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

#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from frida_interfaces.msg import ObjectDetection
from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import (
    FindSeat,
    SaveName,
    BeverageLocation,
    Query,
    CropQuery,
    DetectPointingObject,
)
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point, PointStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from utils.decorators import mockable, service_check
from utils.logger import Logger
from utils.task import Task
from utils.status import Status
import time

from frida_constants.vision_constants import (
    SAVE_NAME_TOPIC,
    FIND_SEAT_TOPIC,
    CHECK_PERSON_TOPIC,
    FOLLOW_TOPIC,
    FOLLOW_BY_TOPIC,
    QUERY_TOPIC,
    CROP_QUERY_TOPIC,
    BEVERAGE_TOPIC,
    SET_TARGET_TOPIC,
    POINTING_OBJECT_SERVICE,
)
from frida_constants.vision_classes import (
    BBOX,
)

TIMEOUT = 5.0


class VisionTasks:
    """Class to manage the vision tasks"""

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
        self.moondream_query_client = self.node.create_client(Query, QUERY_TOPIC)
        self.moondream_crop_query_client = self.node.create_client(CropQuery, CROP_QUERY_TOPIC)
        self.track_person_client = self.node.create_client(SetBool, SET_TARGET_TOPIC)
        self.pointing_object_client = self.node.create_client(
            DetectPointingObject, POINTING_OBJECT_SERVICE
        )
        self.beverage_location_client = self.node.create_client(BeverageLocation, BEVERAGE_TOPIC)
        self.detect_person_action_client = ActionClient(self.node, DetectPerson, CHECK_PERSON_TOPIC)

        self.services = {
            Task.RECEPTIONIST: {
                "detect_person": {"client": self.detect_person_action_client, "type": "action"},
                "find_seat": {"client": self.find_seat_client, "type": "service"},
                "save_face_name": {
                    "client": self.save_name_client,
                    "type": "service",
                },
                "moondream_query": {"client": self.moondream_query_client, "type": "service"},
                "beverage_location": {"client": self.beverage_location_client, "type": "service"},
                "follow_by_name": {"client": self.follow_by_name_client, "type": "service"},
            },
            Task.HELP_ME_CARRY: {
                # "track_person": {"client": self.track_person_client, "type": "service"},
                # "moondream_crop_query": {
                #     "client": self.moondream_crop_query_client,
                #     "type": "service",
                # },
                "pointing_object": {
                    "client": self.node.create_client(
                        DetectPointingObject, POINTING_OBJECT_SERVICE
                    ),
                    "type": "service",
                },
            },
            Task.DEBUG: {
                "moondream_query": {"client": self.moondream_query_client, "type": "service"},
                "moondream_crop_query": {
                    "client": self.moondream_crop_query_client,
                    "type": "service",
                },
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

    def get_follow_face(self):
        """Get the face to follow"""
        if self.flag_active_face:
            self.flag_active_face = False
            return self.follow_face["x"], self.follow_face["y"]
        else:
            return None, None

    @mockable(return_value=100)
    @service_check(client="save_name_client", return_value=Status.TERMINAL_ERROR, timeout=TIMEOUT)
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
            return Status.EXECUTION_ERROR

        Logger.success(self.node, f"Name saved: {name}")
        return Status.EXECUTION_SUCCESS

    @mockable(return_value=100)
    @service_check("find_seat_client", [Status.EXECUTION_ERROR, 300], TIMEOUT)
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
                return Status.TARGET_NOT_FOUND, 300

        except Exception as e:
            Logger.error(self.node, f"Error finding seat: {e}")
            return Status.EXECUTION_ERROR, 300

        Logger.success(self.node, f"Seat found: {result.angle}")
        return Status.EXECUTION_SUCCESS, result.angle

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=2, mock=False)
    @service_check("detect_person_action_client", Status.EXECUTION_ERROR, TIMEOUT)
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
                return Status.EXECUTION_SUCCESS
            else:
                Logger.warn(self.node, "No person detected")
                return Status.TARGET_NOT_FOUND
        except Exception as e:
            Logger.error(self.node, f"Error detecting person: {e}")
            return Status.EXECUTION_ERROR

    @mockable(return_value=True, delay=2)
    def detect_guest(self, name: str, timeout: float = TIMEOUT):
        """Returns true when the guest is detected"""
        pass

    @mockable(return_value=True, delay=2)
    @service_check("beverage_location_client", [Status.EXECUTION_ERROR, ""], TIMEOUT)
    def find_drink(self, drink: str, timeout: float = TIMEOUT) -> tuple[int, str]:
        """Find if a drink is available and location"""
        Logger.info(self.node, f"Finding drink: {drink}")
        request = BeverageLocation.Request()
        request.beverage = drink

        try:
            future = self.beverage_location_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No drink found")
                return Status.TARGET_NOT_FOUND, "not found"

        except Exception as e:
            Logger.error(self.node, f"Error finding drink: {e}")
            return Status.EXECUTION_ERROR, "not found"

        Logger.success(self.node, f"Found drink: {drink}")
        return Status.EXECUTION_SUCCESS, result.location

    @mockable(return_value=(Status.EXECUTION_ERROR, ""), delay=5, mock=False)
    @service_check("moondream_query_client", Status.EXECUTION_ERROR, TIMEOUT)
    def moondream_query(self, prompt: str, query_person: bool = False) -> tuple[int, str]:
        """Makes a query of the current image using moondream."""
        Logger.info(self.node, f"Querying image with prompt: {prompt}")
        request = Query.Request()
        request.query = prompt
        request.person = query_person

        try:
            future = self.moondream_query_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No result generated")
                return Status.EXECUTION_ERROR, ""

        except Exception as e:
            Logger.error(self.node, f"Error requesting description: {e}")
            return Status.EXECUTION_ERROR, ""

        Logger.success(self.node, f"Result: {result.result}")
        return Status.EXECUTION_SUCCESS, result.result

    @mockable(return_value=(Status.EXECUTION_ERROR, ""), delay=5, mock=False)
    @service_check("moondream_crop_query_client", Status.EXECUTION_ERROR, TIMEOUT)
    def moondream_crop_query(self, prompt: str, bbox: BBOX) -> tuple[int, str]:
        """Makes a query of the current image using moondream."""
        Logger.info(self.node, f"Querying image with prompt: {prompt}")
        request = CropQuery.Request()
        request.query = prompt
        request.ymin = float(bbox.y1)
        request.xmin = float(bbox.x1)
        request.ymax = float(bbox.y2)
        request.xmax = float(bbox.x2)

        try:
            future = self.moondream_crop_query_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No result generated")
                return Status.EXECUTION_ERROR, ""

        except Exception as e:
            Logger.error(self.node, f"Error requesting description: {e}")
            return Status.EXECUTION_ERROR, ""

        Logger.success(self.node, f"Result: {result.result}")
        return Status.EXECUTION_SUCCESS, result.result

    @mockable(return_value=(Status.EXECUTION_ERROR, ""), delay=5, mock=False)
    @service_check("moondream_query_client", Status.EXECUTION_ERROR, TIMEOUT)
    def moondream_query_async(self, prompt: str, query_person: bool = False, callback=None):
        """Makes a query of the current image using moondream. Runs asynchronously."""
        Logger.info(self.node, f"Querying image with prompt: {prompt}")
        request = Query.Request()
        request.query = prompt
        request.person = query_person

        try:
            future = self.moondream_query_client.call_async(request)
            future.add_done_callback(lambda fut: self._handle_moondream_response(fut, callback))

        except Exception as e:
            Logger.error(self.node, f"Error requesting description: {e}")

    def _handle_moondream_response(self, future, callback):
        """Callback to handle the response from the description service."""
        try:
            result = future.result()
            if result.success:
                Logger.success(self.node, f"Result: {result.result}")
                if callback:
                    callback(Status.EXECUTION_SUCCESS, result.result)
            else:
                Logger.warn(self.node, "No result generated")
                if callback:
                    callback(Status.EXECUTION_ERROR, "")
        except Exception as e:
            Logger.error(self.node, f"Error processing response: {e}")
            if callback:
                callback(Status.EXECUTION_ERROR, "")

    @mockable(return_value=None, delay=2)
    @service_check("follow_by_name_client", Status.EXECUTION_ERROR, TIMEOUT)
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
            return Status.EXECUTION_ERROR

        Logger.success(self.node, f"Following face success: {name}")
        return Status.EXECUTION_SUCCESS

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=2)
    @service_check("track_person_client", Status.EXECUTION_ERROR, TIMEOUT)
    def track_person(self):
        """Track the person in the image"""
        Logger.info(self.node, "Tracking person")
        request = SetBool.Request()
        request.data = True

        try:
            future = self.track_person_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error tracking person: {e}")
            return Status.EXECUTION_ERROR

        Logger.success(self.node, "Person tracking success")
        return Status.EXECUTION_SUCCESS

    def describe_person(self, callback):
        """Describe the person in the image"""
        Logger.info(self.node, "Describing person")
        prompt = "Describe the person in the image"
        self.moondream_query_async(prompt, query_person=True, callback=callback)

    def get_pointing_bag(self, timeout: float = TIMEOUT) -> tuple[int, ObjectDetection]:
        time.sleep(TIMEOUT)
        """Get the bag in the image"""
        Logger.info(self.node, "Getting bag in the image")
        request = DetectPointingObject.Request()

        try:
            future = self.pointing_object_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            result = future.result()

            if not result.success:
                Logger.warn(self.node, "No bag found")
                return Status.TARGET_NOT_FOUND, BBOX(), PointStamped()
        except Exception as e:
            Logger.error(self.node, f"Error getting bag: {e}")
            return Status.EXECUTION_ERROR, BBOX(), PointStamped
        Logger.success(self.node, f"Bag found: {result.detection}")
        bbox = BBOX()
        bbox.x1 = result.detection.xmin
        bbox.x2 = result.detection.xmax
        bbox.y1 = result.detection.ymin
        bbox.y2 = result.detection.ymax
        return Status.EXECUTION_SUCCESS, bbox, result.detection.point3d

    def describe_bag(self, bbox: BBOX) -> tuple[int, str]:
        """Describe the person in the image"""
        Logger.info(self.node, "Describing the bag")
        prompt = "Please describe the bag in the image"
        return self.moondream_crop_query(prompt, bbox)

    def describe_bag_moondream(self):
        """Describe the bag using only moondream"""
        Logger.info(self.node, "Describing bag")
        prompt = "Describe the bag that the person is pointing at using the folling format: the bag on your left is small and green"
        return Status.EXECUTION_SUCCESS, self.moondream_query(prompt, query_person=False)


if __name__ == "__main__":
    rclpy.init()
    node = Node("vision_tasks")
    vision_tasks = VisionTasks(node, task="RECEPTIONIST")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

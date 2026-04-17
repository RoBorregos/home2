#!/usr/bin/env python3
"""

Navigation Area SubTask Manager

"""

import json
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from frida_constants.navigation_constants import (
    AREAS_SERVICE,
    CHECK_DOOR_SERVICE,
    MOVE_LOCATION_SERVICE,
    MOVE_DISTANCE_SERVICE,
    MOVE_UNTIL_OBJECT_SERVICE,
    MOVE_LOCATION_IGNORE_OBSTACLES_SERVICE,
    SUBTASK_MANAGER,
)
from frida_interfaces.srv import (
    CheckDoor,
    MapAreas,
    MoveLocation,
    MoveDistance,
    MoveUntilObject,
    MoveLocationIgnoreObstacles,
)

from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.task import Task


class NavigationTasks:
    """Class to manage the navigation tasks"""

    def __init__(self, task_manager: Node, task: Task, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data
        self.task = task

        # Action clients and services
        self.door_checking_srv = self.node.create_client(CheckDoor, CHECK_DOOR_SERVICE)
        self.retrieve_areas_srv = self.node.create_client(MapAreas, AREAS_SERVICE)
        self.move_to_location_srv = self.node.create_client(MoveLocation, MOVE_LOCATION_SERVICE)
        self.move_distance_srv = self.node.create_client(MoveDistance, MOVE_DISTANCE_SERVICE)
        self.move_until_object_srv = self.node.create_client(
            MoveUntilObject, MOVE_UNTIL_OBJECT_SERVICE
        )
        self.move_location_ignore_obstacles_srv = self.node.create_client(
            MoveLocationIgnoreObstacles, MOVE_LOCATION_IGNORE_OBSTACLES_SERVICE
        )

        # Task Actions and Services check
        self.services = {
            Task.DEBUG: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
                "move_to_location_srv": {"client": self.move_to_location_srv, "type": "service"},
                "move_distance_srv": {"client": self.move_distance_srv, "type": "service"},
                "move_until_object_srv": {"client": self.move_until_object_srv, "type": "service"},
                "move_location_ignore_obstacles_srv": {
                    "client": self.move_location_ignore_obstacles_srv,
                    "type": "service",
                },
            },
        }

        self.setup_backup_map()

        if not self.mock_data:
            self.setup_services()

    def setup_backup_map(self):
        """Load backup map info"""
        try:
            package_share_directory = get_package_share_directory("frida_constants")
            file_path = os.path.join(package_share_directory, "map_areas/areas.json")
            with open(file_path, "r") as file:
                data = json.load(file)
            if data is not None:
                self.areas_backup = data
                CLog.nav(self.node, "INFO", "Areas Json BackUp Loaded")
            else:
                raise Exception("Data is empty")
        except Exception as e:
            self.areas_backup = Status.EXECUTION_ERROR
            CLog.nav(self.node, "WARN", f"Areas Json Backup Failed Error: {e}")

    def setup_services(self):
        """Initialize services and actions"""

        if self.task not in self.services:
            CLog.nav(self.node, "ERROR", "Task not available")
            return

        for key, service in self.services[self.task].items():
            if service["type"] == "service":
                if not service["client"].wait_for_service(
                    timeout_sec=SUBTASK_MANAGER.SERVICE_TIMEOUT.value
                ):
                    CLog.nav(self.node, "WARN", f"{key} service not initialized. ({self.task})")
            elif service["type"] == "action":
                if not service["client"].wait_for_server(
                    timeout_sec=SUBTASK_MANAGER.SERVICE_TIMEOUT.value
                ):
                    CLog.nav(
                        self.node, "WARN", f"{key} action server not initialized. ({self.task})"
                    )

    @mockable(return_value=lambda self: (Status.EXECUTION_SUCCESS, self.areas_backup), delay=1)
    @service_check(
        "retrieve_areas_srv",
        lambda self: (Status.EXECUTION_ERROR, self.areas_backup),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def retrieve_areas(self):
        """Dump areas.json dynamically from areas service"""

        req = MapAreas.Request()
        future = self.retrieve_areas_srv.call_async(req)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SUBTASK_MANAGER.AREAS_RETRIEVE_TIMEOUT.value
        )
        if (future.result() is None) or (future.result().areas == ""):
            CLog.nav(self.node, "ERROR", "Service return empty data")
            return (Status.EXECUTION_ERROR, self.areas_backup)

        else:
            CLog.nav(self.node, "INFO", "Map Areas dumped Succesfully")
            return (Status.EXECUTION_SUCCESS, json.loads(str(future.result().areas)))

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check(
        "door_checking_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def check_door(self):
        """Check if the door is open or closed"""

        request = CheckDoor.Request()
        future = self.door_checking_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is not None:
            if result.status:
                CLog.nav(self.node, "SUCCESS", "Door open")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", "Error getting state with door")
                return (Status.EXECUTION_ERROR, "Error getting door state")
        else:
            CLog.nav(self.node, "ERROR", "Error with request")
            return (Status.EXECUTION_ERROR, "Request error")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "move_to_location_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_to_location(self, location, sublocation):
        """Move to areas json location"""
        if sublocation == "":
            sublocation = "safe_place"
        CLog.nav(self.node, "MOVE", f"Requesting navigation to {location} - {sublocation}")
        request = MoveLocation.Request()
        request.location = location
        request.sublocation = sublocation
        future = self.move_to_location_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", f"Goal {location} reached")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Goal failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        else:
            CLog.nav(self.node, "ERROR", "Service request failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")

    # -----------------------------------------------------------------
    #  Open-loop base motion (bypass Nav2, ignores obstacles)
    # -----------------------------------------------------------------
    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=2)
    @service_check(
        "move_distance_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_forward(self, distance: float):
        """Move the base a fixed distance forward (meters).

        Positive = forward, negative = backward.  Uses /cmd_vel directly;
        Nav2 is *not* involved so obstacles are ignored.
        """
        CLog.nav(self.node, "MOVE", f"Requesting move_forward of {distance:.3f} m")
        request = MoveDistance.Request()
        request.distance = float(distance)
        future = self.move_distance_srv.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SUBTASK_MANAGER.MOVE_DISTANCE_TIMEOUT.value
        )
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", f"Move of {distance:.3f} m completed")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Move failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        else:
            CLog.nav(self.node, "ERROR", "Service request failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=2)
    @service_check(
        "move_distance_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_backwards(self, distance: float):
        """Move the base backward by `distance` metres (positive value).

        Internally sends a negative distance to the MoveDistance service.
        Uses /cmd_vel directly; Nav2 is *not* involved so obstacles are ignored.
        """
        dist = -abs(distance)
        CLog.nav(self.node, "MOVE", f"Requesting move_backwards of {abs(distance):.3f} m")
        request = MoveDistance.Request()
        request.distance = float(dist)
        future = self.move_distance_srv.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SUBTASK_MANAGER.MOVE_DISTANCE_TIMEOUT.value
        )
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", f"Move backwards {abs(distance):.3f} m completed")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Move backwards failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        else:
            CLog.nav(self.node, "ERROR", "Service request failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")

    # -----------------------------------------------------------------
    #  Drive until lidar detects an obstacle within stop_distance
    # -----------------------------------------------------------------
    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check(
        "move_until_object_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_backwards_until_object(self, stop_distance: float = 0.30):
        """Drive backward at low speed until the rear lidar sector detects an
        obstacle within *stop_distance* metres. The robot stops immediately
        once the threshold is reached.

        Uses /cmd_vel directly; Nav2 is *not* involved so the costmap obstacle
        layers are bypassed.
        """
        CLog.nav(
            self.node,
            "MOVE",
            f"Requesting move_backwards_until_object (stop_dist={stop_distance:.2f} m)",
        )
        request = MoveUntilObject.Request()
        request.stop_distance = float(stop_distance)
        request.forward = False
        future = self.move_until_object_srv.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SUBTASK_MANAGER.MOVE_UNTIL_OBJECT_TIMEOUT.value
        )
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(
                    self.node,
                    "SUCCESS",
                    f"Stopped near object (measured {result.measured_distance:.3f} m)",
                )
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Move until object failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        else:
            CLog.nav(self.node, "ERROR", "Service request failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")

    # -----------------------------------------------------------------
    #  Nav2 goal with rear-lidar & camera obstacles ignored
    # -----------------------------------------------------------------
    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "move_location_ignore_obstacles_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_to_location_ignore_obstacles(
        self,
        location: str,
        sublocation: str = "",
        ignore_camera: bool = True,
        ignore_rear_lidar: bool = True,
    ):
        """Navigate to a map area using Nav2 but with the camera obstacle
        layer and / or the rear sector of the lidar temporarily disabled.

        Front-of-robot lidar detection and the static map are still active,
        so the robot will avoid frontal collisions but can back into tight
        spaces that the camera or rear-lidar would otherwise block.
        """
        if sublocation == "":
            sublocation = "safe_place"
        CLog.nav(
            self.node,
            "MOVE",
            f"Requesting move_to_location_ignore_obstacles -> {location}/{sublocation} "
            f"(cam={ignore_camera}, rear_lidar={ignore_rear_lidar})",
        )
        request = MoveLocationIgnoreObstacles.Request()
        request.location = location
        request.sublocation = sublocation
        request.ignore_camera = ignore_camera
        request.ignore_rear_lidar = ignore_rear_lidar
        future = self.move_location_ignore_obstacles_srv.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=SUBTASK_MANAGER.MOVE_LOCATION_IGNORE_OBSTACLES_TIMEOUT.value,
        )
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", f"Goal {location} reached (obstacles ignored)")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Goal failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        else:
            CLog.nav(self.node, "ERROR", "Service request failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")


if __name__ == "__main__":
    rclpy.init()
    node = Node("Navigation_Task_Manager")
    navigationTaskManager = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

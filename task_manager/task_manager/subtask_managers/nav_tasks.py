#!/usr/bin/env python3
"""

Navigation Area SubTask Manager

"""

import json
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from frida_constants.navigation_constants import (
    AREAS_SERVICE,
    CHECK_DOOR_SERVICE,
    MOVE_LOCATION_SERVICE,
    NAV_QUERY_SERVICE,
    DOCK_TABLE_SERVICE,
    SUBTASK_MANAGER,
)
from frida_interfaces.srv import CheckDoor, MapAreas, MoveLocation, NavQuery, DockTable

from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.task import Task

NAV_GOAL_TIMEOUT = 90.0


def _mock_pose():
    """Build a valid placeholder PoseStamped for mocked navigation."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    return pose


def _get_adaptive_bt_path():
    """Get the full installed path for the adaptive behavior tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import os

        return os.path.join(
            get_package_share_directory("nav_main"), "bt", "navigate_adaptive_goal.xml"
        )
    except Exception:
        return ""


ADAPTIVE_BT = _get_adaptive_bt_path()


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
        self.nav_query_srv = self.node.create_client(NavQuery, NAV_QUERY_SERVICE)
        self.dock_table_srv = self.node.create_client(DockTable, DOCK_TABLE_SERVICE)

        # Task Actions and Services check
        self.services = {
            Task.DEBUG: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
                "move_to_location_srv": {"client": self.move_to_location_srv, "type": "service"},
                "nav_query_srv": {"client": self.nav_query_srv, "type": "service"},
                "dock_table_srv": {"client": self.dock_table_srv, "type": "service"},
            },
            Task.RESTAURANT: {
                "go_to_pose_srv": {"client": self.go_to_pose_srv, "type": "service"},
                "get_robot_pose_srv": {"client": self.get_robot_pose_srv, "type": "service"},
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

    # ── Existing methods (pre-mapped environments) ──

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

    @mockable(return_value=(Status.EXECUTION_SUCCESS, {"distance": 5.0}), delay=1)
    @service_check(
        "nav_query_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def get_path_info(self, location_b, sublocation_b, location_a="", sublocation_a=""):
        """Query the real path distance (m) to a map area without moving the
        robot. Empty location_a starts from the robot's current pose.
        Returns (Status, {"distance": float})."""
        if sublocation_b == "":
            sublocation_b = "safe_place"
        if location_a != "" and sublocation_a == "":
            sublocation_a = "safe_place"
        origin = f"{location_a} - {sublocation_a}" if location_a else "current pose"
        CLog.nav(
            self.node,
            "INFO",
            f"Querying path from {origin} to {location_b} - {sublocation_b}",
        )
        request = NavQuery.Request()
        request.location_a = location_a
        request.sublocation_a = sublocation_a
        request.location_b = location_b
        request.sublocation_b = sublocation_b
        future = self.nav_query_srv.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=SUBTASK_MANAGER.NAV_QUERY_TIMEOUT.value
        )
        result = future.result()
        if result is None:
            CLog.nav(self.node, "ERROR", "Path query failed (None result)")
            return (Status.EXECUTION_ERROR, "Error with request")
        if not result.success:
            CLog.nav(self.node, "ERROR", f"Path query failed: {result.error}")
            return (Status.EXECUTION_ERROR, result.error)
        CLog.nav(
            self.node,
            "SUCCESS",
            f"Path to {location_b}: {result.distance_meters:.2f} m",
        )
        return (Status.EXECUTION_SUCCESS, {"distance": result.distance_meters})

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check(
        "dock_table_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def dock_table(self, offset=0.0):
        """Perpendicular-approach (dock to) the table/shelf in front of the robot.

        offset: desired front offset in meters; 0.0 uses the docker default.
        """
        CLog.nav(self.node, "MOVE", f"Requesting table docking (offset={offset})")
        request = DockTable.Request()
        request.offset = float(offset)
        future = self.dock_table_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", "Docked to table")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Docking failed: {result.error}")
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

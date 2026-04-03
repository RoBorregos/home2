#!/usr/bin/env python3
"""

Navigation Area SubTask Manager

"""

import json
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from frida_constants.navigation_constants import AREAS_SERVICE, CHECK_DOOR_SERVICE, SUBTASK_MANAGER
from frida_interfaces.srv import (
    CheckDoor,
    MapAreas,
)

from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.logger import Logger
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

        # Task Actions and Services check
        self.services = {
            Task.DEBUG: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
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
                Logger.info(self.node, "Areas Json BackUp Loaded")
            else:
                raise Exception("Data is empty")
        except Exception as e:
            self.areas_backup = Status.EXECUTION_ERROR
            Logger.warn(self.node, f"Areas Json Backup Failed Error: {e}")

    def setup_services(self):
        """Initialize services and actions"""

        if self.task not in self.services:
            Logger.error(self.node, "Task not available")
            return

        for key, service in self.services[self.task].items():
            if service["type"] == "service":
                if not service["client"].wait_for_service(
                    timeout_sec=SUBTASK_MANAGER.SERVICE_TIMEOUT.value
                ):
                    Logger.warn(self.node, f"{key} service not initialized. ({self.task})")
            elif service["type"] == "action":
                if not service["client"].wait_for_server(
                    timeout_sec=SUBTASK_MANAGER.SERVICE_TIMEOUT.value
                ):
                    Logger.warn(self.node, f"{key} action server not initialized. ({self.task})")

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
            Logger.error(self.node, "Service return empty data")
            return (Status.EXECUTION_ERROR, self.areas_backup)

        else:
            Logger.info(self.node, "Map Areas dumped Succesfully")
            return (Status.EXECUTION_SUCCESS,json.loads(str(future.result().areas)))

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ''), delay=3)
    @service_check(
        "door_checking_srv", (Status.EXECUTION_ERROR,' "Service not started'), timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value
    )
    def check_door(self):
        """Check if the door is open or closed"""

        request = CheckDoor.Request()
        future = self.door_checking_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is not None:
            if result.status:
                Logger.info(self.node, "Door open")
                return (Status.EXECUTION_SUCCESS, '')
            else:
                Logger.error(self.node, "Error getting state with door")
                return (Status.EXECUTION_ERROR, 'Error getting door state')
        else:
            Logger.error(self.node, "Error with request")
            return (Status.EXECUTION_ERROR, 'Request error')


if __name__ == "__main__":
    rclpy.init()
    node = Node("Navigation_Task_Manager")
    navigationTaskManager = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

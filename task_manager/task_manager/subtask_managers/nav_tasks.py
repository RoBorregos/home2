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
    SUBTASK_MANAGER,
    RTAB_PAUSE_SERVICE,
    RTAB_RESUME_SERVICE,
)
from frida_interfaces.srv import CheckDoor, MapAreas, MoveLocation
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Empty, SetBool

from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.colored_logger import CLog
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

TIMEOUT = 10.0


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

        # Additional clients for legacy/special purposes
        self.rtabmap_pause = self.node.create_client(Empty, RTAB_PAUSE_SERVICE)
        self.rtabmap_continue = self.node.create_client(Empty, RTAB_RESUME_SERVICE)
        self.bt_params = self.node.create_client(SetParameters, "/bt_navigator/set_parameters")
        self.bt_lifecycle = self.node.create_client(ChangeState, "/bt_navigator/change_state")
        self.activate_follow = self.node.create_client(SetBool, "/activate_follow")

        # Task Actions and Services check
        self.services = {
            Task.DEBUG: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
                "move_to_location_srv": {"client": self.move_to_location_srv, "type": "service"},
            },
            Task.RESTAURANT: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
                "move_to_location_srv": {"client": self.move_to_location_srv, "type": "service"},
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
            self.areas_backup = {}
            Logger.warn(self.node, f"Areas Json Backup Failed Error: {e}")

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
    @service_check("rtabmap_pause", (Status.EXECUTION_ERROR, "Service not started"), timeout=3)
    def pause_nav(self):
        req = Empty.Request()
        future = self.rtabmap_pause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "pause_nav service call failed (no response)")
            return (Status.EXECUTION_ERROR, "Timeout")
        Logger.info(self.node, "pause_nav successful")
        return (Status.EXECUTION_SUCCESS, "")

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
        """Move to areas json location using MoveLocation service"""
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

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check("activate_follow", (Status.EXECUTION_ERROR, "Service not started"), TIMEOUT)
    def follow_person(self, activate: bool):
        """Activate or deactivate the follow person mode"""
        try:
            request = SetBool.Request()
            request.data = activate
            future = self.activate_follow.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()
            if result is None or not result.success:
                raise Exception("Service call failed")
        except Exception as e:
            Logger.info(self.node, f"Error sending follow person mode: {e}")
            return (Status.EXECUTION_ERROR, str(e))
        Logger.info(self.node, f"Follow person {'activated' if activate else 'deactivated'}")
        return (Status.EXECUTION_SUCCESS, "")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check("bt_params", (Status.EXECUTION_ERROR, "Service not started"), TIMEOUT)
    def change_bt(self, bt_str: str):
        """Change the behavior tree on bt_navigator"""
        try:
            param_name = "default_nav_to_pose_bt_xml"
            if bt_str == "follow":
                param_value = "/workspace/src/navigation/packages/nav_main/bt/follow_dynamic.xml"
            elif bt_str == "adaptive":
                param_value = (
                    "/workspace/src/navigation/packages/nav_main/bt/navigate_adaptive_goal.xml"
                )
            else:
                param_value = "/workspace/src/navigation/packages/nav_main/bt/navigate_to_pose_w_replanning_and_recovery.xml"

            param = Parameter()
            param.name = param_name
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=param_value
            )

            param_request = SetParameters.Request()
            param_request.parameters = [param]
            param_future = self.bt_params.call_async(param_request)
            rclpy.spin_until_future_complete(self.node, param_future)

            if param_future.result() is None:
                raise Exception("Parameter service call failed")

            # Lifecycle toggle
            deactivate_req = ChangeState.Request()
            deactivate_req.transition.id = Transition.TRANSITION_DEACTIVATE
            f1 = self.bt_lifecycle.call_async(deactivate_req)
            rclpy.spin_until_future_complete(self.node, f1)

            activate_req = ChangeState.Request()
            activate_req.transition.id = Transition.TRANSITION_ACTIVATE
            f2 = self.bt_lifecycle.call_async(activate_req)
            rclpy.spin_until_future_complete(self.node, f2)

            Logger.info(self.node, f"Successfully changed BT to: {bt_str}")
            return (Status.EXECUTION_SUCCESS, "")
        except Exception as e:
            Logger.error(self.node, f"Error changing behavior tree: {e}")
            return (Status.EXECUTION_ERROR, str(e))

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=3)
    @service_check("door_checking_srv", (Status.EXECUTION_ERROR, "Service not started"), TIMEOUT)
    def check_door(self):
        """Check if the door is open or closed"""
        request = CheckDoor.Request()
        future = self.door_checking_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result is not None:
            if result.status:
                Logger.info(self.node, "Door open")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                Logger.error(self.node, "Error getting state with door")
                return (Status.EXECUTION_ERROR, "Door closed or error")
        else:
            Logger.error(self.node, "Error with request")
            return (Status.EXECUTION_ERROR, "Request error")


if __name__ == "__main__":
    rclpy.init()
    node = Node("Navigation_Task_Manager")
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

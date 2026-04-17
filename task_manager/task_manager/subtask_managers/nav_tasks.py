#!/usr/bin/env python3
"""

Navigation Area SubTask Manager

"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PointStamped
from frida_constants.navigation_constants import (
    AREAS_SERVICE,
    CHECK_DOOR_SERVICE,
    MOVE_LOCATION_SERVICE,
    GO_TO_POSE_SERVICE,
    GET_ROBOT_POSE_SERVICE,
    SUBTASK_MANAGER,
)
from frida_interfaces.srv import CheckDoor, MapAreas, MoveLocation, GoToPose, GetRobotPose

from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.task import Task

NAV_GOAL_TIMEOUT = 90.0

def _get_adaptive_bt_path():
    """Get the full installed path for the adaptive behavior tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        return os.path.join(
            get_package_share_directory('nav_main'), 'bt', 'navigate_adaptive_goal.xml'
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
        self.go_to_pose_srv = self.node.create_client(GoToPose, GO_TO_POSE_SERVICE)
        self.get_robot_pose_srv = self.node.create_client(GetRobotPose, GET_ROBOT_POSE_SERVICE)

        # Task Actions and Services check
        self.services = {
            Task.DEBUG: {
                "door_checking_srv": {"client": self.door_checking_srv, "type": "service"},
                "retrieve_areas_srv": {"client": self.retrieve_areas_srv, "type": "service"},
                "move_to_location_srv": {"client": self.move_to_location_srv, "type": "service"},
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

    # ── Point-based navigation methods (unmapped / restaurant) ──

    @mockable(return_value=(Status.EXECUTION_SUCCESS, None), delay=1)
    @service_check(
        "get_robot_pose_srv",
        (Status.EXECUTION_ERROR, None),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def get_current_pose(self):
        """Get current robot pose via TF (map -> base_link)."""
        CLog.nav(self.node, "INFO", "Requesting current robot pose")
        request = GetRobotPose.Request()
        future = self.get_robot_pose_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result()
        if result is not None and result.success:
            CLog.nav(self.node, "SUCCESS", "Robot pose retrieved")
            return (Status.EXECUTION_SUCCESS, result.pose)
        error = result.error if result is not None else "Service returned None"
        CLog.nav(self.node, "ERROR", f"Failed to get pose: {error}")
        return (Status.EXECUTION_ERROR, None)

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "go_to_pose_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_to_pose(self, pose, behavior_tree=""):
        """Navigate to an arbitrary PoseStamped goal."""
        CLog.nav(
            self.node, "MOVE",
            f"Requesting navigation to pose ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})",
        )
        request = GoToPose.Request()
        request.target_pose = pose
        request.behavior_tree = behavior_tree
        future = self.go_to_pose_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=NAV_GOAL_TIMEOUT)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", "Pose goal reached")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Pose goal failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        CLog.nav(self.node, "ERROR", "GoToPose service returned None")
        return (Status.EXECUTION_ERROR, "Service timeout")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "go_to_pose_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def move_to_point(self, point, standoff_distance=2.0):
        """Navigate near a PointStamped, stopping `standoff_distance` meters away.

        Sends the raw point (in its original frame) to nav_central, which
        transforms it to map frame and applies the standoff offset there.
        Uses the adaptive BT for goals that may be inside obstacle cells.

        Args:
            point: Target PointStamped (may be in camera or map frame).
            standoff_distance: How far from the target to stop (meters).
        """
        CLog.nav(
            self.node, "MOVE",
            f"Requesting navigation to point ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f}), "
            f"frame={point.header.frame_id}, standoff={standoff_distance}m",
        )

        goal = PoseStamped()
        goal.header.frame_id = point.header.frame_id if point.header.frame_id else "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = point.point.x
        goal.pose.position.y = point.point.y
        goal.pose.position.z = point.point.z
        goal.pose.orientation.w = 1.0  # orientation computed after transform in nav_central

        request = GoToPose.Request()
        request.target_pose = goal
        # Encode standoff in behavior_tree field: "bt_path|standoff=X.X"
        request.behavior_tree = f"{ADAPTIVE_BT}|standoff={standoff_distance}"
        future = self.go_to_pose_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=NAV_GOAL_TIMEOUT)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", "Point goal reached")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Point goal failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        CLog.nav(self.node, "ERROR", "GoToPose service returned None")
        return (Status.EXECUTION_ERROR, "Service timeout")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "go_to_pose_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def explore_zone(self, distance):
        """Move forward `distance` meters in the current heading direction.

        Used for progressive customer search in unmapped environments.
        """
        CLog.nav(self.node, "MOVE", f"Exploring zone: {distance}m ahead")
        status, current_pose = self.get_current_pose()
        if status != Status.EXECUTION_SUCCESS or current_pose is None:
            CLog.nav(self.node, "ERROR", "Cannot explore: failed to get current pose")
            return (Status.EXECUTION_ERROR, "Failed to get current pose")

        q = current_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = current_pose.pose.position.x + distance * math.cos(yaw)
        goal.pose.position.y = current_pose.pose.position.y + distance * math.sin(yaw)
        goal.pose.position.z = 0.0
        goal.pose.orientation = current_pose.pose.orientation  # keep same heading

        request = GoToPose.Request()
        request.target_pose = goal
        request.behavior_tree = ""  # default BT — goal is in free space
        future = self.go_to_pose_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=NAV_GOAL_TIMEOUT)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", f"Explored {distance}m ahead")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Explore failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        CLog.nav(self.node, "ERROR", "GoToPose service returned None")
        return (Status.EXECUTION_ERROR, "Service timeout")

    @mockable(return_value=(Status.EXECUTION_SUCCESS, ""), delay=5)
    @service_check(
        "go_to_pose_srv",
        (Status.EXECUTION_ERROR, "Service not started"),
        timeout=SUBTASK_MANAGER.SERVICE_TIMEOUT.value,
    )
    def return_to_origin(self, inverse_orientation=False):
        """Navigate back to the map origin (0, 0, 0).

        Args:
            inverse_orientation: If True, face backward (yaw=pi) so robot
                faces the barman when arriving. If False, face forward (yaw=0).
        """
        yaw = math.pi if inverse_orientation else 0.0
        CLog.nav(
            self.node, "MOVE",
            f"Returning to origin (inverse={inverse_orientation})",
        )

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        request = GoToPose.Request()
        request.target_pose = goal
        request.behavior_tree = ""  # default BT — origin is known free space
        future = self.go_to_pose_srv.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=NAV_GOAL_TIMEOUT)
        result = future.result()
        if result is not None:
            if result.success:
                CLog.nav(self.node, "SUCCESS", "Returned to origin")
                return (Status.EXECUTION_SUCCESS, "")
            else:
                CLog.nav(self.node, "ERROR", f"Return to origin failed: {result.error}")
                return (Status.EXECUTION_ERROR, result.error)
        CLog.nav(self.node, "ERROR", "GoToPose service returned None")
        return (Status.EXECUTION_ERROR, "Service timeout")


if __name__ == "__main__":
    rclpy.init()
    node = Node("Navigation_Task_Manager")
    navigationTaskManager = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

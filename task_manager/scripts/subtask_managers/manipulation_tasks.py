#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger

from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import GetJoints, FollowFace, GetOptimalPositionForPlane
from frida_constants.xarm_configurations import XARM_CONFIGURATIONS
from rclpy.action import ActionClient
from typing import List, Union
from utils.decorators import mockable, service_check
from utils.status import Status
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
from geometry_msgs.msg import PointStamped, PoseStamped

# from utils.decorators import service_check
from xarm_msgs.srv import SetDigitalIO

from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
)
# import time as t

XARM_ENABLE_SERVICE = "/xarm/motion_enable"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"

TIMEOUT = 5.0

RAD_TO_DEG = 180 / 3.14159265359
DEG_TO_RAD = 3.14159265359 / 180


class ManipulationTasks:
    """Class to manage the vision tasks"""

    # STATE = {
    #     "TERMINAL_ERROR": -1,
    #     "EXECUTION_ERROR": 0,
    #     "EXECUTION_SUCCESS": 1,
    #     "TARGET_NOT_FOUND": 2,
    # }

    SUBTASKS = {
        "RECEPTIONIST": [],
        "RESTAURANT": [],
        "SERVE_BREAKFAST": [],
        "STORING_GROCERIES": [],
        "STICKLER_RULES": [],
        "DEMO": [],
    }

    def __init__(self, task_manager, task, mock_data=False) -> None:
        """Initialize the class"""

        if not isinstance(task_manager, Node):
            raise ValueError("task_manager must be a Node")

        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        # simulation = 1
        self.node.declare_parameter("cancel_after_secs", 5.0)

        self._move_joints_action_client = ActionClient(
            self.node, MoveJoints, "/manipulation/move_joints_action_server"
        )

        self.gripper_client = self.node.create_client(SetDigitalIO, "/xarm/set_tgpio_digital")

        self._get_joints_client = self.node.create_client(GetJoints, "/manipulation/get_joints")
        self.follow_face_client = self.node.create_client(FollowFace, "/follow_face")
        self.follow_person_client = self.node.create_client(FollowFace, "/follow_person")
        self._manipulation_action_client = ActionClient(
            self.node, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self._fix_position_to_plane_client = self.node.create_client(
            GetOptimalPositionForPlane,
            "/manipulation/get_optimal_position_for_plane",
        )

    def open_gripper(self):
        """Opens the gripper"""
        return self._set_gripper_state("open")

    def close_gripper(self):
        """Closes the gripper"""
        return self._set_gripper_state("close")

    def follow_person(self, follow: bool) -> int:
        """Save the name of the person detected"""

        if follow:
            Logger.info(self.node, "Following face")
        else:
            Logger.info(self.node, "Stopping following face")
        request = FollowFace.Request()
        request.follow_face = follow

        try:
            future = self.follow_person_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
            result = future.result()
            if result is None:
                raise Exception("Timeout Exceed")
            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error following person: {e}")
            return Status.EXECUTION_ERROR

        Logger.success(self.node, "Following person request successful")
        return Status.EXECUTION_SUCCESS

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check("gripper_client", Status.EXECUTION_ERROR, TIMEOUT)
    def _set_gripper_state(self, state: str):
        """
        Controls the gripper state.
        State: 'open' o 'close'
        """
        try:
            # if not self.gripper_client.wait_for_service(timeout_sec=TIMEOUT):
            #     Logger.error(self.node, "Gripper service not available")
            #     return Status.ExecutionError

            req = SetDigitalIO.Request()
            req.ionum = 1
            req.value = 0 if state == "open" else 1  # 0=Open, 1=close

            future = self.gripper_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, TIMEOUT)

            if future.result() is not None:
                Logger.info(self.node, f"Gripper {state} successfully")
                return Status.EXECUTION_SUCCESS

            Logger.error(self.node, "Failure in gripper service")
            return Status.EXECUTION_ERROR

        except Exception as e:
            Logger.error(self.node, f"Error gripper: {str(e)}")
            return Status.TERMINAL_ERROR

    @mockable(return_value=Status.EXECUTION_SUCCESS, delay=2)
    def move_joint_positions(
        self,
        joint_positions: Union[List[float], dict] = None,
        named_position: str = None,
        velocity: float = 0.1,
        degrees=False,  # set to true if joint_positions are in degrees
    ):
        """Set position of joints.
        If joint_positions is a dict, keys are treated as joint_names
        and values as joint positions.
        Named position has priority over joint_positions.
        """
        if named_position:
            # joint_positions = self.get_named_target(named_position)
            # joint_positions = joint_positions["positions"].keys()

            joint_positions = self.get_named_target(named_position)

            degrees = joint_positions.get("degrees", False)

            joint_positions = joint_positions["joints"]

            self.node.get_logger().info(f"dict: {joint_positions}")

        # Determine format of joint_positions and apply degree conversion if needed.
        if isinstance(joint_positions, dict):
            joint_names = list(joint_positions.keys())
            joint_vals = list(joint_positions.values())
            if degrees:
                joint_vals = [x * DEG_TO_RAD for x in joint_vals]
        elif isinstance(joint_positions, list):
            joint_names = []
            joint_vals = joint_positions.copy()
            if degrees:
                joint_vals = [x * DEG_TO_RAD for x in joint_vals]
        else:
            Logger.error(self.node, "joint_positions must be a list or a dict")
            return Status.EXECUTION_ERROR

        future = self._send_joint_goal(
            joint_names=joint_names, joint_positions=joint_vals, velocity=velocity
        )

        # Wait for goal to be accepted.
        if not self._wait_for_future(future):
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def get_named_target(self, target_name: str):
        """Get named target"""
        return XARM_CONFIGURATIONS[target_name]

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check("_get_joints_client", Status.EXECUTION_ERROR, TIMEOUT)
    def get_joint_positions(
        self,
        degrees=False,  # set to true to return in degrees
    ) -> dict:
        """Get the current joint positions"""
        # self._get_joints_client.wait_for_service(timeout_sec=TIMEOUT)
        future = self._get_joints_client.call_async(GetJoints.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        result = future.result()
        if degrees:
            result.joint_positions = [x * RAD_TO_DEG for x in result.joint_positions]
        print("Joint positions from service: ", result.joint_positions)
        return dict(zip(result.joint_names, result.joint_positions))

    # let the server pick the default values
    @service_check("_move_joints_action_client", -1, TIMEOUT)
    def _send_joint_goal(
        self,
        joint_names=[],
        joint_positions=[],
        velocity=0.0,
        acceleration=0.0,
        planner_id="",
    ):
        print("Joint names: ", joint_names)
        print("Joint positions: ", joint_positions)
        goal_msg = MoveJoints.Goal()
        goal_msg.joint_names = joint_names
        goal_msg.joint_positions = joint_positions
        goal_msg.velocity = velocity
        goal_msg.acceleration = acceleration
        goal_msg.planner_id = planner_id

        self._move_joints_action_client.wait_for_server()

        self.node.get_logger().info("Sending joint goal...")
        return self._move_joints_action_client.send_goal_async(goal_msg)

    def _wait_for_future(self, future) -> bool:
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            True

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        if not result.success:
            return False
        return True

    @mockable(return_value=Status.EXECUTION_SUCCESS, mock=False)
    @service_check(client="follow_face_client", return_value=Status.TERMINAL_ERROR, timeout=TIMEOUT)
    def follow_face(self, follow) -> int:
        """Save the name of the person detected"""

        if follow:
            Logger.info(self.node, "Following face")
        else:
            Logger.info(self.node, "Stopping following face")
        request = FollowFace.Request()
        request.follow_face = follow

        try:
            future = self.follow_face_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if not result.success:
                raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error following face: {e}")
            return Status.EXECUTION_ERROR

        Logger.success(self.node, "Following face request successful")
        return Status.EXECUTION_SUCCESS

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_manipulation_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def pick_object(self, object_name: str, in_configuration: bool = False):
        """Pick an object by name
        object_name: name of the object to pick
        in_configuration: True if the object is in the configuration"""
        # if not self._manipulation_action_client.wait_for_server(timeout_sec=TIMEOUT):
        #     Logger.error(self.node, "Manipulation action server not available")
        #     return Status.EXECUTION_ERROR

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PICK
        goal_msg.pick_params.object_name = object_name
        goal_msg.pick_params.in_configuration = in_configuration

        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)

        if future.result() is None:
            Logger.error(self.node, "Failed to send pick request")
            return Status.EXECUTION_ERROR

        Logger.info(self.node, f"Pick request for {object_name} sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        Logger.info(self.node, f"Pick result: {result}")
        if result.success:
            Logger.success(self.node, f"Pick request for {object_name} successful")
        else:
            Logger.error(self.node, f"Pick request for {object_name} failed")
            return Status.EXECUTION_ERROR

        return Status.EXECUTION_SUCCESS

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_manipulation_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def place(self):
        # if not self._manipulation_action_client.wait_for_server(timeout_sec=TIMEOUT):
        #     Logger.error(self.node, "Manipulation action server not available")
        #     return Status.EXECUTION_ERROR

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send place request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Place request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        Logger.info(self.node, f"Place result: {result}")
        if result.success:
            Logger.success(self.node, "Place request successful")
        else:
            Logger.error(self.node, "Place request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def place_on_shelf(self, plane_height: int, tolerance: int):
        # if not self._manipulation_action_client.wait_for_server(timeout_sec=TIMEOUT):
        #     Logger.error(self.node, "Manipulation action server not available")
        #     return Status.EXECUTION_ERROR

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        goal_msg.place_params.is_shelf = True
        goal_msg.scan_environment = True
        if plane_height is not None and tolerance is not None:
            goal_msg.place_params.table_height = plane_height
            goal_msg.place_params.table_height_tolerance = tolerance
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send place request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Place in shelf request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        Logger.info(self.node, f"Place in shelf result: {result}")
        if result.success:
            Logger.success(self.node, "Place request successful")
        else:
            Logger.error(self.node, "Place request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def pour(self, pour_object_name: str, pour_container_name: str):
        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.POUR
        goal_msg.pour_params.object_name = pour_object_name
        goal_msg.pour_params.bowl_name = pour_container_name
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send pour request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Pour request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        Logger.info(self.node, f"Pour result: {result}")
        if result.success:
            Logger.success(self.node, "Pour request successful")
        else:
            Logger.error(self.node, "Pour request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def place_in_point(self, point: Union[PointStamped, PoseStamped]):
        # Can receive a PointStamped or PoseStamped and will attempt to place the object at that point.
        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        if isinstance(point, PointStamped):
            goal_msg.place_params.forced_pose.header.frame_id = point.header.frame_id
            goal_msg.place_params.forced_pose.pose.position = point.point
        elif isinstance(point, PoseStamped):
            goal_msg.place_params.forced_pose = point
        else:
            Logger.error(self.node, "Invalid point type for place_in_point")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, f"Placing in point/pose: {goal_msg.place_params.forced_pose}")
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send place in point request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Place in point request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        Logger.info(self.node, f"Place in point result: {result}")
        if result.success:
            Logger.success(self.node, "Place in point request successful")
        else:
            Logger.error(self.node, "Place in point request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def pan_to(self, degrees: float):
        joint_positions = self.get_joint_positions(degrees=True)
        joint_positions["joint1"] = joint_positions["joint1"] - degrees
        self.move_joint_positions(joint_positions=joint_positions, velocity=0.75, degrees=True)

    def point(self, degrees: float):
        joint_positions = self.get_joint_positions(degrees=True)
        joint_positions["joint2"] = joint_positions["joint2"] + degrees
        self.move_joint_positions(joint_positions=joint_positions, velocity=0.75, degrees=True)

        joint_positions["joint2"] = joint_positions["joint2"] - degrees
        self.move_joint_positions(joint_positions=joint_positions, velocity=0.75, degrees=True)

    def check_lower(self, degrees: float):
        joint_positions = self.get_joint_positions(degrees=True)
        joint_positions["joint5"] = joint_positions["joint5"] + degrees
        self.move_joint_positions(joint_positions=joint_positions, velocity=0.75, degrees=True)

    def check_upper(self, degrees: float):
        joint_positions = self.get_joint_positions(degrees=True)
        joint_positions["joint5"] = joint_positions["joint5"] - degrees
        self.move_joint_positions(joint_positions=joint_positions, velocity=0.75, degrees=True)

    def move_to_position(self, named_position: str):
        self.move_joint_positions(named_position=named_position, velocity=0.75, degrees=True)

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_fix_position_to_plane_client", return_value=Status.TERMINAL_ERROR, timeout=TIMEOUT
    )
    def get_optimal_position_for_plane(
        self,
        est_heigth: float,
        tolerance: float = 0.2,
        table_or_shelf: bool = True,
        approach_plane=True,
    ):
        """Fix the robot to a plane
        table_or_shelf: True for table, False for shelf
        """
        req = GetOptimalPositionForPlane.Request()
        req.plane_est_min_height = est_heigth - tolerance
        req.plane_est_max_height = est_heigth + tolerance
        req.table_or_shelf = table_or_shelf
        req.approach_plane = approach_plane

        future = self._fix_position_to_plane_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        result = future.result()
        if result is None:
            Logger.error(self.node, "Failed to get optimal position for plane")
            return Status.EXECUTION_ERROR
        result: GetOptimalPositionForPlane.Response
        if result.is_valid:
            Logger.success(self.node, f"Optimal position for plane: {result.pt1}")
            return
        Logger.error(self.node, "Invalid position for plane")
        return Status.EXECUTION_ERROR


if __name__ == "__main__":
    rclpy.init()
    node = Node("manipulation_tasks")
    Manipulation_tasks = ManipulationTasks(node, task="DEMO")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

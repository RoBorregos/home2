#!/usr/bin/env python3

"""
Node to manage the manipulation tasks of the robot.
It provides a high-level interface to control the robot's arm
and gripper, as well as to perform manipulation tasks such as
picking and placing objects, following faces, pouring liquids,
moving to specific positions, and interacting with planes and shelves.
This node abstracts away the low-level details of the robot's
control and provides a simple interface for the task manager to use.
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from task_manager.utils.logger import Logger

# from geometry_msgs.msg import PoseStamped
from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import (
    GetJoints,
    FollowFace,
    GetOptimalPositionForPlane,
    GetOptimalPoseForPlane,
    RemoveCollisionObject,
    FixedDistanceMove,
)
from frida_constants.xarm_configurations import XARM_CONFIGURATIONS
from rclpy.action import ActionClient
from typing import List, Union
from task_manager.utils.decorators import mockable, service_check
from task_manager.utils.status import Status
from frida_interfaces.action import ManipulationAction, GoToHand, MoveToPose
from frida_interfaces.msg import ManipulationTask, Constraint
from geometry_msgs.msg import PointStamped, PoseStamped

# from utils.decorators import service_check
from xarm_msgs.srv import SetDigitalIO

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
    GO_TO_HAND_ACTION_SERVER,
    MOVE_TO_POSE_ACTION_SERVER,
    GRASP_LINK_FRAME,
    FIXED_DISTANCE_MOVE_SERVICE,
)
import time as t

from enum import Enum

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
        self._fixed_distance_move_client = self.node.create_client(
            FixedDistanceMove, FIXED_DISTANCE_MOVE_SERVICE
        )
        self.follow_face_client = self.node.create_client(FollowFace, "/follow_face")
        self.follow_person_client = self.node.create_client(FollowFace, "/follow_person")
        self._remove_collision_object_client = self.node.create_client(
            RemoveCollisionObject, "/manipulation/remove_collision_object"
        )
        self._manipulation_action_client = ActionClient(
            self.node, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self._fix_position_to_plane_client = self.node.create_client(
            GetOptimalPositionForPlane,
            "/manipulation/get_optimal_position_for_plane",
        )
        self._get_optimal_pose_for_plane_client = self.node.create_client(
            GetOptimalPoseForPlane,
            "/manipulation/get_optimal_pose_for_plane",
        )
        self._go_to_hand_action_client = ActionClient(self.node, GoToHand, GO_TO_HAND_ACTION_SERVER)
        self._move_to_pose_action_client = ActionClient(
            self.node, MoveToPose, MOVE_TO_POSE_ACTION_SERVER
        )
        self._flat_grasp_estimator_client = self.node.create_client(
            SetBool, "/flat_grasp_estimator/enable"
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

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
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)

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

        # _send_joint_goal returns -1 (int) if service_check fails
        if future is None or isinstance(future, int):
            Logger.error(self.node, "Failed to send joint goal")
            return Status.EXECUTION_ERROR

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

    @service_check("_fixed_distance_move_client", Status.EXECUTION_ERROR, TIMEOUT)
    def move_arm_vertical(self, distance: float, descend: bool = False):
        """Move the TCP a fixed distance in Z (xArm mode 5 closed-loop).
        descend=False raises the arm (+Z); descend=True lowers it (-Z)."""
        request = FixedDistanceMove.Request()
        request.distance = float(distance)
        request.descend = descend
        future = self._fixed_distance_move_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
        result = future.result()
        if result is None or not result.success:
            Logger.error(self.node, "move_arm_vertical failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

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
            Logger.error(self.node, "Joint goal was rejected")
            return False

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)

        result = result_future.result().result
        if not result.success:
            return False
        return True

    @mockable(return_value=Status.EXECUTION_SUCCESS, mock=False)
    @service_check(client="follow_face_client", return_value=Status.TERMINAL_ERROR, timeout=TIMEOUT)
    def follow_face(self, follow) -> int:
        """Activate or deactivate face following on the arm."""

        if follow:
            Logger.info(self.node, "Following face")
        else:
            Logger.info(self.node, "Stopping following face")
        request = FollowFace.Request()
        request.follow_face = follow

        try:
            future = self.follow_face_client.call_async(request)
            # Mode switching in follow_face_node takes time, use longer timeout
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()

            if result is None:
                raise Exception("Service call timed out")

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
    def pick_object(
        self, object_name: str, in_configuration: bool = False, scan_environment: bool = False
    ):
        """Pick an object by name
        object_name: name of the object to pick
        in_configuration: True if the object is in the configuration
        scan_environment: True to scan environment before picking (useful for shelf picks)"""

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PICK
        goal_msg.pick_params.object_name = object_name
        goal_msg.pick_params.in_configuration = in_configuration
        goal_msg.scan_environment = scan_environment

        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)

        if future.result() is None:
            Logger.error(self.node, "Failed to send pick request")
            return Status.EXECUTION_ERROR

        Logger.info(self.node, f"Pick request for {object_name} sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        if not result_future.done():
            Logger.error(self.node, "Action timed out after 60s")
            return Status.EXECUTION_ERROR
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
    def set_flat_grasp_estimator(self, enable: bool) -> int:
        """Enable or disable the flat grasp estimator node."""
        if not self._flat_grasp_estimator_client.wait_for_service(timeout_sec=5.0):
            Logger.error(self.node, "Flat grasp estimator service not available")
            return Status.EXECUTION_ERROR

        req = SetBool.Request()
        req.data = enable
        future = self._flat_grasp_estimator_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            state = "enabled" if enable else "disabled"
            Logger.info(self.node, f"Flat grasp estimator {state}")
            return Status.EXECUTION_SUCCESS

        Logger.error(self.node, "Failed to set flat grasp estimator state")
        return Status.EXECUTION_ERROR

    def pick_cutlery(self, object_name: str) -> int:
        """Pick a cutlery object (fork, knife, spoon).
        Enables the flat grasp estimator, performs the pick, then disables it."""
        self.set_flat_grasp_estimator(True)
        try:
            result = self.pick_object(object_name)
        finally:
            self.set_flat_grasp_estimator(False)
        return result

    def place(self, close_to: str = "", special_request: str = ""):
        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        if close_to:
            goal_msg.place_params.close_to = close_to
        if special_request:
            goal_msg.place_params.special_request = special_request
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send place request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Place request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        if not result_future.done():
            Logger.error(self.node, "Action timed out after 60s")
            return Status.EXECUTION_ERROR
        result = result_future.result().result
        Logger.info(self.node, f"Place result: {result}")
        if result.success:
            Logger.success(self.node, "Place request successful")
        else:
            Logger.error(self.node, "Place request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    class Direction(Enum):
        LEFT = "left"
        RIGHT = "right"

    def _attempt_place_on_side(self, direction: Direction) -> int:
        """
        Attempts to place an object on the floor at the specified side.
        Returns Status.EXECUTION_SUCCESS if successful, Status.EXECUTION_ERROR otherwise.

        Args:
            direction: Direction.LEFT or Direction.RIGHT

        Returns:
            Status code indicating success or failure
        """
        side_name = direction.value
        named_position = f"place_floor_{side_name}"

        Logger.info(
            self.node, f"{side_name.capitalize()} side clear. Attempting {named_position}..."
        )

        result = self.move_joint_positions(named_position=named_position, velocity=0.3)

        if result == Status.EXECUTION_SUCCESS:
            Logger.success(self.node, f"Object placed on {side_name.upper()}")

            open_gripper_result = self.open_gripper()
            if open_gripper_result == Status.EXECUTION_SUCCESS:
                Logger.success(self.node, "Gripper opened successfully")
            else:
                Logger.error(self.node, "Failed to open gripper")

            self.move_joint_positions(named_position="pick_stare_at_table", velocity=0.3)
            return Status.EXECUTION_SUCCESS

        Logger.warn(self.node, f"Movement to {side_name} failed")
        return Status.EXECUTION_ERROR

    def _check_side_blocked(self, direction: Direction) -> bool:
        """
        Physically moves the arm 5 degrees to check for obstacles.
        Returns True if blocked, False if clear.
        """
        try:
            current_joints = self.get_joint_positions(degrees=True)
            test_joints = current_joints.copy()

            lower_offset = -10.0
            test_joints["joint5"] += lower_offset
            result_tilt = self.move_joint_positions(
                joint_positions=test_joints, velocity=0.2, degrees=True
            )

            if result_tilt != Status.EXECUTION_SUCCESS:
                Logger.warn(self.node, "Path is BLOCKED at tilt")
                return True

            offset = 170.0
            if direction == self.Direction.LEFT:
                test_joints["joint1"] += offset
            elif direction == self.Direction.RIGHT:
                test_joints["joint1"] -= offset

            Logger.info(self.node, f"Scanning {direction.value}...")
            result_pan = self.move_joint_positions(
                joint_positions=test_joints, velocity=0.2, degrees=True
            )

            if result_pan == Status.EXECUTION_SUCCESS:
                Logger.info(self.node, f"Path to {direction.value} is CLEAR")
                return False
            else:
                Logger.warn(self.node, f"Path to {direction.value} is BLOCKED")
                return True

        except Exception as e:
            Logger.warning(self.node, f"Error scanning {direction.value}: {e}")
            return True

    def place_on_floor(self, named_position: str = "pick_stare_at_table") -> int:
        try:
            Logger.info(self.node, f"Moving to {named_position}...")
            result = self.move_joint_positions(named_position=named_position, velocity=0.2)

            if result != Status.EXECUTION_SUCCESS:
                Logger.error(self.node, "Failed to reach start position")
                return Status.EXECUTION_ERROR

            has_collision_left = self._check_side_blocked(self.Direction.LEFT)

            self.move_joint_positions(named_position=named_position, velocity=0.3)

            if not has_collision_left:
                for i in range(3):
                    Logger.info(self.node, f"Attempt {i + 1} to place on left side...")
                    result = self._attempt_place_on_side(self.Direction.LEFT)
                    if result == Status.EXECUTION_SUCCESS:
                        return Status.EXECUTION_SUCCESS
                    if i < 2:
                        Logger.warn(self.node, "Retrying left side...")
                        t.sleep(1)
                Logger.warn(self.node, "Movement to left failed, trying right side...")

            has_collision_right = self._check_side_blocked(self.Direction.RIGHT)

            self.move_joint_positions(named_position=named_position, velocity=0.3)

            if not has_collision_right:
                for i in range(3):
                    Logger.info(self.node, f"Attempt {i + 1} to place on right side...")
                    result = self._attempt_place_on_side(self.Direction.RIGHT)
                    if result == Status.EXECUTION_SUCCESS:
                        return Status.EXECUTION_SUCCESS
                    if i < 2:
                        Logger.warn(self.node, "Retrying right side...")
                        t.sleep(1)
                Logger.error(self.node, "Movement to right also failed")

            Logger.error(self.node, "CRITICAL: Both sides are blocked or planning failed")
            return Status.EXECUTION_ERROR

        except Exception as e:
            Logger.error(self.node, f"Error in place_on_floor: {e}")
            return Status.EXECUTION_ERROR

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
        # wait for result — place on shelf can take 60-120s because of multiple
        # pre-place + full + halfway attempts with OMPL planning. A 60s timeout
        # here was causing the client to give up while the server was still
        # executing the place, leaving the FSM in a bad state (object already
        # dropped in shelf but task_manager thought it failed and retried).
        SHELF_PLACE_TIMEOUT = 180.0
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=SHELF_PLACE_TIMEOUT)
        if not result_future.done():
            Logger.error(self.node, f"Action timed out after {SHELF_PLACE_TIMEOUT}s")
            return Status.EXECUTION_ERROR
        result = result_future.result().result
        Logger.info(self.node, f"Place in shelf result: {result}")
        if result.success:
            Logger.success(self.node, "Place request successful")
        else:
            Logger.error(self.node, "Place request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def pour(
        self, pour_object_name: str, pour_container_name: str, object_already_grasped: bool = False
    ):
        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.POUR
        goal_msg.pour_params.object_name = pour_object_name
        goal_msg.pour_params.bowl_name = pour_container_name
        goal_msg.pour_params.object_already_grasped = object_already_grasped
        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send pour request")
            return Status.EXECUTION_ERROR
        Logger.info(self.node, "Pour request sent")
        # wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        if not result_future.done():
            Logger.error(self.node, "Action timed out after 60s")
            return Status.EXECUTION_ERROR
        result = result_future.result().result
        Logger.info(self.node, f"Pour result: {result}")
        if result.success:
            Logger.success(self.node, "Pour request successful")
        else:
            Logger.error(self.node, "Pour request failed")
            return Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    def clear_collision_objects(self, include_attached: bool = True) -> int:
        """Remove all collision objects from the planning scene.
        Useful before movements that MoveIt would reject due to stale
        collision objects left in the scene by perception."""
        try:
            if not self._remove_collision_object_client.wait_for_service(timeout_sec=2.0):
                Logger.warn(self.node, "remove_collision_object service not available")
                return Status.EXECUTION_ERROR
            request = RemoveCollisionObject.Request()
            request.id = "all"
            request.include_attached = include_attached
            future = self._remove_collision_object_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
            result = future.result()
            if result is None or not result.success:
                Logger.warn(self.node, "Failed to clear collision objects")
                return Status.EXECUTION_ERROR
            Logger.info(self.node, "Planning scene collision objects cleared")
            return Status.EXECUTION_SUCCESS
        except Exception as e:
            Logger.error(self.node, f"Error clearing collision objects: {e}")
            return Status.EXECUTION_ERROR

    def pan_to(self, degrees: float):
        joint_positions = self.get_joint_positions(degrees=True)
        if not isinstance(joint_positions, dict):
            Logger.error(self.node, f"Failed to get joint positions in pan_to: {joint_positions}")
            return Status.EXECUTION_ERROR
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

    def move_to_position(self, named_position: str, velocity: float = 0.75):
        self.move_joint_positions(named_position=named_position, velocity=velocity, degrees=True)

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    def align_arm_toward_centroid(
        self,
        point: PointStamped,
        pre_pose: str = "washing_machine_arrow_pose",
        velocity: float = 0.2,
        frame: str = "base_link",
    ) -> int:
        """Move to `pre_pose` then rotate j1, j2 and j5 so the arm shaft AND
        the gripper approach axis both point at `point`. j3, j4, j6 stay
        locked at the pre-pose values. Returns `EXECUTION_ERROR` if no
        positive-cosine solution exists or if the solved joints would exceed
        the soft joint limits.
        """
        if point is None or point.header.frame_id == "":
            Logger.error(self.node, "align_arm_toward_centroid: invalid point")
            return Status.EXECUTION_ERROR

        if (
            self.move_joint_positions(named_position=pre_pose, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            Logger.error(self.node, f"Failed to move to pre-pose {pre_pose}")
            return Status.EXECUTION_ERROR

        rclpy.spin_once(self.node, timeout_sec=0.5)
        current = self.get_joint_positions()
        if not isinstance(current, dict) or "joint3" not in current:
            Logger.error(self.node, f"align_arm_toward_centroid: bad joints {current}")
            return Status.EXECUTION_ERROR
        j3, j4, j5, j6 = (
            current["joint3"],
            current["joint4"],
            current["joint5"],
            current["joint6"],
        )

        try:
            transform = self._tf_buffer.lookup_transform(
                frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_in = do_transform_point(point, transform)
        except Exception as e:
            Logger.error(self.node, f"TF {point.header.frame_id} -> {frame} failed: {e}")
            return Status.EXECUTION_ERROR

        cx, cy, cz = float(point_in.point.x), float(point_in.point.y), float(point_in.point.z)
        Logger.info(
            self.node,
            f"align_arm_toward_centroid: centroid in {frame} = "
            f"({cx:.3f}, {cy:.3f}, {cz:.3f}); locked "
            f"(j3,j4,j5,j6) = ({j3:.2f}, {j4:.2f}, {j5:.2f}, {j6:.2f}) rad",
        )

        j1, j2, j5_solved, info = _solve_arrow_alignment(
            (cx, cy, cz),
            j3_lock=j3,
            j4_lock=j4,
            j5_seed=j5,
            j6_lock=j6,
        )
        if j1 is None or j2 is None or j5_solved is None:
            Logger.error(
                self.node,
                f"align_arm_toward_centroid: cannot align; "
                f"arm_cos={info.get('arm_cos'):.3f}, "
                f"approach_cos={info.get('approach_cos'):.3f}. Aborting.",
            )
            return Status.EXECUTION_ERROR

        if j1 > math.pi:
            j1 -= 2 * math.pi
        elif j1 < -math.pi:
            j1 += 2 * math.pi

        if not (-math.pi <= j1 <= math.pi):
            Logger.error(self.node, f"j1={j1:.3f} rad out of [-π, π]. Aborting.")
            return Status.EXECUTION_ERROR
        if not (_J2_MIN <= j2 <= _J2_MAX):
            Logger.error(
                self.node,
                f"j2={j2:.3f} rad out of [{_J2_MIN:.3f}, {_J2_MAX:.3f}]. Aborting.",
            )
            return Status.EXECUTION_ERROR
        if not (_J5_MIN <= j5_solved <= _J5_MAX):
            Logger.error(
                self.node,
                f"j5={j5_solved:.3f} rad out of [{_J5_MIN:.3f}, {_J5_MAX:.3f}]. Aborting.",
            )
            return Status.EXECUTION_ERROR

        T = _fk_grasp_frame(j1, j2, j3, j4, j5_solved, j6)
        tip, approach = T[:3, 3], T[:3, 2]
        Logger.info(
            self.node,
            f"Solved arrow alignment: "
            f"j1={math.degrees(j1):+.1f}°, j2={math.degrees(j2):+.1f}°, "
            f"j5={math.degrees(j5_solved):+.1f}°; tip ~"
            f"({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}); "
            f"approach ({approach[0]:+.2f}, {approach[1]:+.2f}, {approach[2]:+.2f}); "
            f"arm_cos={info['arm_cos']:.3f}, approach_cos={info['approach_cos']:.3f}",
        )

        target_joints = {
            "joint1": j1,
            "joint2": j2,
            "joint3": j3,
            "joint4": j4,
            "joint5": j5_solved,
            "joint6": j6,
        }
        return self.move_joint_positions(joint_positions=target_joints, velocity=velocity)

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_move_to_pose_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def move_to_point_offset(
        self,
        point: PointStamped,
        offset_xyz: tuple = (0.0, 0.0, 0.0),
        frame: str = "base_link",
        target_link: str = GRASP_LINK_FRAME,
        velocity: float = 0.2,
        planner_id: str = "RRTConnect",
        tolerance_position: float = 0.02,
        tolerance_orientation: float = 0.1,
    ) -> int:
        """Transform `point` into `frame`, add `offset_xyz`, and command
        `target_link` to that pose via the MoveToPose action.
        """
        if point is None or point.header.frame_id == "":
            Logger.error(self.node, "move_to_point_offset: invalid point")
            return Status.EXECUTION_ERROR

        try:
            transform = self._tf_buffer.lookup_transform(
                frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_in = do_transform_point(point, transform)
        except Exception as e:
            Logger.error(
                self.node,
                f"TF {point.header.frame_id} -> {frame} failed: {e}",
            )
            return Status.EXECUTION_ERROR

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = point_in.point.x + float(offset_xyz[0])
        pose.pose.position.y = point_in.point.y + float(offset_xyz[1])
        pose.pose.position.z = point_in.point.z + float(offset_xyz[2])
        pose.pose.orientation.w = 1.0

        Logger.info(
            self.node,
            f"move_to_point_offset target in {frame}: "
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f}) offset={offset_xyz}",
        )

        goal = MoveToPose.Goal()
        goal.pose = pose
        goal.velocity = float(velocity)
        goal.planner_id = planner_id
        goal.target_link = target_link
        goal.tolerance_position = float(tolerance_position)
        goal.tolerance_orientation = float(tolerance_orientation)

        self._move_to_pose_action_client.wait_for_server()
        send_future = self._move_to_pose_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=TIMEOUT)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            Logger.error(self.node, "MoveToPose goal rejected")
            return Status.EXECUTION_ERROR

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        result = result_future.result().result
        if result.success:
            Logger.success(self.node, "MoveToPose reached target")
            return Status.EXECUTION_SUCCESS

        Logger.error(self.node, "MoveToPose failed to reach target")
        return Status.EXECUTION_ERROR

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_move_to_pose_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def move_to_height_in_front(
        self,
        reference_point: PointStamped,
        forward_distance: float = 0.5,
        lateral: float = 0.0,
        frame: str = "base_link",
        target_link: str = GRASP_LINK_FRAME,
        velocity: float = 0.2,
        planner_id: str = "RRTConnect",
        tolerance_position: float = 0.02,
        tolerance_orientation: float = 0.1,
        orientation_xyzw: tuple = (0.0, 0.7071068, 0.0, 0.7071068),
        per_axis_orientation_tolerance: tuple = None,
    ) -> int:
        """Move `target_link` to a pose in `frame` at fixed (forward_distance, lateral)
        and z equal to `reference_point`'s height after transforming into `frame`.
        Default orientation is a 90° pitch about Y so the gripper approach axis
        points along `frame`'s +X (forward).
        """
        if reference_point is None or reference_point.header.frame_id == "":
            Logger.error(self.node, "move_to_height_in_front: invalid reference_point")
            return Status.EXECUTION_ERROR

        try:
            transform = self._tf_buffer.lookup_transform(
                frame,
                reference_point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            ref_in = do_transform_point(reference_point, transform)
        except Exception as e:
            Logger.error(
                self.node,
                f"TF {reference_point.header.frame_id} -> {frame} failed: {e}",
            )
            return Status.EXECUTION_ERROR

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(forward_distance)
        pose.pose.position.y = float(lateral)
        pose.pose.position.z = ref_in.point.z
        pose.pose.orientation.x = float(orientation_xyzw[0])
        pose.pose.orientation.y = float(orientation_xyzw[1])
        pose.pose.orientation.z = float(orientation_xyzw[2])
        pose.pose.orientation.w = float(orientation_xyzw[3])

        Logger.info(
            self.node,
            f"move_to_height_in_front target in {frame}: "
            f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f}) [ref z from {reference_point.header.frame_id}]",
        )

        goal = MoveToPose.Goal()
        goal.pose = pose
        goal.velocity = float(velocity)
        goal.planner_id = planner_id
        goal.target_link = target_link
        goal.tolerance_position = float(tolerance_position)
        goal.tolerance_orientation = float(tolerance_orientation)

        if per_axis_orientation_tolerance is not None:
            constraint = Constraint()
            constraint.orientation = pose.pose.orientation
            constraint.frame_id = frame
            constraint.target_link = target_link
            constraint.tolerance_orientation = [
                float(per_axis_orientation_tolerance[0]),
                float(per_axis_orientation_tolerance[1]),
                float(per_axis_orientation_tolerance[2]),
            ]
            constraint.weight = 1.0
            constraint.parameterization = 1  # rotation vector (per-axis angle)
            goal.apply_constraint = True
            goal.constraint = constraint
            Logger.info(
                self.node,
                f"Path orientation constraint per-axis tol={constraint.tolerance_orientation} "
                f"in {frame}",
            )

        self._move_to_pose_action_client.wait_for_server()
        send_future = self._move_to_pose_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=TIMEOUT)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            Logger.error(self.node, "MoveToPose goal rejected")
            return Status.EXECUTION_ERROR

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        result = result_future.result().result
        if result.success:
            Logger.success(self.node, "MoveToPose reached height-in-front target")
            return Status.EXECUTION_SUCCESS

        Logger.error(self.node, "MoveToPose failed to reach height-in-front target")
        return Status.EXECUTION_ERROR

    @mockable(return_value=Status.EXECUTION_SUCCESS)
    @service_check(
        client="_go_to_hand_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def go_to_hand(self, point: PointStamped, hand_offset: float = 0.1) -> int:
        """Move the arm to a position suitable for handing over an object.

        Args:
            point: 3D point in space to approach (PointStamped)
            hand_offset: radial distance from the point to position the EEF (meters)

        Returns:
            Status code indicating success or failure
        """
        goal_msg = GoToHand.Goal()
        goal_msg.point = point
        goal_msg.hand_offset = hand_offset

        self._go_to_hand_action_client.wait_for_server()
        future = self._go_to_hand_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)

        if future.result() is None or not future.result().accepted:
            Logger.error(self.node, "GoToHand goal was rejected")
            return Status.EXECUTION_ERROR

        Logger.info(self.node, "GoToHand goal accepted, waiting for result...")
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)

        result = result_future.result().result
        if result.success:
            Logger.success(self.node, "GoToHand completed successfully")
            return Status.EXECUTION_SUCCESS

        Logger.error(self.node, "GoToHand failed to reach target pose")
        return Status.EXECUTION_ERROR

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

    def get_optimal_pose_for_plane(
        self,
        est_heigth: float,
        tolerance: float = 0.2,
        projected_distance: float = 0.5,
    ) -> PoseStamped:
        """send to aproach table"""

        req = GetOptimalPoseForPlane.Request()
        req.plane_est_min_height = est_heigth - tolerance
        req.plane_est_max_height = est_heigth + tolerance
        req.projected_distance = projected_distance

        future = self._get_optimal_pose_for_plane_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        result = future.result()
        if result is None:
            Logger.error(self.node, "Failed to get optimal pose for plane")
            return None
        result: GetOptimalPoseForPlane.Response
        if not result.success:
            Logger.error(self.node, "Invalid pose for plane")
            return None

        return result.optimal_pose

    @mockable(return_value=Status.MOCKED)
    @service_check(
        client="_manipulation_action_client", return_value=Status.EXECUTION_ERROR, timeout=TIMEOUT
    )
    def place_in_point(self, point: PointStamped):
        """Place object at a specific point in space."""
        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PLACE
        goal_msg.place_params.forced_pose = PoseStamped()

        goal_msg.place_params.forced_pose.header.frame_id = point.header.frame_id
        goal_msg.place_params.forced_pose.pose.position.x = point.point.x
        goal_msg.place_params.forced_pose.pose.position.y = point.point.y
        goal_msg.place_params.forced_pose.pose.position.z = point.point.z

        future = self._manipulation_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            Logger.error(self.node, "Failed to send place_in_point request")
            return Status.EXECUTION_ERROR

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        if not result_future.done():
            Logger.error(self.node, "Action timed out after 60s")
            return Status.EXECUTION_ERROR
        result = result_future.result().result
        if result.success:
            Logger.success(self.node, "Place in point successful")
            return Status.EXECUTION_SUCCESS
        Logger.error(self.node, "Place in point failed")
        return Status.EXECUTION_ERROR


# ---------------------------------------------------------------------------
# Analytic xArm6 FK + arrow-shape alignment solver (used by
# `align_arm_toward_centroid`). All constants come from the xArm6 URDF and
# the FRIDA mount; predicting the grasp frame in `base_link` here avoids a
# round-trip through MoveIt for every solver iteration.
# ---------------------------------------------------------------------------

_XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
_XARM_MOUNT_YAW = math.pi / 2
_J1_OFFSET = (0.0, 0.0, 0.267)
_J3_OFFSET = (0.0535, -0.2845, 0.0)
_J4_OFFSET = (0.0775, 0.3425, 0.0)
_J6_OFFSET = (0.076, 0.097, 0.0)
_GRASP_YAW = -math.pi / 4

# Soft limits stay inside xArm6 hardware (j2 ∈ [-118°, +120°],
# j5 ∈ [-97°, +178°]) so the solver never commands into a hard stop.
_J2_MIN = math.radians(-110)
_J2_MAX = math.radians(90)
_J5_MIN = math.radians(-90)
_J5_MAX = math.radians(115)

# joint2 axis origin in base_link (xarm_base translation + joint1 z offset).
_SHOULDER_IN_BASE = np.array(
    [_XARM_MOUNT_XYZ[0], _XARM_MOUNT_XYZ[1], _XARM_MOUNT_XYZ[2] + _J1_OFFSET[2]]
)


def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _rot(axis: str, a: float) -> np.ndarray:
    T = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def _fk_grasp_frame(j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> np.ndarray:
    """4x4 transform of `gripper_grasp_frame` expressed in `base_link`."""
    T = _trans(*_XARM_MOUNT_XYZ) @ _rot("z", _XARM_MOUNT_YAW)
    T = T @ _trans(*_J1_OFFSET) @ _rot("z", j1)
    T = T @ _rot("x", -math.pi / 2) @ _rot("z", j2)
    T = T @ _trans(*_J3_OFFSET) @ _rot("z", j3)
    T = T @ _trans(*_J4_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j4)
    T = T @ _rot("x", math.pi / 2) @ _rot("z", j5)
    T = T @ _trans(*_J6_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j6)
    T = T @ _rot("z", _GRASP_YAW)
    return T


def _best_alignment(direction_fn, target_unit: np.ndarray, lo: float, hi: float, samples: int):
    """Pick the angle in [lo, hi] whose `direction_fn` has max cosine with
    `target_unit`. Returns (best_angle, best_cos, best_dir, extra_metric).
    """
    best_v: Optional[float] = None
    best_cos = -math.inf
    best_dir = None
    extra = 0.0
    for v in np.linspace(lo, hi, samples):
        direction, magnitude = direction_fn(float(v))
        if direction is None:
            continue
        cos = float(np.dot(direction, target_unit))
        if cos > best_cos:
            best_cos = cos
            best_v = float(v)
            best_dir = direction
            extra = magnitude
    return best_v, best_cos, best_dir, extra


def _solve_arrow_alignment(
    centroid_xyz: Tuple[float, float, float],
    j3_lock: float,
    j4_lock: float,
    j5_seed: float,
    j6_lock: float,
    sweep_points: int = 240,
    iterations: int = 3,
) -> Tuple[Optional[float], Optional[float], Optional[float], dict]:
    """Solve (j1, j2, j5) so the arm shaft AND the gripper approach axis both
    point at `centroid_xyz` (in `base_link`). j3, j4, j6 stay locked.

    Iterates j2 ↔ j5 because changing j5 shifts the wrist by ~12 cm, which
    desyncs the arm-shaft direction; re-solving j2 corrects it.
    """
    target_vec = np.array(centroid_xyz, dtype=float) - _SHOULDER_IN_BASE
    target_dist = float(np.linalg.norm(target_vec))
    if target_dist < 1e-6:
        return None, None, None, {"error": "centroid coincides with shoulder"}
    target_unit = target_vec / target_dist

    azimuth = math.atan2(target_unit[1], target_unit[0])
    j1 = -math.pi / 2 + azimuth

    def arm_dir(j2: float, j5: float):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        v = T[:3, 3] - _SHOULDER_IN_BASE
        d = float(np.linalg.norm(v))
        return (v / d, d) if d > 1e-6 else (None, 0.0)

    def approach_dir(j2: float, j5: float):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        n = float(np.linalg.norm(T[:3, 2]))
        return (T[:3, 2] / n, 0.0) if n > 1e-6 else (None, 0.0)

    j5_iter = float(j5_seed)
    j2_iter: Optional[float] = None
    arm_cos = approach_cos = -math.inf
    arm_dir_v = approach_dir_v = None
    arm_reach = 0.0

    for _ in range(max(1, iterations)):
        j2_iter, arm_cos, arm_dir_v, arm_reach = _best_alignment(
            lambda v: arm_dir(v, j5_iter),
            target_unit,
            _J2_MIN,
            _J2_MAX,
            sweep_points,
        )
        if j2_iter is None or arm_cos < 0.0:
            return (
                None,
                None,
                None,
                {
                    "azimuth_deg": math.degrees(azimuth),
                    "target_dist": target_dist,
                    "arm_cos": arm_cos,
                    "error": "no j2 gives forward arm alignment",
                },
            )
        j5_iter, approach_cos, approach_dir_v, _ = _best_alignment(
            lambda v: approach_dir(j2_iter, v),
            target_unit,
            _J5_MIN,
            _J5_MAX,
            sweep_points,
        )
        if j5_iter is None or approach_cos < 0.0:
            return (
                None,
                None,
                None,
                {
                    "azimuth_deg": math.degrees(azimuth),
                    "target_dist": target_dist,
                    "arm_cos": arm_cos,
                    "approach_cos": approach_cos,
                    "error": "no j5 gives forward gripper approach",
                },
            )

    info = {
        "azimuth_deg": math.degrees(azimuth),
        "target_dist": target_dist,
        "arm_cos": arm_cos,
        "arm_reach_at_best": arm_reach,
        "approach_cos": approach_cos,
        "arm_dir": arm_dir_v.tolist() if arm_dir_v is not None else None,
        "approach_dir": approach_dir_v.tolist() if approach_dir_v is not None else None,
    }
    return float(j1), float(j2_iter), float(j5_iter), info


if __name__ == "__main__":
    rclpy.init()
    node = Node("manipulation_tasks")
    Manipulation_tasks = ManipulationTasks(node, task="DEMO")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

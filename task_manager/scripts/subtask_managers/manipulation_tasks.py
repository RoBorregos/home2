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
from frida_interfaces.srv import GetJoints
from frida_constants.xarm_configurations import XARM_CONFIGURATIONS
from rclpy.action import ActionClient
from typing import List, Union
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

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1,
        "TARGET_NOT_FOUND": 2,
    }

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

        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        # simulation = 1
        self.node.declare_parameter("cancel_after_secs", 5.0)

        self._move_joints_action_client = ActionClient(
            self.node, MoveJoints, "/manipulation/move_joints_action_server"
        )

        self._get_joints_client = self.node.create_client(GetJoints, "/manipulation/get_joints")

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
            joint_positions = self.get_named_target(named_position)

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
            return self.STATE["EXECUTION_ERROR"]

        future = self._send_joint_goal(
            joint_names=joint_names, joint_positions=joint_vals, velocity=velocity
        )

        # Wait for goal to be accepted.
        if not self._wait_for_future(future):
            return self.STATE["EXECUTION_ERROR"]
        return self.STATE["EXECUTION_SUCCESS"]

    def get_named_target(self, target_name: str):
        """Get named target"""
        return XARM_CONFIGURATIONS[target_name]

    def get_joint_positions(
        self,
        degrees=False,  # set to true to return in degrees
    ) -> dict:
        """Get the current joint positions"""
        self._get_joints_client.wait_for_service(timeout_sec=TIMEOUT)
        future = self._get_joints_client.call_async(GetJoints.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        result = future.result()
        if degrees:
            result.joint_positions = [x * RAD_TO_DEG for x in result.joint_positions]
        return dict(zip(result.joint_names, result.joint_positions))

    # let the server pick the default values
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


if __name__ == "__main__":
    rclpy.init()
    node = Node("manipulation_tasks")
    Manipulation_tasks = ManipulationTasks(node, task="DEMO")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

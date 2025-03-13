#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from xarm_msgs.srv import SetInt16, SetInt16ById, MoveVelocity
from frida_interfaces.action import MoveJoints
from frida_interfaces.srv import GetJoints
from rclpy.action import ActionClient
from typing import List
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
    SERVICES = {"activate_arm": 0, "deactivate_arm": 1, "move_arm_velocity": 2}
    SUBTASKS = {
        "RECEPTIONIST": [
            SERVICES["activate_arm"],
            SERVICES["deactivate_arm"],
            SERVICES["move_arm_velocity"],
        ],
        "RESTAURANT": [
            # SERVICES["activate_arm"],
            # SERVICES["deactivate_arm"],
            # SERVICES["move_arm_velocity"],
        ],
        "SERVE_BREAKFAST": [
            # SERVICES["activate_arm"],
            # SERVICES["deactivate_arm"],
            # SERVICES["move_arm_velocity"],
        ],
        "STORING_GROCERIES": [
            # SERVICES["activate_arm"],
            # SERVICES["deactivate_arm"],
            # SERVICES["move_arm_velocity"],
        ],
        "STICKLER_RULES": [
            # SERVICES["activate_arm"],
            # SERVICES["deactivate_arm"],
            # SERVICES["move_arm_velocity"],
        ],
        "DEMO": [
            SERVICES["activate_arm"],
            SERVICES["deactivate_arm"],
            SERVICES["move_arm_velocity"],
        ],
    }

    def __init__(self, task_manager, task, mock_data=False) -> None:
        """Initialize the class"""

        self.node = task_manager
        self.mock_data = mock_data
        self.task = task
        simulation = 1
        self.node.declare_parameter("cancel_after_secs", 5.0)

        self.motion_enable_client = self.node.create_client(SetInt16ById, XARM_ENABLE_SERVICE)
        self.mode_client = self.node.create_client(SetInt16, XARM_SETMODE_SERVICE)
        self.state_client = self.node.create_client(SetInt16, XARM_SETSTATE_SERVICE)
        self.move_client = self.node.create_client(MoveVelocity, XARM_MOVEVELOCITY_SERVICE)

        self._move_joints_action_client = ActionClient(
            self.node, MoveJoints, "move_joints_action_server"
        )

        self._get_joints_client = self.node.create_client(GetJoints, "get_joints")

        if not self.mock_data and not simulation:
            self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""
        if self.task not in ManipulationTasks.SUBTASKS:
            Logger.error(self.node, "Task not available")
            return

        if ManipulationTasks.SERVICES["activate_arm"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.motion_enable_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motion enable client not initialized")
            if not self.mode_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motion enable client not initialized")
            if not self.state_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motion enable client not initialized")

        if ManipulationTasks.SERVICES["deactivate_arm"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.motion_enable_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motion enable client not initialized")

        if ManipulationTasks.SERVICES["move_arm_velocity"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.move_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Move client not initialized")

    def activate_arm(self):
        """Activate arm"""

        Logger.info(self.node, "Activating arm")
        # Set motion
        motion_request = SetInt16ById.Request()
        motion_request.id = 8
        motion_request.data = 1
        # Set state
        state_request = SetInt16.Request()
        state_request.data = 0
        # Set mode
        # 0: position control mode
        mode_request = SetInt16.Request()
        mode_request.data = 0

        try:
            future_motion = self.motion_enable_client.call_async(motion_request)
            rclpy.spin_until_future_complete(self.node, future_motion, timeout_sec=TIMEOUT)

            future_mode = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self.node, future_mode, timeout_sec=TIMEOUT)

            future_state = self.state_client.call_async(state_request)
            rclpy.spin_until_future_complete(self.node, future_state, timeout_sec=TIMEOUT)

        except Exception as e:
            Logger.error(self.node, f"Error Activating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm Activated!")
        return self.STATE["EXECUTION_SUCCESS"]

    def deactivate_arm(self):
        """Desactivate arm"""

        Logger.info(self.node, "Desactivating arm")
        # Set motion
        motion_request = SetInt16ById.Request()
        motion_request.id = 8
        motion_request.data = 0

        try:
            future_motion = self.motion_enable_client.call_async(motion_request)
            rclpy.spin_until_future_complete(self.node, future_motion, timeout_sec=TIMEOUT)

        except Exception as e:
            Logger.error(self.node, f"Error desactivating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm Desactivated!")
        return self.STATE["EXECUTION_SUCCESS"]

    def set_move_mode(self):
        Logger.info(self.node, "Setting move  arm")
        mode_request = SetInt16.Request()
        mode_request.data = 4
        try:
            future_mode = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self.node, future_mode, timeout_sec=TIMEOUT)
        except Exception as e:
            Logger.error(self.node, f"Error moving arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm activated for moving!")
        return self.STATE["EXECUTION_SUCCESS"]

    def move_to(self, x: float, y: float):
        Logger.info(self.node, "Moving arm with velocity")

        # Set motion
        x = x * -1
        if x > 0.1:
            x_vel = 0.1
        elif x < -0.1:
            x_vel = -0.1
        else:
            x_vel = x
        if y > 0.1:
            y_vel = 0.1
        elif y < -0.1:
            y_vel = -0.1
        else:
            y_vel = y

        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = [x_vel, 0.0, 0.0, 0.0, y_vel, 0.0, 0.0]

        try:
            print(f"mock moving to {x} {y}")
            future_move = self.move_client.call_async(motion_msg)
            future_move.add_done_callback(self.state_response_callback)  # Fire-and-forget

        except Exception as e:
            Logger.error(self.node, f"Error desactivating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm moved")
        return self.STATE["EXECUTION_SUCCESS"]

    ## CALLBACKS FOR FORGET SERVICE STATE
    def state_response_callback(self, future):
        """Callback for state service response"""
        try:
            result = future.result()
            if result:
                Logger.info(self.node, "Arm moved")
            else:
                Logger.error(self.node, "Failed to move arm")
        except Exception as e:
            self.Logger.error(self.node, f"move service call failed: {str(e)}")

    def move_joint_positions(
        self,
        joint_positions: List[float] = None,
        named_position: str = None,
        velocity: float = 0.1,
        degrees=False,  # set to true if joint_positions are in degrees
    ):
        """Set position of joints"""
        """ named_position has priority over joint_positions"""
        # Send goal
        if named_position:
            joint_positions = self.get_named_target(named_position)
        if degrees:
            joint_positions = [x * DEG_TO_RAD for x in joint_positions]
        future = self._send_joint_goal(joint_positions=joint_positions, velocity=velocity)

        # Wait for goal to be accepted
        if not self._wait_for_future(future):
            return self.STATE["EXECUTION_ERROR"]
        return self.STATE["EXECUTION_SUCCESS"]

    def get_named_target(self, target_name: str):
        """Get named target"""
        if target_name == "home":
            return [-55.0, -3.0, -52.0, 0.0, 53.0, -55.0]
        elif target_name == "zero":
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            return None

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

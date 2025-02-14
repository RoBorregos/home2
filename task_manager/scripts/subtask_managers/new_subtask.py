#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import rclpy
from frida_interfaces.action import Xarm_move
from rclpy.action import ActionClient
from rclpy.node import Node
from utils.logger import Logger
from xarm_msgs.srv import SetInt16, SetInt16ById

XARM_ENABLE_SERVICE = "/xarm/motion_enable"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"


TIMEOUT = 5.0


class ManipulationTasks:
    """Class to manage the vision tasks"""

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1,
        "TARGET_NOT_FOUND": 2,
    }
    SERVICES = {"activate_arm": 0, "desactivate_arm": 1, "move_arm": 2}
    SUBTASKS = {
        "DEMO": [
            SERVICES["activate_arm"],
            SERVICES["desactivate_arm"],
            SERVICES["move_arm"],
        ]
    }

    def __init__(self, task_manager, task, mock_data=False) -> None:
        """Initialize the class"""
        self.node = task_manager
        self.mock_data = mock_data
        self.task = task

        self.motion_enable_client = self.node.create_client(SetInt16ById, XARM_ENABLE_SERVICE)
        self.mode_client = self.node.create_client(SetInt16, XARM_SETMODE_SERVICE)
        self.state_client = self.node.create_client(SetInt16, XARM_SETSTATE_SERVICE)
        self.move_client = ActionClient(self, Xarm_move, "xarm_move_actions")

        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""
        if self.task not in ManipulationTasks.SUBTASKS:
            Logger.error(self.node, "Task not available")
            return

        if ManipulationTasks.SERVICES["activate_arm"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.motion_enable_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motiion enable client not initialized")
            if not self.mode_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motiion enable client not initialized")
            if not self.state_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motiion enable client not initialized")

        if ManipulationTasks.SERVICES["desactivate_arm"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.motion_enable_client.wait_for_service(timeout_sec=TIMEOUT):
                Logger.warn(self.node, "Motiion enable client not initialized")

        if ManipulationTasks.SERVICES["move_arm"] in ManipulationTasks.SUBTASKS[self.task]:
            if not self.move_client.wait_for_server(timeout_sec=TIMEOUT):
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
        mode_request = SetInt16.Request()
        mode_request.data = 4

        try:
            future_motion = self.motion_enable_client.call_async(motion_request)
            rclpy.spin_until_future_complete(self.node, future_motion, timeout_sec=TIMEOUT)

            future_mode = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(
                self.node, future_mode, timeout_sec=TIMEOUT
            )  # Fire-and-forget

            # result = future.result()
            future_state = self.state_client.call_async(state_request)
            rclpy.spin_until_future_complete(self.node, future_state, timeout_sec=TIMEOUT)

            # if not result.success:
            #     raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error Activating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm Activated!")
        return self.STATE["EXECUTION_SUCCESS"]

    def desactivate_arm(self):
        """Desactivate arm"""

        Logger.info(self.node, "Desactivating arm")
        # Set motion
        motion_request = SetInt16ById.Request()
        motion_request.id = 8
        motion_request.data = 0

        try:
            future_motion = self.motion_enable_client.call_async(motion_request)
            rclpy.spin_until_future_complete(self.node, future_motion, timeout_sec=TIMEOUT)

            # if not result.success:
            #     raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error desactivating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm Desactivated!")
        return self.STATE["EXECUTION_SUCCESS"]

    def move_to(self, x: float, y: float):
        """Desactivate arm"""

        Logger.info(self.node, "Moving arm")
        # Set motion
        x = x * -1
        if x > 0.1:
            x_vel = 0.1
        elif x < -0.1:
            x_vel = -0.1
        else:
            x_vel = x

        motion_msg = Xarm_move.Goal()
        motion_msg.speeds = [x_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        try:
            print(f"mock moving to {x} {y}")
            self.move_client.send_goal_async(motion_msg)
            # if not result.success:
            #     raise Exception("Service call failed")

        except Exception as e:
            Logger.error(self.node, f"Error desactivating arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Arm moved")
        return self.STATE["EXECUTION_SUCCESS"]


if __name__ == "__main__":
    rclpy.init()
    node = Node("manipulation_tasks")
    Manipulation_tasks = ManipulationTasks(node, task="DEMO")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

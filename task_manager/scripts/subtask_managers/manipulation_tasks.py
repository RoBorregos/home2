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
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2State
from frida_pymoveit2.robots import xarm6
# import time as t

XARM_ENABLE_SERVICE = "/xarm/motion_enable"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"

TIMEOUT = 5.0


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
        # self.node.declare_parameter("joint_positions", [-55.0, -3.0, -52.0, 0.0, 53.0, -55.0])
        self.node.declare_parameter("cancel_after_secs", 5.0)
        # self.executor_thread = threading.Thread(target=self.spin_executor, daemon=True)
        # self.executor_thread.start()

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=xarm6.joint_names(),
            base_link_name=xarm6.base_link_name(),
            end_effector_name=xarm6.end_effector_name(),
            group_name=xarm6.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit2.planner_id = (
            self.node.get_parameter("planner_id").get_parameter_value().string_value
        )
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()

        self.motion_enable_client = self.node.create_client(SetInt16ById, XARM_ENABLE_SERVICE)
        self.mode_client = self.node.create_client(SetInt16, XARM_SETMODE_SERVICE)
        self.state_client = self.node.create_client(SetInt16, XARM_SETSTATE_SERVICE)
        self.move_client = self.node.create_client(MoveVelocity, XARM_MOVEVELOCITY_SERVICE)

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

    def move_to(self, x: float, y: float):
        Logger.info(self.node, "Moving arm with velocity")

        mode_request = SetInt16.Request()
        mode_request.data = 4

        try:
            future_mode = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self.node, future_mode, timeout_sec=TIMEOUT)

        except Exception as e:
            Logger.error(self.node, f"Error changing mode of arm: {e}")
            return self.STATE["EXECUTION_ERROR"]

        Logger.success(self.node, "Mode changed!")

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

    def getFutureJointsPositions(self):
        """Send position of joints"""
        joints_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return joints_positions

    def move_joints_positions(self, joint_positions):
        """Set position of joints"""
        # rclpy.init()
        # node = Node("setJointsPositions")

        if joint_positions is None:
            Logger.error(self.node, "Joint positions not found")
            return
        if len(joint_positions) != 6:
            Logger.error(self.node, "Joint positions not valid")
            return
        Logger.info(self.node, f"Setting joint positions to: {joint_positions}")
        # Set position of joints

        # self.node.declare_parameter("joints_postions", joint_positions)
        self.node.declare_parameter("synchronous", True)
        if not self.node.has_parameter("planner_id"):
            self.node.declare_parameter("planner_id", "RRTConnectConfigDefault")

        # callback_group = ReentrantCallbackGroup()
        # moveit2 = MoveIt2(
        #     node=node,
        #     joint_names=xarm6.joint_names(),
        #     base_link_name=xarm6.base_link_name(),
        #     end_effector_name=xarm6.end_effector_name(),
        #     group_name=xarm6.MOVE_GROUP_ARM,
        #     callback_group=callback_group,
        # )
        # moveit2.planner_id = (
        #     node.get_parameter("planner_id").get_parameter_value().string_value
        # )

        # executor = rclpy.executors.MultiThreadedExecutor(2)
        # executor.add_node(node)
        # executor_thread = Thread(target=executor.spin, daemon=True, args=())
        # executor_thread.start()
        # node.create_rate(1.0).sleep()

        self.moveit2.max_velocity = 0.15
        self.moveit2.max_acceleration = 0.1

        # joint_positions = (
        #     self.node.get_parameter("joint_positions").get_parameter_value().double_array_value
        # )

        synchronous = self.node.get_parameter("synchronous").get_parameter_value().bool_value
        cancel_after_secs = (
            self.node.get_parameter("cancel_after_secs").get_parameter_value().double_value
        )

        self.node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        self.moveit2.move_to_configuration(joint_positions)

        if synchronous:
            self.moveit2.wait_until_executed()
        else:
            print("Current State:" + str(self.moveit2.query_state()))
            rate = self.create_rate(10)
            while self.moveit2.query_state() != MoveIt2State.EXECUTING:
                rate.sleep()

            print("Current State:" + str(self.moveit2.query_state()))
            future = self.moveit2.get_execution_future()

            if cancel_after_secs > 0.0:
                sleep_time = self.create_rate(cancel_after_secs)
                sleep_time.sleep()
                print("Cancelling goal")
                self.moveit2.cancel_execution()

            while not future.done():
                rate.sleep()

            print("Result status: " + str(future.result().status))
            print("Result error code: " + str(future.result().error_code))

        rclpy.shutdown()
        self.executor_thread.join()
        exit(0)


if __name__ == "__main__":
    rclpy.init()
    node = Node("manipulation_tasks")
    Manipulation_tasks = ManipulationTasks(node, task="DEMO")

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

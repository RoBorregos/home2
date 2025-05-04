import rclpy
from xarm_msgs.srv import SetInt16, MoveVelocity, SetDigitalIO
from frida_constants.manipulation_constants import (
    XARM_MOVEVELOCITY_SERVICE,
    JOINT_VELOCITY_MODE,
    XARM_SET_DIGITAL_TGPIO_SERVICE,
)
from frida_motion_planning.utils.ros_utils import wait_for_future
from typing import List


class XArmServices:
    def __init__(
        self, node, mode_client: rclpy.client.Client, state_client: rclpy.client.Client
    ):
        self.node = node
        self.mode_client = mode_client
        self.state_client = state_client
        if self.mode_client is None or self.state_client is None:
            self.node.get_logger().warn(
                "Cannot initialize XArmServices as no xArm services are available"
            )
            self.move_velocity_client = None
            self.gripper_io_client = None
            return
        self.move_velocity_client = self.node.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE
        )
        self.move_velocity_client.wait_for_service()
        self.gripper_io_client = self.node.create_client(
            SetDigitalIO, XARM_SET_DIGITAL_TGPIO_SERVICE
        )

    def set_joint_velocity(
        self, velocities: List[float], set_mode: bool = True
    ) -> bool:
        if self.move_velocity_client is None:
            self.node.get_logger().warn(
                "Cannot move joint velocity as move velocity service is not available"
            )
            return False
        if set_mode:
            res = self.set_mode(JOINT_VELOCITY_MODE)
            if not res:
                return False
        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = velocities
        future = self.move_velocity_client.call_async(motion_msg)
        future = wait_for_future(future)
        if future:
            self.node.get_logger().info(
                f"Set joint velocity service response: {future.result().get_result()}"
            )
            return True
        return False

    def set_mode(self, mode: int = 0) -> bool:
        self.mode_client.wait_for_service()
        request = SetInt16.Request()
        request.data = mode
        future = self.mode_client.call_async(request)
        while rclpy.ok() and not future.done():
            pass
        if future.result() is not None:
            self.node.get_logger().info(
                f"Set mode service response: {future.result().message}"
            )
        else:
            self.node.get_logger().error("Failed to call set mode service")
            return False

        self.state_client.wait_for_service()
        request = SetInt16.Request()
        request.data = 0
        future = self.state_client.call_async(request)
        while rclpy.ok() and not future.done():
            pass
        if future.result() is not None:
            self.node.get_logger().info(
                f"Set state service response: {future.result().message}"
            )
        else:
            self.node.get_logger().error("Failed to call set state service")
            return False
        return True

    def set_gripper_state(self, state: str) -> bool:
        if self.move_velocity_client is None:
            self.node.get_logger().warn(
                "Cannot set gripper state as move velocity service is not available"
            )
            return False
        request = SetInt16.Request()
        request.data = 1 if state == "open" else 0
        future = self.move_velocity_client.call_async(request)
        future = wait_for_future(future)
        if future:
            self.node.get_logger().info(
                f"Set gripper state service response: {future.result().get_result()}"
            )
            return True
        return False

    def open_gripper(self) -> bool:
        return self.set_gripper_tgpio_digital_state(0)

    def close_gripper(self) -> bool:
        return self.set_gripper_tgpio_digital_state(1)

    def set_gripper_tgpio_digital_state(self, state: int) -> bool:
        if self.gripper_io_client is None:
            self.node.get_logger().warn(
                "Cannot set gripper state as gripper io service is not available"
            )
            return False
        self.node.get_logger().info(
            f"Setting gripper state to {state} using SetDigitalIO service"
        )
        self.gripper_io_client.wait_for_service()
        self.node.get_logger().info("Gripper IO service is available")
        request = SetDigitalIO.Request()
        request.ionum = 1
        request.value = int(state)
        self.gripper_io_client.call_async(request)
        # future = wait_for_future(future)
        return True

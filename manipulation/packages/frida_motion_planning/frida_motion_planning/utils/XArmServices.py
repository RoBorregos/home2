import rclpy
from xarm_msgs.srv import SetInt16, MoveVelocity
from frida_constants.manipulation_constants import (
    XARM_MOVEVELOCITY_SERVICE,
    JOINT_VELOCITY_MODE,
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
            return
        self.move_velocity_client = self.node.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE
        )
        self.move_velocity_client.wait_for_service()

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

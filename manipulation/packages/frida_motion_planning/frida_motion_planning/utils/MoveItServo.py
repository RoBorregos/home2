from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2Servo
from typing import Tuple
from frida_motion_planning.utils.Servo import Servo


class MoveItServo(Servo):
    def __init__(
        self,
        node: Node,
        callback_group: ReentrantCallbackGroup,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
    ):
        self.node = node
        self.servo = MoveIt2Servo(
            node,
            frame_id="world",
            linear_speed=max_velocity,
            angular_speed=max_velocity,
            enable_at_init=False,
            callback_group=callback_group,
        )

    def enable_servo(self) -> None:
        self.servo.enable()

    def disable_servo(self) -> None:
        self.servo.disable()

    def update_servo(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        self.servo.servo(linear=linear, angular=angular, enable_if_disabled=False)

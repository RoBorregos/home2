#!/usr/bin/env python3
"""Checks VAMP's robot model against the running move_group on random states.

Reports the end effector error of VAMP's forward kinematics and how often VAMP and MoveIt disagree on
self-collision. Run it after regenerating the model:
    ros2 run vamp_moveit_plugin check_collision_model.py --ros-args -p samples:=500
"""

import os
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import ContactInformation, RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.node import Node

# VAMP's Python bindings are built inside its submodule and are not installed by colcon
sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../vamp/src")
)
import vamp  # noqa: E402

GROUP = "xarm6"
FRAME = "base_link"


class CollisionModelCheck(Node):
    def __init__(self):
        super().__init__("collision_model_check")
        self.samples = self.declare_parameter("samples", 200).value
        self.robot = getattr(vamp, self.declare_parameter("robot", "frida_real").value)
        self.joint_names = list(self.robot.joint_names())
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def robot_state(self, configuration: np.ndarray) -> RobotState:
        state = RobotState(is_diff=True)
        state.joint_state.name = self.joint_names
        state.joint_state.position = [float(value) for value in configuration]
        return state

    def end_effector_error(self, configuration: np.ndarray) -> float:
        request = GetPositionFK.Request(
            fk_link_names=[self.robot.end_effector()],
            robot_state=self.robot_state(configuration),
        )
        request.header.frame_id = FRAME
        position = self.call(self.fk, request).pose_stamped[0].pose.position
        vamp_position = np.asarray(self.robot.eefk(configuration), dtype=float)[:3, 3]
        return float(
            np.linalg.norm(vamp_position - [position.x, position.y, position.z])
        )

    def moveit_self_collides(self, configuration: np.ndarray) -> bool:
        request = GetStateValidity.Request(
            robot_state=self.robot_state(configuration), group_name=GROUP
        )
        contacts = self.call(self.validity, request).contacts
        return any(
            contact.body_type_1 != ContactInformation.WORLD_OBJECT
            and contact.body_type_2 != ContactInformation.WORLD_OBJECT
            for contact in contacts
        )

    def run(self):
        for client in (self.fk, self.validity):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"{client.srv_name} is not available")
        rng = np.random.default_rng(0)
        errors = []
        false_collisions = missed_collisions = 0
        for _ in range(self.samples):
            # Only contacts are compared, so joint limits do not matter here
            configuration = rng.uniform(-np.pi, np.pi, len(self.joint_names)).astype(
                np.float32
            )
            errors.append(self.end_effector_error(configuration))
            vamp_collides = not self.robot.validate(configuration)
            moveit_collides = self.moveit_self_collides(configuration)
            false_collisions += vamp_collides and not moveit_collides
            missed_collisions += moveit_collides and not vamp_collides

        self.get_logger().info(
            f"End effector error: max {max(errors) * 100:.2f} cm, "
            f"mean {np.mean(errors) * 100:.2f} cm over {self.samples} states"
        )
        self.get_logger().info(
            f"Self-collision: VAMP collides where MoveIt does not in {false_collisions}, "
            f"misses a MoveIt collision in {missed_collisions}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollisionModelCheck()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

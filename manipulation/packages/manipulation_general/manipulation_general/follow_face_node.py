#!/usr/bin/env python3

"""
Follow Face Node - Controls xArm to track a detected face using joint velocity commands.
Provides a /follow_face service to activate/deactivate face tracking,
and switches the arm between velocity mode (4) and MoveIt mode (1) accordingly.
"""

import time

import rclpy
from frida_constants.manipulation_constants import FOLLOW_FACE_ARM_SERVICE
from frida_constants.vision_constants import FOLLOW_TOPIC
from frida_interfaces.srv import FollowFace
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pick_and_place.pipelines.follow import FaceState, face_off, face_on, face_tick
from pick_and_place.robot.follow_arm import FollowArm

RUN_LOOP_PERIOD = 0.1


class FollowFaceNode(Node):
    """Node that tracks a detected face by sending joint velocity commands to the xArm."""

    def __init__(self):
        super().__init__("follow_face_node")
        callback_group = ReentrantCallbackGroup()

        # Face detection subscription
        self.create_subscription(
            Point,
            FOLLOW_TOPIC,
            self._face_detection_callback,
            2,
            callback_group=callback_group,
        )

        # Service clients
        self.arm = FollowArm(
            self, callback_group, self._velocity_done_callback, face_extras=True
        )

        # Wait for critical services
        self.arm.wait_for_services()

        # Disable TGPIO reset on state changes so the gripper stays closed
        # across mode switches. Must be called AFTER the driver is up.
        self.arm.disable_tgpio_reset()

        # Follow face service
        self.service = self.create_service(
            FollowFace,
            FOLLOW_FACE_ARM_SERVICE,
            self._follow_face_service_callback,
            callback_group=callback_group,
        )

        # State
        self.state = FaceState()

        self.create_timer(
            RUN_LOOP_PERIOD, self._run_loop, callback_group=callback_group
        )
        self.get_logger().info("FollowFaceNode has started.")

    # -- Service callback --

    def _follow_face_service_callback(
        self, request: FollowFace.Request, response: FollowFace.Response
    ):
        """Handle follow face service requests."""
        if request.follow_face:
            face_on(self.arm, self.state)
        else:
            face_off(self.arm, self.state)

        response.success = True
        return response

    # -- Face detection --

    def _face_detection_callback(self, msg: Point):
        """Receive face position from vision."""
        self.state.face_x = msg.x
        self.state.face_y = msg.y
        self.state.last_face_detection_time = time.time()
        self.state.has_new_face_data = True

    # -- Movement --

    def _velocity_done_callback(self, future):
        """Callback when velocity command completes."""
        try:
            result = future.result()
            if not result:
                self.get_logger().error("Velocity command returned no result")
        except Exception as e:
            self.get_logger().error(f"Velocity command failed: {e}")
        finally:
            self.arm.busy = False

    # -- Main loop --

    def _run_loop(self):
        """Timer callback: send velocity commands to track the face."""
        face_tick(self.arm, self.state)


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = FollowFaceNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

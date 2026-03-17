#!/usr/bin/env python3

"""Utility node that relays RViz debug points to the OpenContainer action."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from frida_constants.manipulation_constants import OPEN_CONTAINER_ACTION_SERVER
from frida_interfaces.action import OpenContainer


class OpenContainerDebugClient(Node):
    """Listens to RViz point debug topic and triggers the OpenContainer action."""

    def __init__(self) -> None:
        super().__init__("open_container_debug_client")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("queue_size", 10)

        point_topic = "/clicked_point"

        self._goal_in_progress = False

        self._action_client = ActionClient(
            self,
            OpenContainer,
            OPEN_CONTAINER_ACTION_SERVER,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"Waiting for OpenContainer action server at {OPEN_CONTAINER_ACTION_SERVER}"
        )
        self._action_client.wait_for_server()
        self.get_logger().info("OpenContainer action server is ready")

        self._subscription = self.create_subscription(
            PointStamped,
            point_topic,
            self._point_callback,
        )

        self.get_logger().info(f"Listening for debug points on topic '{point_topic}'")

    def _point_callback(self, msg: PointStamped) -> None:
        if self._goal_in_progress:
            self.get_logger().warn("Action goal already in progress, ignoring point")
            return

        if not self._action_client.server_is_ready():
            self.get_logger().warn("OpenContainer action server not ready yet")
            return

        self.get_logger().info(
            "Sending OpenContainer goal for point "
            f"[{msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}]"
        )

        goal = OpenContainer.Goal()
        goal.close_point = msg

        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        self._goal_in_progress = True

    def _goal_response_callback(self, future: rclpy.task.Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_in_progress = False
            self.get_logger().error(f"Failed to send OpenContainer goal: {exc}")
            return

        if not goal_handle.accepted:
            self._goal_in_progress = False
            self.get_logger().warn("OpenContainer goal was rejected")
            return

        self.get_logger().info("OpenContainer goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: rclpy.task.Future) -> None:
        self._goal_in_progress = False
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to get OpenContainer result: {exc}")
            return

        success = bool(result.success)
        if success:
            self.get_logger().info("OpenContainer action finished successfully")
        else:
            self.get_logger().error("OpenContainer action reported failure")

    def _feedback_callback(self, feedback_msg: OpenContainer.FeedbackMessage) -> None:
        state = feedback_msg.feedback.execution_state
        if state:
            self.get_logger().info(f"OpenContainer feedback: {state}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OpenContainerDebugClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

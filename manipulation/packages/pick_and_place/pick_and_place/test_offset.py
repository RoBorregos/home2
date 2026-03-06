#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped
from frida_interfaces.action import GoToHand
from frida_constants.manipulation_constants import GO_TO_HAND_ACTION_SERVER


class TestGoToHandClient(Node):

    def __init__(self):
        super().__init__("test_go_to_hand_client")

        self._client = ActionClient(
            self,
            GoToHand,
            GO_TO_HAND_ACTION_SERVER
        )

        self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

        self.get_logger().info("Waiting for GoToHand action server...")
        self._client.wait_for_server()
        self.get_logger().info("Connected to GoToHand action server.")
        self.get_logger().info("Waiting for clicked points in RViz...")

    # ------------------------------------------------------------------

    def clicked_point_callback(self, msg: PointStamped):

        self.get_logger().info(
            f"Received clicked point: ({msg.point.x:.3f}, "
            f"{msg.point.y:.3f}, {msg.point.z:.3f})"
        )

        goal = GoToHand.Goal()
        goal.point = msg
        goal.hand_offset = 0.1

        send_goal_future = self._client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------------------

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("Goal handle is None (server not responding?)")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted by server")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # ------------------------------------------------------------------

    def get_result_callback(self, future):

        result = future.result()

        if result is None:
            self.get_logger().error("Failed to receive result")
            return

        action_result = result.result
        status = result.status

        self.get_logger().info(f"Action finished with status: {status}")
        self.get_logger().info(f"Result: {action_result}")


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = TestGoToHandClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
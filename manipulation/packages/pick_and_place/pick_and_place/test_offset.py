#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PointStamped
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

        self._client.wait_for_server()
        self.get_logger().info("Connected to GoToHand action server.")
        self.get_logger().info("Waiting for clicked points in RViz...")

    def clicked_point_callback(self, msg: PointStamped):

        self.get_logger().info(f"Received clicked point: {msg.point}")

        request = GoToHand.Goal()
        request.point = msg.point
        future = self._client.send_goal_async(request)
        self.wait_for_future(future)
        action_result = future.result().get_result()

        return future.result(), action_result
    
    def wait_for_future(self, future):
        if future is None:
            self.get_logger().error("Service call failed: future is None")
            return False
        while not future.done():
            pass
        return future

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
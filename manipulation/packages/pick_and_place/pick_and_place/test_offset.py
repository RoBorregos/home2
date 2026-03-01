#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PointStamped
from frida_interfaces.action import OffsetMove
from frida_constants.manipulation_constants import OFFSET_MOVE_ACTION_SERVER


class TestOffsetMoveClient(Node):

    def __init__(self):
        super().__init__("test_offset_move_client")

        self._client = ActionClient(
            self,
            OffsetMove,
            OFFSET_MOVE_ACTION_SERVER
        )

        self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

        self.get_logger().info("Waiting for clicked points in RViz...")

    def clicked_point_callback(self, msg: PointStamped):

        self.get_logger().info(
            f"Clicked point received: "
            f"x={msg.point.x:.3f}, "
            f"y={msg.point.y:.3f}, "
            f"z={msg.point.z:.3f}"
        )

        # PointStamped → PoseStamped
        pose = PoseStamped()
        pose.header = msg.header

        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.send_goal(pose)

    def send_goal(self, pose: PoseStamped):

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal = OffsetMove.Goal()
        goal.pose = pose
        goal.offset = 0.3
        goal.direction = OffsetMove.Goal.HORIZONTAL

        self.get_logger().warning("Sending OffsetMove goal...")

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Feedback received")


def main(args=None):
    rclpy.init(args=args)

    node = TestOffsetMoveClient()

    try:
        rclpy.spin(node)   # 🔥 IMPORTANTE
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
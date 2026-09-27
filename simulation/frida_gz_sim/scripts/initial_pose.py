#!/usr/bin/env python3
"""Publishes the robot's known spawn pose on /initialpose.

On the robot a human drops a 2D Pose Estimate in RViz; nav_central blocks in
_wait_for_initial_pose() until that arrives. The sim knows where it spawned the
robot, so it just says so, repeatedly until nav_central is listening.
"""

import math

import rclpy
from frida_constants.navigation_constants import INITIAL_POSE_TOPIC
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

# Same diagonal nav2's AMCL/rviz default uses: confident but not singular
COVARIANCE = [0.0] * 36
COVARIANCE[0] = COVARIANCE[7] = 0.25
COVARIANCE[35] = 0.068


class InitialPose(Node):
    def __init__(self):
        super().__init__("sim_initial_pose")
        self.x = self.declare_parameter("x", 0.0).value
        self.y = self.declare_parameter("y", 0.0).value
        self.yaw = self.declare_parameter("yaw", 0.0).value
        self.repeat = self.declare_parameter("repeat", 15).value
        self.period = self.declare_parameter("period", 2.0).value
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 10
        )
        self.sent = 0
        self.create_timer(self.period, self._publish)

    def _publish(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance = COVARIANCE
        self.pub.publish(msg)
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info(
                f"Initial pose ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.1f} deg)"
            )
        if self.sent >= self.repeat:
            self.get_logger().info("Initial pose published, exiting")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Publishes the robot's known spawn pose on /initialpose, then stops.

On the robot a human drops a 2D Pose Estimate in RViz; nav_central blocks in
_wait_for_initial_pose() until that arrives. The sim knows where it spawned the
robot, so it says so itself.

It must stop as soon as it has been heard: slam_toolbox re-localizes on every
/initialpose, so a message that lands after the robot has started driving snaps
its estimate back to the spawn pose. That desynchronises the scan from the map,
the planner starts failing and the robot ends up somewhere else entirely.
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
        # Messages sent once the consumer is subscribed, then the node exits
        self.target = self.declare_parameter("target", "nav_central").value
        self.confirm = self.declare_parameter("confirm", 3).value
        self.timeout = self.declare_parameter("timeout", 120.0).value
        self.period = self.declare_parameter("period", 0.5).value
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 10
        )
        self.sent = 0
        self.heard = 0
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + self.timeout
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
        # slam_toolbox also listens here, so wait for the consumer that gates the setup
        if (
            self.target in self.get_node_names()
            and self.pub.get_subscription_count() >= 2
        ):
            self.heard += 1
        if self.heard >= self.confirm:
            self.get_logger().info("Initial pose delivered, exiting")
            raise SystemExit
        if self.get_clock().now().nanoseconds / 1e9 > self.deadline:
            self.get_logger().warn("Nobody subscribed to the initial pose, exiting")
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

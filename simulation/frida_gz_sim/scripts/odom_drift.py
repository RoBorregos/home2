#!/usr/bin/env python3
"""Republishes the true gz odometry as /odom and odom -> base_link with wheel-odometry drift.

The drifted pose integrates the true body-frame motion with a translation scale error and a
yaw error per metre and per radian driven; all zero (the default) passes the truth through.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


class OdomDrift(Node):
    def __init__(self):
        super().__init__("odom_drift")
        # Fraction the reported translation is off, rad of yaw error per m and per rad driven
        self.declare_parameter("scale", 0.0)
        self.declare_parameter("yaw_per_m", 0.0)
        self.declare_parameter("yaw_per_rad", 0.0)
        self.true = None
        self.pose = None
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.create_subscription(Odometry, "/sim/odom_true", self._on_odom, 10)
        self.get_logger().info("Odom drift ready: /sim/odom_true -> /odom")

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        true = (p.x, p.y, _yaw(msg.pose.pose.orientation))
        if self.true is None:
            self.true, self.pose = true, list(true)
        # Body-frame step of the true base since the last message
        dx, dy = true[0] - self.true[0], true[1] - self.true[1]
        c, s = math.cos(self.true[2]), math.sin(self.true[2])
        forward, left = c * dx + s * dy, -s * dx + c * dy
        turn = math.remainder(true[2] - self.true[2], 2 * math.pi)
        self.true = true
        scale = 1.0 + self._param("scale")
        distance = math.hypot(forward, left)
        yaw = self.pose[2]
        self.pose[0] += scale * (math.cos(yaw) * forward - math.sin(yaw) * left)
        self.pose[1] += scale * (math.sin(yaw) * forward + math.cos(yaw) * left)
        self.pose[2] += (
            turn
            + self._param("yaw_per_m") * distance
            + self._param("yaw_per_rad") * abs(turn)
        )
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = self.pose[0], self.pose[1]
        out.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        out.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        out.twist = msg.twist
        self.odom_pub.publish(out)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y = self.pose[:2]
        tf.transform.rotation = out.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[tf]))


def main(args=None):
    rclpy.init(args=args)
    node = OdomDrift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Mobile base velocity commands and odometry pose, for moves the arm makes together with the base."""

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformException

BASE_FRAME = "base_link"
ODOM_FRAME = "odom"
# Nav2 runs with stamped cmd_vel, as table_docker.py publishes it
CMD_VEL_TOPIC = "/cmd_vel"


class Base:
    def __init__(self, node, tf_buffer):
        self._node = node
        self._tf_buffer = tf_buffer
        self._log = node.get_logger()
        self._pub = node.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

    @property
    def logger(self):
        return self._log

    def pose(self) -> Optional[np.ndarray]:
        """(x, y, yaw) of base_link in odom, or None without TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                ODOM_FRAME, BASE_FRAME, rclpy.time.Time()
            ).transform
        except TransformException as error:
            self._log.warn(f"No TF {ODOM_FRAME} -> {BASE_FRAME}: {error}")
            return None
        q = transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        return np.array([transform.translation.x, transform.translation.y, yaw])

    def send(self, vx: float, vy: float, wz: float) -> None:
        """Body-frame velocity (m/s, rad/s)."""
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.twist.linear.x, msg.twist.linear.y = float(vx), float(vy)
        msg.twist.angular.z = float(wz)
        self._pub.publish(msg)

    def stop(self) -> None:
        self.send(0.0, 0.0, 0.0)

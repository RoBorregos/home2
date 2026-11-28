#!/usr/bin/env python3
"""Relays nav2's /cmd_vel onto the plain Twist topic the gz velocity controller takes.

Nav2 runs with enable_stamped_cmd_vel, so /cmd_vel carries TwistStamped while the
gz bridge speaks gz.msgs.Twist. Subscribing here also makes /cmd_vel visible to
nav_central's requirement check before nav2 is up, and the watchdog stops the base
when commands stop arriving - gz holds the last velocity forever. The error params
(settable at runtime) make the base execute commands the way a slipping omnibase does.
"""

import random
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        input_topic = self.declare_parameter("input_topic", "/cmd_vel").value
        output_topic = self.declare_parameter("output_topic", "/sim/cmd_vel").value
        self.stamped = self.declare_parameter("stamped", True).value
        self.timeout = self.declare_parameter("timeout", 0.5).value
        # Scale per axis, constant biases while moving (m/s, rad/s) and white noise (m/s)
        for name in ("scale_x", "scale_y", "scale_yaw"):
            self.declare_parameter(name, 1.0)
        for name in ("lateral_bias", "yaw_bias", "noise"):
            self.declare_parameter(name, 0.0)
        self.pub = self.create_publisher(Twist, output_topic, 10)
        if self.stamped:
            self.create_subscription(
                TwistStamped, input_topic, lambda m: self._relay(m.twist), 10
            )
        else:
            self.create_subscription(Twist, input_topic, self._relay, 10)
        self.last_command = 0.0
        self.stopped = True
        self.create_timer(self.timeout / 2.0, self._watchdog)
        self.get_logger().info(
            f"cmd_vel relay ready: {input_topic} "
            f"({'TwistStamped' if self.stamped else 'Twist'}) -> {output_topic}"
        )

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _relay(self, twist: Twist):
        self.last_command = time.monotonic()
        self.stopped = False
        moving = any((twist.linear.x, twist.linear.y, twist.angular.z))
        out = Twist()
        out.linear.x = twist.linear.x * self._param("scale_x")
        out.linear.y = twist.linear.y * self._param("scale_y")
        out.angular.z = twist.angular.z * self._param("scale_yaw")
        if moving:
            noise = self._param("noise")
            out.linear.x += random.gauss(0.0, noise) if noise > 0.0 else 0.0
            out.linear.y += self._param("lateral_bias") + (
                random.gauss(0.0, noise) if noise > 0.0 else 0.0
            )
            out.angular.z += self._param("yaw_bias")
        self.pub.publish(out)

    def _watchdog(self):
        if self.stopped or time.monotonic() - self.last_command < self.timeout:
            return
        self.stopped = True
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

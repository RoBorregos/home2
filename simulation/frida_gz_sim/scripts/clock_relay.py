#!/usr/bin/env python3
"""Publishes the Gazebo clock on /clock at a fixed rate instead of every physics step.

Bridging /clock at the physics rate (about 1 kHz) makes every sim-time node spend a core
just on clock messages; gz-transport drops the extra messages before they reach Python.
"""

import rclpy
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

# Clock messages per wall second
RATE = 100


class ClockRelay(Node):
    def __init__(self):
        super().__init__("clock_relay")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.gz = GzNode()
        options = SubscribeOptions()
        options.msgs_per_sec = RATE
        self.gz.subscribe(GzClock, "/clock", self._on_clock, options)
        self.get_logger().info(f"Relaying the Gazebo clock at {RATE} Hz")

    def _on_clock(self, msg: GzClock):
        out = Clock()
        out.clock.sec, out.clock.nanosec = msg.sim.sec, msg.sim.nsec
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ClockRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

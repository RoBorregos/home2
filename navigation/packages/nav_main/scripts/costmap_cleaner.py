#!/usr/bin/env python3
"""Flush phantom obstacles from the local costmap while the robot is stationary.

The 3D voxel_layer occasionally marks a stray point (sensor noise / a brief floor
return). nav2 only clears a marked voxel when a later sensor ray passes THROUGH
it — which needs the geometry to change, i.e. the robot to move. So when the base
is stopped, a phantom mark can linger and the robot will not advance until the
mark happens to clear on its own ("it advances until the costmap is cleaned").

This node watches the odometry; once the base has been essentially still for
`stationary_time` seconds it periodically calls the local-costmap "clear entirely"
service. Real obstacles are re-marked by the very next sensor update (the lidar +
ZED run at several Hz), so the only thing that actually disappears is the
transient junk. While the robot is moving, nav2's normal raytrace clearing does
the job and this node stays out of the way.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap


class CostmapCleaner(Node):
    def __init__(self):
        super().__init__('costmap_cleaner')
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        # Below these speeds the base counts as "stopped".
        self.linear_threshold = self.declare_parameter('linear_threshold', 0.02).value   # m/s
        self.angular_threshold = self.declare_parameter('angular_threshold', 0.02).value  # rad/s
        # Must be stopped this long before we start clearing (avoids clearing on
        # brief pauses mid-manoeuvre).
        self.stationary_time = self.declare_parameter('stationary_time', 0.5).value       # s
        # How often to clear while stopped.
        self.clear_period = self.declare_parameter('clear_period', 1.0).value             # s
        self.clear_service = self.declare_parameter(
            'clear_service', '/local_costmap/clear_entirely_local_costmap').value

        self._last_moving = self.get_clock().now()
        self._last_clear = self.get_clock().now()

        self.client = self.create_client(ClearEntireCostmap, self.clear_service)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"costmap_cleaner: clears {self.clear_service} every {self.clear_period}s "
            f"after {self.stationary_time}s stopped (lin<{self.linear_threshold} "
            f"ang<{self.angular_threshold})")

    def _odom_cb(self, msg):
        v = msg.twist.twist
        speed = (v.linear.x ** 2 + v.linear.y ** 2) ** 0.5
        if speed > self.linear_threshold or abs(v.angular.z) > self.angular_threshold:
            self._last_moving = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        stationary_for = (now - self._last_moving).nanoseconds / 1e9
        if stationary_for < self.stationary_time:
            return
        if (now - self._last_clear).nanoseconds / 1e9 < self.clear_period:
            return
        if not self.client.service_is_ready():
            return
        self.client.call_async(ClearEntireCostmap.Request())
        self._last_clear = now


def main(args=None):
    rclpy.init(args=args)
    node = CostmapCleaner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

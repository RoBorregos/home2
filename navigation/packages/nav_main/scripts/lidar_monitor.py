#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


STARTUP_TIMEOUT = 15.0   # seconds to wait for first scan
ONGOING_TIMEOUT = 10.0    # seconds without data before considering lidar dead


class LidarMonitor(Node):

    def __init__(self):
        super().__init__('lidar_monitor')
        self.scan_received = False
        self.last_scan_time = None

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_input',
            self.scan_callback,
            10
        )

        # Startup timeout: if no scan data within STARTUP_TIMEOUT, lidar failed
        self.startup_timer = self.create_timer(STARTUP_TIMEOUT, self.startup_timeout_cb)

        # Ongoing check: periodically verify we're still receiving data
        self.health_timer = self.create_timer(1.0, self.health_check_cb)

        self.get_logger().info(
            f"Lidar monitor started. Waiting up to {STARTUP_TIMEOUT}s for first scan..."
        )

    def scan_callback(self, msg):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info("Lidar is publishing scan data. Health OK.")
            self.startup_timer.cancel()
        self.last_scan_time = self.get_clock().now()

    def startup_timeout_cb(self):
        if not self.scan_received:
            self.get_logger().fatal(
                f"Lidar FAILED: No scan data received within {STARTUP_TIMEOUT}s. "
                "Device may be disconnected or unhealthy. Shutting down."
            )
            raise SystemExit(1)
        self.startup_timer.cancel()

    def health_check_cb(self):
        if not self.scan_received:
            return 

        if self.last_scan_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > ONGOING_TIMEOUT:
            self.get_logger().fatal(
                f"Lidar FAILED: No scan data for {elapsed:.1f}s "
                f"(threshold: {ONGOING_TIMEOUT}s). "
                "Device may have disconnected. Shutting down."
            )
            raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().fatal("Lidar monitor exiting due to health failure.")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

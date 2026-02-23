#!/usr/bin/env python3
"""Simple utility node to ping the customer detection service."""

import sys
import argparse

import rclpy
from rclpy.node import Node

from frida_interfaces.srv import Customer
from frida_constants.vision_constants import GET_CUSTOMER_TOPIC


class CustomerServiceTester(Node):
    """ROS 2 client that calls the customer service once (or repeatedly)."""

    def __init__(self, wait_timeout: float):
        super().__init__("customer_service_tester")
        self._wait_timeout = wait_timeout
        self._client = self.create_client(Customer, GET_CUSTOMER_TOPIC)

    def call_service(self):
        """Send an empty request and return the response (or None on failure)."""
        if not self._client.wait_for_service(timeout_sec=self._wait_timeout):
            self.get_logger().error(
                f"Service {GET_CUSTOMER_TOPIC} not available after {self._wait_timeout}s"
            )
            return None

        self.get_logger().info("Requesting current customer position…")
        future = self._client.call_async(Customer.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._wait_timeout)

        if not future.done():
            self.get_logger().error(
                f"Timed out waiting for response from {GET_CUSTOMER_TOPIC}"
            )
            return None

        response = future.result()
        if response is None:
            self.get_logger().error("Service returned no data")
            return None

        if response.found:
            point = response.point.point
            self.get_logger().info(
                f"Customer found at (x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f})"
            )
        else:
            self.get_logger().warn("Service responded but no customer was detected")
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Test the /vision/get_customer service")
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the service and its response (default: 10)",
    )
    cli_args = parser.parse_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=args)
    node = CustomerServiceTester(wait_timeout=cli_args.wait_timeout)

    try:
        node.call_service()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

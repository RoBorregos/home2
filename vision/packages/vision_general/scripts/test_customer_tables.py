#!/usr/bin/env python3

"""
Test node for the CustomerTables service.
Calls /vision/customer_tables and prints the results.

Usage:
    ros2 run vision_general test_customer_tables
"""

import rclpy
from rclpy.node import Node

from frida_interfaces.srv import CustomerTables

CUSTOMER_TABLES_TOPIC = "/vision/customer_tables"


class TestCustomerTables(Node):
    def __init__(self):
        super().__init__("test_customer_tables")

        self.client = self.create_client(CustomerTables, CUSTOMER_TABLES_TOPIC)

        self.get_logger().info(f"Waiting for service '{CUSTOMER_TABLES_TOPIC}'...")
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service not available, retrying...")

        self.get_logger().info("Service found. Sending request...")
        self.call_service()

    def call_service(self):
        req = CustomerTables.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.done():
            self.get_logger().error("Request timed out after 15s")
            return

        if future.exception() is not None:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return

        result = future.result()

        if not result.success:
            self.get_logger().error("Service returned success=False")
            return

        tables = result.customer_tables
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"CustomerTables response — {len(tables)} table(s) detected\n"
            f"{'='*50}"
        )

        for i, table in enumerate(tables):
            tp = table.table_point.point
            people = table.people.list
            self.get_logger().info(
                f"\n  Table {i + 1}:\n"
                f"    Position (3D): x={tp.x:.3f}  y={tp.y:.3f}  z={tp.z:.3f}\n"
                f"    Customers at this table: {len(people)}"
            )
            for j, person in enumerate(people):
                pp = person.point3d.point
                self.get_logger().info(
                    f"      Person {j + 1}:\n"
                    f"        Pixel centroid: ({person.x}, {person.y})\n"
                    f"        Position (3D):  x={pp.x:.3f}  y={pp.y:.3f}  z={pp.z:.3f}"
                )

        total_customers = sum(len(t.people.list) for t in tables)
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"Total customers assigned to tables: {total_customers}\n"
            f"{'='*50}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestCustomerTables()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

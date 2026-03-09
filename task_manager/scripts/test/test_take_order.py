#!/usr/bin/env python3

"""
Test node for the take_order method of HRITasks.

Initializes the HRI task manager and runs take_order with a configurable
menu so the full pipeline (STT → LLM extraction → embedding matching →
confirmation) can be verified end-to-end on real hardware.
"""

import rclpy
from rclpy.node import Node

from subtask_managers.hri_tasks import HRITasks
from utils.status import Status
from utils.task import Task
from utils.logger import Logger

# ---------------------------------------------------------------------------
# Configuration – edit these values before running the test
# ---------------------------------------------------------------------------

# Menu items to offer the customer
MENU_ITEMS = [
    "water",
    "orange juice",
    "coffee",
    "tea",
    "soda",
    "sandwich",
    "burger",
    "salad",
    "pizza",
    "fries",
]

# How many times take_order is allowed to retry before giving up
RETRIES = 3


class TakeOrderTestNode(Node):
    """Minimal node that exercises HRITasks.take_order()."""

    def __init__(self):
        super().__init__("test_take_order")
        self.get_logger().info("Initializing take_order test node…")

        self.hri = HRITasks(self, task=Task.RESTAURANT)

        # Give ROS time to discover services
        rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("HRI tasks ready. Starting take_order test.")
        self.run()

    def run(self):
        Logger.info(self, f"Menu: {', '.join(MENU_ITEMS)}")
        Logger.info(self, f"Max retries: {RETRIES}")

        status, ordered_items = self.hri.take_order(
            menu_items=MENU_ITEMS,
            retries=RETRIES,
        )

        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, f"take_order SUCCESS – items: {ordered_items}")
        elif status == Status.TIMEOUT:
            Logger.warn(self, "take_order TIMEOUT – customer never confirmed an order.")
        else:
            Logger.error(self, f"take_order returned unexpected status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = TakeOrderTestNode()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

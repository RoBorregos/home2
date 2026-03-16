#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from utils.status import Status
from utils.task import Task
from subtask_managers.nav_tasks import NavigationTasks


class TestNavigationManager(Node):
    def __init__(self):
        super().__init__("NavigationTaskManager")

        self.navigation_manager = NavigationTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Test Navigation Manager started.")
        self.run()

    def run(self):
        result = self.navigation_manager.areas_dump()

        if result != Status.EXECUTION_ERROR:
            self.get_logger().info("SUCCESS")
        else:
            self.get_logger().error("FAILED")


def main(args=None):
    rclpy.init(args=args)
    node = TestNavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

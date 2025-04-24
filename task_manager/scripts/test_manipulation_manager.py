#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy

# from config.hri.debug import config as test_hri_config
from rclpy.node import Node

# from subtask_managers.hri_tasks import HRITasks

from subtask_managers.manipulation_tasks import ManipulationTasks

PICK_OBJECT = "zucaritas"

TEST = "PICK"
# TEST = "PLACE"


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = {}
        # self.subtask_manager["hri"] = HRITasks(self, config=test_hri_config)

        self.subtask_manager["manipulation"] = ManipulationTasks(self, task="DEMO", mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        # """testing vision tasks"""
        if TEST == "PICK":
            self.get_logger().info(f"Trying to pick {PICK_OBJECT}")
            result = self.subtask_manager["manipulation"].pick_object(PICK_OBJECT)
            self.get_logger().info(f"Result: {result}")
        elif TEST == "PLACE":
            self.get_logger().info(f"Trying to place {PICK_OBJECT}")
            result = self.subtask_manager["manipulation"].place()
            self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = TestTaskManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

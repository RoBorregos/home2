#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy
from rclpy.node import Node
from utils.subtask_manager import SubtaskManager, Task
from utils.logger import Logger
import time as t


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = SubtaskManager(
            self,
            task=Task.HELP_ME_CARRY,
            mock_areas=["manipulation","hri","navigation"],
        )
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        while True:
            response = self.subtask_manager.vision.get_track_person()
            Logger.info(self,f"Status tracker = {response}")
            t.sleep(3)
        

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

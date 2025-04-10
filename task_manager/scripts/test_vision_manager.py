#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy
from rclpy.node import Node

from subtask_managers.vision_tasks import VisionTasks
from utils.task import Task
from utils.logger import Logger

task = Task.DEBUG


class TestVision(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.manager = VisionTasks(self, task=task, mock_data=False)
        self.get_logger().info("TestTaskManager has started.")
        self.running_task = True
        self.response = "test"
        self.done = False

    def setResponse(self, status, response):
        self.response = response
        print("RECEIVED RESPONSE")

    def run(self):
        if task == Task.DEBUG:
            if not self.done:
                self.manager.describe_person(self.setResponse)
                self.done = True
            else:
                Logger.info(self, f"Vision task result: {self.response}")

            # status, description = self.manager.describe_bag([0, 0, 1, 1])
            # print(description)

            if self.response != "test":
                self.get_logger().info(f"Vision task result: {self.response}")
                self.done = True
                self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = TestVision()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

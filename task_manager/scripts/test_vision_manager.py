#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy

# from config.hri.debug import config as test_hri_config
from rclpy.node import Node

# from subtask_managers.hri_tasks import HRITasks
from subtask_managers.manipulation_tasks import ManipulationTasks

from subtask_managers.vision_tasks import VisionTasks
from utils.subtask_manager import SubtaskManager
from utils.task import Task
from utils.logger import Logger


class TestVision(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DEBUG, mock_areas=["navigation", "manipulation"])
        self.subtask_manager.vision
        self.get_logger().info("TestTaskManager has started.")
        self.response = "aaa"
        self.done = False
        self.running_task = True

    def setResponse(self, status, response):
        self.response = response
        print("RECEIVED RESPONSE")

    

    def run(self):
        if self.done == False:
            self.subtask_manager.vision.describe_person(self.setResponse)
            # self.subtask_manager.vision.moondream_query_async("Describe image", False, self.setResponse)
            self.done = True
        else:
            Logger.info(self, f"Vision task result: {self.response}")



        if self.response != "aaa":
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


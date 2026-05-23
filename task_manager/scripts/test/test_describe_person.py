#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import Task
from task_manager.subtask_managers.vision_tasks import VisionTasks


class TestPersonDescription(Node):
    def __init__(self):
        super().__init__("describe_person_test")
        self.vision = VisionTasks(self.node, task=Task.HRIC)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.result = {"status": None, "description": ""}

    def on_description(self, status, description):
        self.result["status"] = status
        self.result["description"] = description

    def run_test(self):
        self.vision.describe_person(callback=self.on_description)

        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.result["status"] == Status.EXECUTION_SUCCESS:
            print(self.result["description"])
        elif self.done.is_set():
            print(f"Status: {self.result['status']}, description: {self.result['description']}")
        else:
            print("No result")

def main(args=None):
    rclpy.init(args=args)
    node = TestHRICFaceSave()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = {}
        self.subtask_manager["hri"] = HRITasks(self, config=test_hri_config)

        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        """testing vision tasks"""

        for attr_name in dir(self.subtask_manager["hri"]):
            print(attr_name, ":", getattr(self.subtask_manager["hri"], attr_name))

        user_request = self.subtask_manager["hri"].hear()
        say_res = self.subtask_manager["hri"].say("Hi, my name is frida")
        print("user reuquest:", user_request)
        print("say_res:", say_res)

        drink = self.subtask_manager["hri"].extract_data("Drink", user_request)

        self.get_logger().info(f"Extracted data: {drink}")


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

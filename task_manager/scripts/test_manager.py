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

        user_request = self.subtask_manager["hri"].hear()
        self.subtask_manager["hri"].say("Hi, my name is frida", wait=True)

        drink = self.subtask_manager["hri"].extract_data("Drink", user_request)

        self.get_logger().info(f"Extracted data: {drink}")

        commands = self.subtask_manager["hri"].command_interpreter(user_request)

        self.get_logger().info(f"Interpreted commands: {commands}")

        command_strs = [
            f"I will do action:{command.action}, ({command.complement}), ({command.characteristic})"
            for command in commands
        ]
        command_str = " and ".join(command_strs)

        fixed_text = self.subtask_manager["hri"].refactor_text(command_str)
        self.subtask_manager["hri"].say(fixed_text)


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

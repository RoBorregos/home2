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

        self.subtask_manager["hri"].say(
            "Hi, my name is frida. What is your favorite drink?", wait=True
        )
        self.get_logger().info("Hearing from the user...")

        # This line does run
        user_request = self.subtask_manager["hri"].hear()

        self.get_logger().info(f"Heard: {user_request}")

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

        self.subtask_manager["hri"].say("Can you tell me where to go?")
        location_hint = self.subtask_manager["hri"].hear()

        # Previous line doesn't return
        self.get_logger().info(f"location_hint: {location_hint}")

        closest_found = self.subtask_manager["hri"].find_closest(
            location_hint, "location"
        )

        self.subtask_manager["hri"].say(f"Got it, I will go to {closest_found}!")


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

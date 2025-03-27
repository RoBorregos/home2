#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks


def confirm_preference(interpreted_text, extracted_data):
    return "I heard you like " + extracted_data + ". Is that correct?"


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.subtask_manager = {}
        self.hri_manager = HRITasks(self, config=test_hri_config)

        # wait for a bit
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        # Testing compound commands

        # Test ask

        # s, favorite_drink = self.hri_manager.ask_and_confirm(
        #     "What is your favorite drink?",
        #     "drink",
        #     "The question 'What is your favorite drink?' was asked, full_text corresponds to the response.",
        #     confirm_preference,
        #     False,
        #     3,
        #     5,
        # )

        # self.hri_manager.say(f"I undestood your favorite drink is {favorite_drink}")

        s, interest1 = self.hri_manager.ask_and_confirm(
            "What is your favorite main interest?",
            "interest",
            "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
            confirm_preference,
            False,
            3,
            5,
        )

        s, interest2 = self.hri_manager.ask_and_confirm(
            "What is your favorite second interest?",
            "interest",
            "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
            confirm_preference,
            False,
            3,
            5,
        )

        s, common_interest = self.hri_manager.common_interest(
            "mike", interest1, "rodrigo", interest2
        )

        self.hri_manager.say(common_interest)

        # Testing atomic commands

        # self.subtask_manager["hri"].say(
        #     "Hi, my name is frida. What is your favorite drink?", wait=True
        # )
        # self.get_logger().info("Hearing from the user...")

        # user_request = self.subtask_manager["hri"].hear()

        # self.get_logger().info(f"Heard: {user_request}")

        # drink = self.subtask_manager["hri"].extract_data("Drink", user_request)

        # self.get_logger().info(f"Extracted data: {drink}")

        # commands = self.subtask_manager["hri"].command_interpreter(user_request)

        # self.get_logger().info(f"Interpreted commands: {commands}")

        # command_strs = [
        #     f"I will do action:{command.action}, ({command.complement}), ({command.characteristic})"
        #     for command in commands
        # ]
        # command_str = " and ".join(command_strs)

        # fixed_text = self.subtask_manager["hri"].refactor_text(command_str)
        # self.subtask_manager["hri"].say(fixed_text)

        # self.subtask_manager["hri"].say("I'm frida, Can you tell me where to go?")
        # location_hint = self.subtask_manager["hri"].hear()


def main(args=None):
    rclpy.init(args=args)
    node = TestHriManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

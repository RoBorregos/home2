#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks
from utils.task import Task
import json


def confirm_preference(interpreted_text, extracted_data):
    return "I heard you like " + extracted_data + ". Is that correct?"


TEST_TASK = Task.RECEPTIONIST
TEST_COMPOUND = False
TEST_INDIVIDUAL_FUNCTIONS = True


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.hri_manager = HRITasks(self, config=test_hri_config, task=TEST_TASK)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.test_embeddings()
        self.run()

    def run(self):
        # Testing compound commands

        if TEST_COMPOUND:
            self.compound_functions()

        if TEST_INDIVIDUAL_FUNCTIONS:
            self.individual_functions()

    def individual_functions(self):
        # Test say
        self.hri_manager.say("Hi, my name is frida. What is your favorite drink?", wait=True)
        self.get_logger().info("Hearing from the user...")

        # Test hear
        s, user_request = self.hri_manager.hear(min_audio_length=10.0)
        self.get_logger().info(f"Heard: {user_request}")

        # Test extract_data
        s, drink = self.hri_manager.extract_data("Drink", user_request)
        self.get_logger().info(f"Extracted data: {drink}")

        self.hri_manager.say("Hi, my name is frida. What is your name?", wait=True)
        self.get_logger().info("Hearing from the user...")

        # Test hear
        s, user_request = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        # Test extract_data
        s, drink = self.hri_manager.extract_data("name", user_request)
        self.get_logger().info(f"Extracted data: {drink}")

    def compound_functions(self):
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

    def test_embeddings(self):
        """Testing the embeddings service via HRITasks"""

        hri = self.hri_manager

        # Adding single item
        self.get_logger().info("Adding single item: rotten_potatoes")
        result = hri.add_item(["rotten_potatoes"], json.dumps([{}]))
        self.get_logger().info(f"Result: {result}")

        # Adding multiple items with metadata
        self.get_logger().info("Adding multiple items with metadata")
        documents = ["apple pie with cinnamon", "banana_pie", "mango_pie_with milk"]
        metadata = [{"price": "500"}, {"price": "400"}, {"price": "450"}]
        result = hri.add_item(documents, json.dumps(metadata))
        self.get_logger().info(f"Result: {result}")

        # Querying items
        self.get_logger().info("Querying 'potatoes' from item collection")
        results = hri.query_item("potatoes", top_k=3)
        self.get_logger().info(f"Query results: {results}")

        self.get_logger().info("Querying 'cinnamon' from item collection")
        results = hri.query_item("cinnamon", top_k=3)
        self.get_logger().info(f"Query results: {results}")

        # Adding and querying location
        self.get_logger().info("Adding single location with metadata")
        location_doc = ["kitchen"]
        location_metadata = [{"floor": "1", "type": "room"}]
        result = hri.add_location(location_doc, json.dumps(location_metadata))
        self.get_logger().info(f"Result: {result}")

        self.get_logger().info("Querying 'kitchen' from location collection")
        results = hri.query_location("kitchen", top_k=1)
        self.get_logger().info(f"Query results: {results}")

        # ---- save_command_history ----
        self.get_logger().info("Saving command history for add_item command")
        hri.save_command_history(
            command="add_item",
            complement=documents,
            characteristic="items",
            result="Success",
            status=1,
        )

        self.get_logger().info("Querying command_history collection for the saved command")
        history = hri._query_("add_item", "command_history", top_k=1)
        self.get_logger().info(f"Command history query results: {history}")
        assert any("add_item" in entry for entry in history), "Command history entry not found"


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

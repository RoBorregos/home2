#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import json

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks
from utils.task import Task


def confirm_preference(interpreted_text, extracted_data):
    return "I heard you like " + extracted_data + ". Is that correct?"


TEST_TASK = Task.RECEPTIONIST
TEST_COMPOUND = False
TEST_INDIVIDUAL_FUNCTIONS = True
TEST_EMBEDDINGS = False


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.hri_manager = HRITasks(self, config=test_hri_config, task=TEST_TASK)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        # Testing compound commands

        if TEST_COMPOUND:
            self.compound_functions()

        if TEST_INDIVIDUAL_FUNCTIONS:
            self.individual_functions()

        if TEST_EMBEDDINGS:
            self.test_embeddings()

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

        # Test categorize objects
        s, categorized_shelves, objects_to_add = self.hri_manager.categorize_objects(
            ["butter", "pear", "oats", "turkey", "pineapple"],
            {
                "1": ["milk", "cheese", "yogurt"],
                "2": ["apple", "banana", "grapes"],
                "3": [],
                "4": ["chicken", "beef"],
            },
        )

        self.get_logger().info(f"categorized_shelves: {str(categorized_shelves)}")
        self.get_logger().info(f"objects_to_add: {str(objects_to_add)}")

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
        results = hri.query_item("potatoes", top_k=1)
        self.get_logger().info(f"Query results: {results}")

        self.get_logger().info("Querying 'cinnamon' from item collection")
        results = hri.query_item("cinnamon", top_k=3)
        self.get_logger().info(f"Query results: {results}")

        # Adding and querying location
        self.get_logger().info("Querying 'kitchen' from location collection")

        results_location = hri.query_location("kitchen table", top_k=1)
        subarea = hri.get_subarea(results_location)
        area = hri.get_area(results_location)
        self.get_logger().info(f"Subarea: {subarea}")
        self.get_logger().info(f"Area: {area}")
        self.get_logger().info(f"Query results: {results_location}")

        # ---- save_command_history ----
        self.get_logger().info("Saving command history for add_item command")

        hri.add_command_history(
            command="add_item",
            complement="complement for testing 6",
            characteristic="items 6",
            result="Success",
            status=1,
        )

        self.get_logger().info("Querying command_history collection for the saved command")
        history = hri.query_command_history("add_item")
        context = hri.get_context(history)
        complement = hri.get_complement(history)
        characteristic = hri.get_characteristic(history)
        result = hri.get_result(history)
        status = hri.get_status(history)

        self.get_logger().info(f"context history query results: {context}")
        self.get_logger().info(f"complement history query results: {complement}")
        self.get_logger().info(f"characteristic history query results: {characteristic}")
        self.get_logger().info(f"result history query results: {result}")
        self.get_logger().info(f"status history query results: {status}")
        # ---- end save_command_history ----

        self.get_logger().info("TESTING THE FIND CLOSEST FUNCTION")
        # Test find_closest
        result_closest = hri.find_closest(documents, "milk")
        self.get_logger().info(f"Closest result: {result_closest}")


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

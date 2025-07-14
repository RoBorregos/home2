#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import time
from typing import Union

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from subtask_managers.hri_tasks import HRITasks

# from subtask_managers.subtask_meta import SubtaskMeta
from utils.baml_client.types import (
    AnswerQuestion,
    CommandListLLM,
    Count,
    FindPerson,
    FindPersonByName,
    FollowPersonUntil,
    GetPersonInfo,
    GetVisualInfo,
    GiveObject,
    GoTo,
    GuidePersonTo,
    PickObject,
    PlaceObject,
    SayWithContext,
)
from utils.status import Status
from utils.task import Task

InterpreterAvailableCommands = Union[
    CommandListLLM,
    GoTo,
    PickObject,
    FindPersonByName,
    FindPerson,
    Count,
    GetPersonInfo,
    GetVisualInfo,
    AnswerQuestion,
    FollowPersonUntil,
    GuidePersonTo,
    GiveObject,
    PlaceObject,
    SayWithContext,
]


def confirm_preference(interpreted_text, extracted_data):
    return "I heard you like " + extracted_data + ". Is that correct?"


TEST_TASK = Task.DEBUG
TEST_COMPOUND = False
TEST_INDIVIDUAL_FUNCTIONS = False
TEST_EMBEDDINGS = True
TEST_ASYNC_LLM = False
TEST_STREAMING = False
TEST_MAP = False
TEST_OBJECT_LOCATION = False


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

        if TEST_ASYNC_LLM:
            self.async_llm_test()

        if TEST_STREAMING:
            self.test_streaming()

        if TEST_MAP:
            self.test_map()

        if TEST_OBJECT_LOCATION:
            self.test_object_location()

    def individual_functions(self):
        # Test say
        self.hri_manager.say("Hi, my name is frida. What is your favorite drink?", wait=True)
        self.get_logger().info("Hearing from the user...")

        # Test hear
        s, user_request = self.hri_manager.hear()
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
        s, categorized_shelves = self.hri_manager.get_shelves_categories(
            {
                "1": ["milk", "cheese", "yogurt"],
                "2": ["apple", "banana", "grapes"],
                "3": [],
                "4": ["chicken", "beef"],
            },
        )

        self.get_logger().info(f"categorized_shelves: {str(categorized_shelves)}")

    def test_streaming(self):
        accepted_future = self.hri_manager.hear_streaming(
            silence_time=20,
            timeout=20,
        )
        start_time = time.time()
        offset_time = 10
        cancelled = False

        self.get_logger().info("Waiting for goal to be accepted...")
        rclpy.spin_until_future_complete(self, accepted_future, timeout_sec=4 + 1)

        # Get the goal handle
        goal_handle = accepted_future.result()
        self.get_logger().info(f"Goal handle type: {type(goal_handle)}")
        self.get_logger().info(f"Goal handle: {goal_handle}")

        if hasattr(goal_handle, "accepted"):
            self.get_logger().info(f"Goal handle accepted: {goal_handle.accepted}")
        else:
            self.get_logger().error("Goal handle doesn't have 'accepted' attribute!")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal was not accepted!")
            return

        goal_future = goal_handle.get_result_async()

        while not goal_future.done():
            self.get_logger().info("Waiting for the streaming future to complete...")
            self.get_logger().info(f"Future feedback: {self.hri_manager.current_transcription}")

            if time.time() - start_time > offset_time:
                self.get_logger().info("GOAL CANCELED")
                if not cancelled:
                    # Cancel the goal directly using the goal handle
                    self.get_logger().info("Attempting to cancel goal...")
                    try:
                        self.hri_manager.cancel_hear_action()
                        # cancel_future = goal_handle.cancel_goal_async()
                        # self.get_logger().info(f"Cancel future type: {type(cancel_future)}")
                        # rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                        # cancel_result = cancel_future.result()
                        # self.get_logger().info(f"Cancel result: {cancel_result}")
                        # self.get_logger().info(f"Cancel result type: {type(cancel_result)}")
                    except Exception as e:
                        self.get_logger().error(f"Error cancelling goal: {e}")
                    cancelled = True
            rclpy.spin_once(self, timeout_sec=0.1)

        # s, user_request = self.hri_manager.hear()
        # self.get_logger().info(f"Heard: {user_request}")

        # s, keyword = self.hri_manager.interpret_keyword(["yes", "no", "maybe"], timeout=5.0)
        # self.get_logger().info(f"Interpreted keyword: {keyword}")

    def compound_functions(self):
        s, name = self.hri_manager.ask_and_confirm(
            "What is your name?",
            "LLM_name",
            # "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
            # confirm_preference,
            use_hotwords=False,
            # 3,
            # 5,
        )

        self.hri_manager.say(f"Hi {name}, nice to meet you!", wait=True)

        # s, interest1 = self.hri_manager.ask_and_confirm(
        #     "What is your favorite main interest?",
        #     "LLM_interest",
        #     "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
        #     confirm_preference,
        #     False,
        #     3,
        #     5,
        # )

        # s, interest2 = self.hri_manager.ask_and_confirm(
        #     "What is your favorite second interest?",
        #     "LLM_interest",
        #     "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
        #     confirm_preference,
        #     False,
        #     3,
        #     5,
        # )

        # s, common_interest = self.hri_manager.common_interest(
        #     "mike", interest1, "rodrigo", interest2
        # )

        # self.hri_manager.say(common_interest)

    def test_embeddings(self):
        """Testing the embeddings service via HRITasks using only specified objects from the given list"""

        test_cases = [
            {
                "name": "Beverages, drinks and snacks",
                "table_objects": ["apple", "fanta_soda", "orange", "cookies", "chips"],
                "shelves": {0: ["tea_lipton", "coke_soda"], 1: ["banana"], 2: ["cereal"]},
                "answer": {
                    0: {"category": "beverages", "objects_to_add": ["fanta_soda"]},
                    1: {"category": "fruits", "objects_to_add": ["apple", "orange"]},
                    2: {"category": "snacks", "objects_to_add": ["cookies", "chips"]},
                },
            },
            {
                "name": "Sweets, utencils and sports with empty",
                "table_objects": ["apple", "cookies", "fork", "spoon", "tenis_ball"],
                "shelves": {0: ["tea_lipton"], 1: ["bowl"], 2: []},
                "answer": {
                    0: {"category": "sweets", "objects_to_add": ["apple", "cookies"]},
                    1: {"category": "utencils", "objects_to_add": ["fork", "spoon"]},
                    2: {"category": "sports", "objects_to_add": ["tenis_ball"]},
                },
            },
            {
                "name": "Beverages and utencils",
                "table_objects": ["coke_soda", "fanta_soda", "fork"],
                "shelves": {
                    0: ["tea_lipton"],
                    1: ["spoon"],
                },
                "answer": {
                    0: {"category": "beverages", "objects_to_add": ["coke_soda", "fanta_soda"]},
                    1: {"category": "utencils", "objects_to_add": ["fork"]},
                },
            },
        ]

        for i, test_case in enumerate(test_cases, 1):
            self.get_logger().info(f"\n=== Test Case {i}: {test_case['name']} ===")
            self.get_logger().info(f"Table objects: {test_case['table_objects']}")
            self.get_logger().info(f"Shelves: {test_case['shelves']}")

            try:
                s, categorized_shelves, objects_to_add = self.hri_manager.categorize_objects(
                    test_case["table_objects"], test_case["shelves"]
                )

                if s == Status.EXECUTION_SUCCESS:
                    expected_added_objects = test_case["answer"]

                    test_passed = True

                    for shelve in expected_added_objects:
                        if len(objects_to_add[shelve]) != len(
                            expected_added_objects[shelve]["objects_to_add"]
                        ):
                            test_passed = False
                            break
                        for placed_object in objects_to_add[shelve]:
                            if (
                                placed_object
                                not in expected_added_objects[shelve]["objects_to_add"]
                            ):
                                test_passed = False
                                break

                    if test_passed:
                        self.get_logger().info("Test passed!")
                    else:
                        self.get_logger().error("Test failed.")
                        self.get_logger().error("Expected answer: " + str(expected_added_objects))
                        self.get_logger().error(
                            "Function response: "
                            + "objects_to_add: "
                            + str(objects_to_add)
                            + ", categories: "
                            + str(categorized_shelves)
                        )

                else:
                    self.get_logger().error(f"✗ FAILED - Status: {s}")

            except Exception as e:
                self.get_logger().error(f"✗ EXCEPTION in test case {i}: {str(e)}")

            self.get_logger().info("-" * 50)

    def async_llm_test(self):
        test = self.hri_manager.extract_data("LLM_name", "My name is John Doe", is_async=True)

        self.get_logger().info(f"Extract data future: {test}")
        self.get_logger().info(f"Extract data future status: {test.done(), test.result()}")

        self.get_logger().info("Waiting for the future to complete...")

        rclpy.spin_until_future_complete(self, test)

        self.get_logger().info(f"Extracted data: {test.result()}")

        # Test original functionality
        test = self.hri_manager.extract_data("LLM_name", "My name is John Doe")
        self.get_logger().info(f"Extract data result: {test}")

        s, res = self.hri_manager.common_interest("John", "Football", "Gilbert", "Basketball")

        self.get_logger().info(f"Common interest result: {res}")

        # Test async LLM with a timeout
        f = self.hri_manager.common_interest(
            "John", "Football", "Gilbert", "Basketball", is_async=True
        )
        rclpy.spin_until_future_complete(self, f)

        self.get_logger().info(f"Common interest future: {f}")
        self.get_logger().info(f"Common interest future status: {f.done()}, {f.result()}")

    def test_map(self):
        """
        Test the map functionality of the HRITasks.
        """
        self.get_logger().info("Testing map functionality...")

        # Show the map with a specific item
        self.hri_manager.show_map(name="Phone")
        time.sleep(5)

        # Clear the map
        self.hri_manager.show_map(clear_map=True)
        time.sleep(2)

        # Show the map again to verify it is cleared
        self.hri_manager.show_map(name="Mug")
        time.sleep(5)

    def test_object_location(self):
        object_name = "cheese"
        self.get_logger().info("Testing object location retrieval without context...")
        res = self.hri_manager.query_location(object_name, top_k=3)
        for i, location in enumerate(res):
            self.get_logger().info(f"{i + 1}: {location}")

        self.get_logger().info("Testing object location retrieval with context...")
        res_with_context = self.hri_manager.query_location(object_name, top_k=3, use_context=True)
        for i, location in enumerate(res_with_context):
            self.get_logger().info(f"{i + 1}: {location}")


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

#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import os
import json
import time
import csv
from datetime import datetime
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


DATA_DIR = "/workspace/src/hri/packages/nlp/test/"
OUTPUT_DIR = os.path.join(DATA_DIR, "output")
TEST_COMPOUND = False
TEST_INDIVIDUAL_FUNCTIONS = False
TEST_CATEGORIZE_SHELVES = False
TEST_ASYNC_LLM = False
TEST_STREAMING = False
TEST_MAP = False
TEST_OBJECT_LOCATION = False
TEST_IS_POSITIVE = False
TEST_IS_NEGATIVE = False
TEST_DATA_EXTRACTOR = False


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.hri_manager = HRITasks(self, config=test_hri_config, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        if TEST_COMPOUND:
            self.compound_functions()

        if TEST_INDIVIDUAL_FUNCTIONS:
            self.individual_functions()

        if TEST_CATEGORIZE_SHELVES:
            self.test_categorize_shelves()

        if TEST_ASYNC_LLM:
            self.async_llm_test()

        if TEST_STREAMING:
            self.test_streaming()

        if TEST_MAP:
            self.test_map()

        if TEST_OBJECT_LOCATION:
            self.test_object_location()

        if TEST_IS_POSITIVE:
            self.test_is_positive()

        if TEST_IS_NEGATIVE:
            self.test_is_negative()

        if TEST_DATA_EXTRACTOR:
            self.test_data_extractor()

        exit(0)

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
        s, user_request = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        s, keyword = self.hri_manager.interpret_keyword(["yes", "no", "maybe"], timeout=5.0)
        self.get_logger().info(f"Interpreted keyword: {keyword}")

    def compound_functions(self):
        s, loc, orientation = self.hri_manager.get_location_orientation("kitchen")

        self.get_logger().info(f"Final result: {loc}, {orientation}")

        # s, name = self.hri_manager.ask_and_confirm(
        #     "What is your name?",
        #     "LLM_name",
        #     # "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
        #     # confirm_preference,
        #     use_hotwords=False,
        #     # 3,
        #     # 5,
        # )

        # self.hri_manager.say(f"Hi {name}, nice to meet you!", wait=True)

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

    def test_categorize_shelves(self):
        test_cases_file = os.path.join(DATA_DIR, "categorize_objects.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"categorize_objects_{date_str}.csv")

        results = []
        passed_tests = 0

        for i, test_case in enumerate(test_cases, 1):
            self.get_logger().info(f"Test case {i}: {test_case['name']}")

            # Convert string keys to integers for shelves
            shelves = {int(k): v for k, v in test_case["shelves"].items()}
            expected = {int(k): v for k, v in test_case["answer"].items()}

            actual_output = None
            success = False

            try:
                s, _, objects_to_add, categorized_shelves = self.hri_manager.categorize_objects(
                    test_case["table_objects"], shelves
                )

                if s == Status.EXECUTION_SUCCESS:
                    actual_output = {
                        "objects_to_add": objects_to_add,
                        "categories": categorized_shelves,
                    }

                    success = True
                    for shelve in expected:
                        if len(objects_to_add[shelve]) != len(expected[shelve]["objects_to_add"]):
                            success = False
                            break

                        # Handle category as a list - take first element
                        actual_category = categorized_shelves[shelve]
                        if isinstance(actual_category, list):
                            actual_category = actual_category[0] if actual_category else None

                        if expected[shelve]["category"] != actual_category:
                            success = False
                            break

                        for placed_object in objects_to_add[shelve]:
                            if placed_object not in expected[shelve]["objects_to_add"]:
                                success = False
                                break

                    if success:
                        passed_tests += 1
                        self.get_logger().info("Test passed!")
                    else:
                        self.get_logger().error("Test failed.")

                else:
                    self.get_logger().error(f"FAILED: {s}")
                    actual_output = f"ERROR: {s}"

            except Exception as e:
                self.get_logger().error(f"EXCEPTION: {str(e)}")
                actual_output = f"EXCEPTION: {str(e)}"

            results.append([i, test_case["name"], str(expected), str(actual_output), success])
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["test_number", "name", "expected_output", "actual_output", "success"])
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")

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

    def test_is_positive(self):
        test_cases_file = os.path.join(DATA_DIR, "is_positive.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"is_positive_{date_str}.csv")

        results = []
        passed_tests = 0

        for i, (input_text, expected_output) in enumerate(test_cases, 1):
            self.get_logger().info(f"Test case {i}")

            actual_output = None
            success = False

            try:
                s, is_positive = self.hri_manager.is_positive(input_text)

                if s == Status.EXECUTION_SUCCESS:
                    actual_output = is_positive
                    success = is_positive == expected_output
                    if success:
                        passed_tests += 1
                        self.get_logger().info("Test passed!")
                    else:
                        self.get_logger().error("Test failed.")

                else:
                    self.get_logger().error(f"FAILED: {s}")
                    actual_output = f"ERROR: {s}"

            except Exception as e:
                self.get_logger().error(f"EXCEPTION: {str(e)}")
                actual_output = f"EXCEPTION: {str(e)}"

            results.append([i, input_text, expected_output, actual_output, success])
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["test_number", "input", "expected_output", "actual_output", "success"])
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")

    def test_is_negative(self):
        test_cases_file = os.path.join(DATA_DIR, "is_negative.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"is_negative_{date_str}.csv")

        results = []
        passed_tests = 0

        for i, (input_text, expected_output) in enumerate(test_cases, 1):
            self.get_logger().info(f"Test case {i}")

            actual_output = None
            success = False

            try:
                s, is_negative = self.hri_manager.is_negative(input_text)

                if s == Status.EXECUTION_SUCCESS:
                    actual_output = is_negative
                    success = is_negative == expected_output
                    if success:
                        passed_tests += 1
                        self.get_logger().info("Test passed!")
                    else:
                        self.get_logger().error("Test failed.")

                else:
                    self.get_logger().error(f"FAILED: {s}")
                    actual_output = f"ERROR: {s}"

            except Exception as e:
                self.get_logger().error(f"EXCEPTION: {str(e)}")
                actual_output = f"EXCEPTION: {str(e)}"

            results.append([i, input_text, expected_output, actual_output, success])
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["test_number", "input", "expected_output", "actual_output", "success"])
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")

    def test_data_extractor(self):
        test_cases_file = os.path.join(DATA_DIR, "data_extractor.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"data_extractor_{date_str}.csv")

        results = []
        passed_tests = 0

        for i, (input_text, query, context, expected_output) in enumerate(test_cases, 1):
            self.get_logger().info(f"Test case {i}")

            actual_output = None
            success = False

            try:
                s, extracted_data = self.hri_manager.extract_data(query, input_text, context)

                if s == Status.EXECUTION_SUCCESS:
                    actual_output = extracted_data
                    success = extracted_data == expected_output
                    if success:
                        passed_tests += 1
                        self.get_logger().info("Test passed!")
                    else:
                        self.get_logger().error("Test failed.")

                else:
                    self.get_logger().error(f"FAILED: {s}")
                    actual_output = f"ERROR: {s}"

            except Exception as e:
                self.get_logger().error(f"EXCEPTION: {str(e)}")
                actual_output = f"EXCEPTION: {str(e)}"

            results.append([i, input_text, query, context, expected_output, actual_output, success])
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "test_number",
                    "input",
                    "query",
                    "context",
                    "expected_output",
                    "actual_output",
                    "success",
                ]
            )
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")


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

#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import time
from typing import Union

import rclpy
from config.hri.debug import config as test_hri_config
from rclpy.duration import Duration
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
TEST_EMBEDDINGS = False
TEST_ASYNC_LLM = False
TEST_STREAMING = False
TEST_MAP = False
TEST_OBJECT_LOCATION = True


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
        s, user_request = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        s, keyword = self.hri_manager.interpret_keyword(["yes", "no", "maybe"], timeout=5.0)
        self.get_logger().info(f"Interpreted keyword: {keyword}")

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
        """Testing the embeddings service via HRITasks"""

        hri = self.hri_manager

        # # Adding single item
        # self.get_logger().info("Adding single item: rotten_potatoes")
        # result = hri.add_item(["rotten_potatoes"], json.dumps([{}]))
        # self.get_logger().info(f"Result: {result}")

        # # Adding multiple items with metadata
        # self.get_logger().info("Adding multiple items with metadata")
        # documents = ["apple pie with cinnamon", "banana_pie", "mango_pie_with milk"]
        # metadata = [{"category": "500"}, {"characteristic": "400"}, {"complement": "450"}]
        # result = hri.add_item(documents, json.dumps(metadata))
        # self.get_logger().info(f"Result: {result}")

        # # Querying items
        # self.get_logger().info("Querying 'potatoes' from item collection")

        # results = hri.query_item("potatoes", top_k=1)
        # self.get_logger().info(f"Query results: {hri.get_name(results)}")
        # New implementation of additems for item categorization in shelves

        # objects_to_categorize = ["yogurt", "peach", "can"]
        # objects_shelve_1 = ["milk", "cheese", "cream"]
        # objects_shelve_2 = ["beans", "tommato_soup", "corn_soup"]
        # objects_shelve_3 = ["mango", "banana", "apple"]
        # objects = [objects_shelve_1, objects_shelve_2, objects_shelve_3]
        # shelf_1 = "1"
        # shelf_2 = "2"
        # shelf_3 = "3"
        # shelves = [shelf_1, shelf_2, shelf_3]
        # shelves_with_objects = dict(zip(shelves, objects))
        # categories = {1: "dairy", 2: "fruit", 3: "empty", 4: "meat"}
        # obj = ["watermelon", "sausage", "milk", "pencil case"]
        # objects_categorized = hri.categorize_objects(obj, categories)

        # self.get_logger().info(f"classification : {objects_categorized}")

        # self.get_logger().info("Querying 'cinnamon' from item collection")
        # results = hri.query_item("cinnamon", top_k=3)
        # self.get_logger().info(f"Query results: {hri.get_name(results)}")
        # # Adding and querying location
        # self.get_logger().info("Querying 'kitchen' from location collection")

        # results_location = hri.query_location("kitchen table", top_k=1)
        # subarea = hri.get_subarea(results_location)
        # area = hri.get_area(results_location)
        # self.get_logger().info(f"Subarea: {subarea}")
        # self.get_logger().info(f"Area: {area}")
        # self.get_logger().info(f"Query results: {hri.get_name(results_location)}")

        # ---- save_command_history ----
        self.get_logger().info("Saving command history for go_to command")
        command = GoTo(action="go_to", location_to_go="kitchen")
        command_2 = GoTo(action="go_to", location_to_go="living_room")
        command_3 = GoTo(action="go_to", location_to_go="entrance")
        command_4 = GoTo(action="go_to", location_to_go="bathroom")

        hri.add_command_history(
            command=command,
            result="Success",
            status=1,
        )
        self.get_clock().sleep_for(Duration(seconds=2))
        hri.add_command_history(
            command=command_2,
            result="Success",
            status=1,
        )
        self.get_clock().sleep_for(Duration(seconds=2))
        hri.add_command_history(
            command=command_3,
            result="Failure",
            status=1,
        )
        self.get_clock().sleep_for(Duration(seconds=2))
        hri.add_command_history(
            command=command_4,
            result="Success",
            status=1,
        )
        self.get_clock().sleep_for(Duration(seconds=2))
        self.get_logger().info("Querying command_history collection for the saved command")
        history = hri.query_command_history("go_to", 3)
        # context = hri.get_context(history)
        result = hri.get_result(history)
        status = hri.get_status(history)

        self.get_logger().info(f"history query results: {history}")
        self.get_logger().info(f"result history query results: {result}")
        self.get_logger().info(f"status history query results: {status}")

        # # ---- end save_command_history ----

        # self.get_logger().info("TESTING THE FIND CLOSEST FUNCTION")
        # # Test find_closest

        # documents = ["cheese", "milk", "yogurt"]
        # result_closest = hri.find_closest(documents, "milk")
        # self.get_logger().info(f"Closest result: {result_closest}")

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
        self.get_logger().info("Testing object location retrieval...")
        object_name = "cheese"
        res = self.hri_manager.query_location_with_context(object_name, top_k=10)

        for i, location in enumerate(res):
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

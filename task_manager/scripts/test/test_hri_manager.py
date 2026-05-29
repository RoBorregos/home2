#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import csv
import json
import os
import subprocess
import time
from datetime import datetime
from types import SimpleNamespace
from typing import Union

import rclpy
from rclpy.node import Node

from _merger_helpers import (
    fallback_preserves_all,
    gripper_invariant_holds,
    non_goto_actions_preserved,
    per_command_order_preserved,
)
from task_manager.subtask_managers.hri_tasks import HRITasks

# from subtask_managers.subtask_meta import SubtaskMeta
from task_manager.utils.baml_client.types import (
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
from task_manager.utils.status import Status
from task_manager.utils.task import Task

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

COMMAND_INTERPRETER_SUCCESS_THRESHOLD = 0.9  # Higher than 1 for exact match only

# Choose which tests to perform
TEST_ASK_AND_CONFIRM = False
TEST_INDIVIDUAL_FUNCTIONS = False
TEST_CATEGORIZE_SHELVES = False
TEST_ASYNC_LLM = False
TEST_STREAMING = False
TEST_MAP = False
TEST_OBJECT_LOCATION = False
TEST_IS_POSITIVE = False
TEST_IS_NEGATIVE = False
TEST_DATA_EXTRACTOR = False
TEST_COMMAND_INTERPRETER = False
TEST_COMMAND_INTERPRETER_BAML = False
TEST_WORD_CONFIDENCES = False
TEST_TAKE_ORDER = False
TEST_MERGER = True


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.hri_manager = HRITasks(self, task=Task.DEBUG, mock_data=False)
        rclpy.spin_once(self, timeout_sec=1.0)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        if TEST_ASK_AND_CONFIRM:
            self.test_ask_and_confirm()

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

        if TEST_COMMAND_INTERPRETER:
            self.test_command_interpreter()

        if TEST_COMMAND_INTERPRETER_BAML:
            self.test_command_interpreter_baml()

        if TEST_WORD_CONFIDENCES:
            self.test_word_confidences()

        if TEST_TAKE_ORDER:
            self.test_take_order()

        if TEST_MERGER:
            self.test_merger()

        exit(0)

    def individual_functions(self):
        # Test say
        self.hri_manager.say("Hi, my name is frida. What is your favorite drink?", wait=True)
        self.get_logger().info("Hearing from the user...")

        # Test hear
        s, user_request, _ = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        # Test extract_data
        s, drink = self.hri_manager.extract_data("Drink", user_request)
        self.get_logger().info(f"Extracted data: {drink}")

        self.hri_manager.say("Hi, my name is frida. What is your name?", wait=True)
        self.get_logger().info("Hearing from the user...")

        # Test hear
        s, user_request, _ = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        # Test extract_data
        s, drink = self.hri_manager.extract_data("name", user_request)
        self.get_logger().info(f"Extracted data: {drink}")

    def test_streaming(self):
        s, user_request, _ = self.hri_manager.hear()
        self.get_logger().info(f"Heard: {user_request}")

        s, keyword = self.hri_manager.interpret_keyword(["yes", "no", "maybe"], timeout=5.0)
        self.get_logger().info(f"Interpreted keyword: {keyword}")

    def test_word_confidences(self):
        self.hri_manager.say("Please say something.", wait=True)
        s, transcription, word_confidences = self.hri_manager.hear()

        if s != Status.EXECUTION_SUCCESS:
            self.get_logger().warn("No speech detected.")
            return

        self.get_logger().info(f"Transcription: {transcription}")
        self.get_logger().info("Word confidences:")
        for word, confidence in word_confidences.items():
            self.get_logger().info(f"  {word:20s} -> {confidence:.4f}")

        if word_confidences:
            avg_confidence = sum(word_confidences.values()) / len(word_confidences)
            self.get_logger().info(f"Average confidence: {avg_confidence:.4f}")

    def test_take_order(self):
        self.get_logger().info("Running take_order test")
        self.hri_manager.take_order(retries=3)

    def test_ask_and_confirm(self):
        s, name = self.hri_manager.ask_and_confirm(
            "What is your name?",
            "name",
            "The question 'What is your name?' was asked, full_text corresponds to the response.",
        )

        self.hri_manager.say(f"Hi {name}, nice to meet you!", wait=True)

        s, interest1 = self.hri_manager.ask_and_confirm(
            "What is your main interest?",
            "LLM_interest",
            "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
            confirm_preference,
        )

        s, interest2 = self.hri_manager.ask_and_confirm(
            "What is your favorite second interest?",
            "LLM_interest",
            "The question 'What is your favorite main interest?' was asked, full_text corresponds to the response.",
            confirm_preference,
        )

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

    def test_command_interpreter(self):
        test_cases_file = os.path.join(DATA_DIR, "command_interpreter.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"command_interpreter_{date_str}.csv")

        results = []
        passed_tests = 0

        for i, test_case in enumerate(test_cases, 1):
            input_text = test_case["string_cmd"]
            expected_output = test_case["structured_cmd"]

            self.get_logger().info(f"Test case {i}")

            actual_output = None
            success = False

            try:
                s, command_list = self.hri_manager.command_interpreter(input_text)

                if s == Status.EXECUTION_SUCCESS:
                    actual_output = command_list
                    success = True

                    # Parse expected_output string into list of command objects
                    expected_commands = eval(expected_output)

                    # Check if lists have the same length
                    if len(command_list) != len(expected_commands):
                        success = False
                        self.get_logger().error(
                            f"Command list length mismatch: {len(command_list)} vs {len(expected_commands)}"
                        )
                    else:
                        # Compare each command in the list
                        for cmd_idx, (actual_cmd, expected_cmd) in enumerate(
                            zip(command_list, expected_commands)
                        ):
                            # Direct comparison first
                            if actual_cmd == expected_cmd:
                                self.get_logger().info(f"Command {cmd_idx + 1}: Exact match")
                                continue

                            # Use the embeddings service instead of the removed local pg object.
                            s, closest = self.hri_manager.find_closest(
                                [str(expected_cmd)], str(actual_cmd), top_k=1
                            )
                            if s != Status.EXECUTION_SUCCESS or not closest.similarities:
                                success = False
                                self.get_logger().error(
                                    f"Command {cmd_idx + 1} similarity lookup failed: {s}"
                                )
                                break

                            similarity = closest.similarities[0]
                            self.get_logger().info(
                                f"Command {cmd_idx + 1}: Cosine similarity = {similarity:.4f}"
                            )

                            if similarity < COMMAND_INTERPRETER_SUCCESS_THRESHOLD:
                                success = False
                                self.get_logger().error(
                                    f"Command {cmd_idx + 1} failed: {actual_cmd} vs {expected_cmd}"
                                )
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

            results.append([i, input_text, expected_output, actual_output, success])
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "test_number",
                    "input",
                    "expected_output",
                    "actual_output",
                    "success",
                ]
            )
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")

    def test_merger(self):
        """Unit-test task_manager.gpsr.merger against several interleaving
        scenarios. Pure-Python — does not depend on any ROS service. Mirrors
        the in-tree convention: per-scenario PASS/FAIL log + CSV output."""

        # Per-scenario capture of the inputs the closures build internally, so
        # the run loop can report the commands / theoretical positions without
        # every scenario having to return them explicitly.
        from task_manager.gpsr.merger import merge as _real_merge

        cap = {"cmds": [], "coords": {}, "origins": []}

        def _act(kind, **fields):
            return SimpleNamespace(action=kind, **fields)

        def _cmd(*actions):
            c = SimpleNamespace(commands=list(actions))
            cap["cmds"].append(c)
            return c

        def _make_locator(coords):
            cap["coords"] = coords

            def locator(name):
                return coords.get(name)

            return locator

        def merge(commands, *args, **kwargs):
            if "origin" in kwargs:
                cap["origins"].append(kwargs["origin"])
            return _real_merge(commands, *args, **kwargs)

        def _gripper_walk(plan_actions):
            """Stricter than gripper_invariant_holds: also asserts that
            pick→drop is not crossed by another command's gripper action."""
            held_by = None
            for pa in plan_actions:
                if pa.requires_gripper:
                    if held_by is not None:
                        return False
                    held_by = pa.source_cmd
                if pa.releases_gripper:
                    if held_by is None or held_by != pa.source_cmd:
                        return False
                    held_by = None
            return True

        _ARG_FIELDS = (
            "location_to_go",
            "object_to_pick",
            "name",
            "attribute_value",
            "destination",
            "destination_room",
            "info_type",
            "target_to_count",
            "object_category",
        )

        def _label(action):
            kind = getattr(action, "action", "?")
            for arg in _ARG_FIELDS:
                v = getattr(action, arg, None)
                if v:
                    return f"{kind}({v})"
            return kind

        def _describe(pa):
            return f"[c{pa.source_cmd}] {_label(pa.action)}"

        def _describe_plan(plan_actions):
            return " -> ".join(_describe(pa) for pa in plan_actions)

        def _describe_cmd(cmd):
            return " -> ".join(_label(a) for a in cmd.commands)

        scenarios = []

        # 1. Single command identity
        def s_single():
            cmd = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("pick_object", object_to_pick="cup"),
                _act("go_to", location_to_go="living_room"),
                _act("find_person_by_name", name="Angel"),
                _act("give_object"),
            )
            plan = merge([cmd], locator=_make_locator({}))
            ok = (
                [pa.source_idx for pa in plan.actions] == [0, 1, 2, 3, 4]
                and all(pa.source_cmd == 0 for pa in plan.actions)
                and gripper_invariant_holds(plan.actions)
                and per_command_order_preserved(plan.actions, n_cmds=1)
            )
            return ok, plan

        scenarios.append(("single_command_identity", s_single))

        # 2. Two pick-deliver pairs — atomic blocks preserved
        def s_two_blocks():
            cmd_a = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("pick_object", object_to_pick="cup"),
                _act("go_to", location_to_go="bathroom"),
                _act("find_person_by_name", name="Angel"),
                _act("give_object"),
            )
            cmd_b = _cmd(
                _act("go_to", location_to_go="bedroom"),
                _act("pick_object", object_to_pick="book"),
                _act("go_to", location_to_go="office"),
                _act("find_person_by_name", name="Bob"),
                _act("give_object"),
            )
            coords = {
                "kitchen": (0.0, 0.0),
                "bathroom": (1.0, 0.0),
                "bedroom": (10.0, 0.0),
                "office": (11.0, 0.0),
            }
            plan = merge([cmd_a, cmd_b], locator=_make_locator(coords))
            delivers = [pa for pa in plan.actions if pa.action.action == "give_object"]
            ok = (
                _gripper_walk(plan.actions)
                and per_command_order_preserved(plan.actions, n_cmds=2)
                and len(delivers) == 2
                and {pa.source_cmd for pa in delivers} == {0, 1}
            )
            return ok, plan

        scenarios.append(("two_pick_deliver_blocks_atomic", s_two_blocks))

        # 3. Shared waypoint visited efficiently — collapse pass dedups the
        # redundant second go_to kitchen, leaving exactly one visit.
        def s_shared_waypoint():
            cmd_a = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("count", target_to_count="apples"),
            )
            cmd_b = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("count", target_to_count="oranges"),
            )
            cmd_c = _cmd(
                _act("go_to", location_to_go="garage"),
                _act("count", target_to_count="cars"),
            )
            coords = {"kitchen": (5.0, 5.0), "garage": (-5.0, -5.0)}
            plan = merge([cmd_a, cmd_b, cmd_c], locator=_make_locator(coords))
            kitchen_gotos = [
                pa
                for pa in plan.actions
                if pa.action.action == "go_to" and pa.action.location_to_go == "kitchen"
            ]
            # Both counts (cmd0 apples + cmd1 oranges) should appear before
            # the garage go_to, i.e. on the same kitchen trip.
            try:
                garage_idx = next(
                    i
                    for i, pa in enumerate(plan.actions)
                    if pa.action.action == "go_to" and pa.action.location_to_go == "garage"
                )
            except StopIteration:
                return False, plan
            counts_before_garage = [
                pa for pa in plan.actions[:garage_idx] if pa.action.action == "count"
            ]
            return (
                len(kitchen_gotos) == 1
                and len(counts_before_garage) == 2
                and {pa.source_cmd for pa in counts_before_garage} == {0, 1}
            ), plan

        scenarios.append(("shared_waypoint_visited_efficiently", s_shared_waypoint))

        # 4. find→get_info precedence preserved
        def s_find_then_info():
            cmd_a = _cmd(
                _act("go_to", location_to_go="couch"),
                _act("find_person", attribute_value="standing"),
                _act("get_person_info", info_type="pose"),
            )
            cmd_b = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("find_person_by_name", name="Bob"),
                _act("get_person_info", info_type="name"),
            )
            coords = {"couch": (0.0, 0.0), "kitchen": (10.0, 10.0)}
            plan = merge([cmd_a, cmd_b], locator=_make_locator(coords))
            ok = True
            for cmd_idx in (0, 1):
                cmd_actions = [pa for pa in plan.actions if pa.source_cmd == cmd_idx]
                find_pos = next(
                    (
                        i
                        for i, pa in enumerate(cmd_actions)
                        if pa.action.action.startswith("find_person")
                    ),
                    None,
                )
                info_pos = next(
                    (
                        i
                        for i, pa in enumerate(cmd_actions)
                        if pa.action.action == "get_person_info"
                    ),
                    None,
                )
                if find_pos is None or info_pos is None or find_pos >= info_pos:
                    ok = False
                    break
            return ok, plan

        scenarios.append(("find_then_get_info_precedence", s_find_then_info))

        # 5. Pure-find commands interleave by NN
        def s_no_gripper_interleave():
            cmd_a = _cmd(
                _act("go_to", location_to_go="A"),
                _act("count", target_to_count="x"),
            )
            cmd_b = _cmd(
                _act("go_to", location_to_go="B"),
                _act("count", target_to_count="y"),
            )
            cmd_c = _cmd(
                _act("go_to", location_to_go="C"),
                _act("count", target_to_count="z"),
            )
            coords = {"A": (3.0, 0.0), "B": (1.0, 0.0), "C": (5.0, 0.0)}
            plan = merge([cmd_a, cmd_b, cmd_c], locator=_make_locator(coords))
            go_tos = [
                pa.action.location_to_go for pa in plan.actions if pa.action.action == "go_to"
            ]
            ok = go_tos == ["B", "A", "C"] and per_command_order_preserved(plan.actions, n_cmds=3)
            return ok, plan

        scenarios.append(("no_gripper_commands_interleave_freely", s_no_gripper_interleave))

        # 6. Auxiliary say-with-context command does not split a gripper block
        def s_aux_say_does_not_split():
            cmd_a = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("pick_object", object_to_pick="snack"),
                _act("go_to", location_to_go="lobby"),
                _act("place_object"),
            )
            cmd_say = _cmd(
                _act("go_to", location_to_go="start"),
                _act("say_with_context", user_instruction="hello", previous_command_info=[]),
            )
            coords = {"kitchen": (0, 0), "lobby": (1, 0), "start": (0, 5)}
            plan = merge([cmd_a, cmd_say], locator=_make_locator(coords))
            cmd_a_positions = [i for i, pa in enumerate(plan.actions) if pa.source_cmd == 0]
            contiguous = cmd_a_positions == list(
                range(min(cmd_a_positions), max(cmd_a_positions) + 1)
            )
            return _gripper_walk(plan.actions) and contiguous, plan

        scenarios.append(("say_with_context_does_not_split_block", s_aux_say_does_not_split))

        # 7. Empty input → empty plan
        def s_empty():
            plan = merge([])
            return plan.actions == [] and plan.fallback == [], plan

        scenarios.append(("empty_commands", s_empty))

        # 8. Fallback preserves original sequences
        def s_fallback():
            cmd_a = _cmd(
                _act("go_to", location_to_go="kitchen"),
                _act("count", target_to_count="apples"),
            )
            cmd_b = _cmd(_act("answer_question"))
            plan = merge([cmd_a, cmd_b])
            ok = (
                len(plan.fallback) == 2
                and [pa.source_idx for pa in plan.fallback[0]] == [0, 1]
                and [pa.source_idx for pa in plan.fallback[1]] == [0]
                and plan.fallback[0][0].action.action == "go_to"
                and plan.fallback[1][0].action.action == "answer_question"
            )
            return ok, plan

        scenarios.append(("fallback_preserves_original_sequences", s_fallback))

        # 9. Option B: a neutral segment from another command interleaves
        # inside an atomic block when geometry favors it.
        def s_neutral_interleaves_atomic_block():
            cmd_pick = _cmd(
                _act("go_to", location_to_go="A"),
                _act("pick_object", object_to_pick="cup"),
                _act("go_to", location_to_go="B"),
                _act("give_object"),
            )
            cmd_neutral = _cmd(
                _act("go_to", location_to_go="C"),
                _act("count", target_to_count="x"),
            )
            # C between A and B → A→C→B (10) cheaper than not-interleaving.
            coords = {"A": (0.0, 0.0), "B": (10.0, 0.0), "C": (5.0, 0.0)}
            plan = merge([cmd_pick, cmd_neutral], locator=_make_locator(coords))
            kinds = [pa.action.action for pa in plan.actions]
            # Expect: go_to A, pick, go_to C, count, go_to B, give_object.
            ok = (
                kinds
                == [
                    "go_to",
                    "pick_object",
                    "go_to",
                    "count",
                    "go_to",
                    "give_object",
                ]
                and gripper_invariant_holds(plan.actions)
                and per_command_order_preserved(plan.actions, n_cmds=2)
            )
            return ok, plan

        scenarios.append(
            ("option_b_neutral_interleaves_atomic_block", s_neutral_interleaves_atomic_block)
        )

        # 10. Collapse: redundant back-to-back go_to to the same location is
        # dropped from the flat plan.
        def s_collapse_redundant_go_to():
            cmd_find = _cmd(
                _act("go_to", location_to_go="A"),
                _act("find_person", attribute_value="standing"),
            )
            cmd_pick = _cmd(
                _act("go_to", location_to_go="A"),
                _act("pick_object", object_to_pick="cup"),
                _act("go_to", location_to_go="B"),
                _act("give_object"),
            )
            coords = {"A": (0.0, 0.0), "B": (10.0, 0.0)}
            plan = merge([cmd_find, cmd_pick], locator=_make_locator(coords))
            go_to_a = [
                pa
                for pa in plan.actions
                if pa.action.action == "go_to" and pa.action.location_to_go == "A"
            ]
            return len(go_to_a) == 1, plan

        scenarios.append(("collapse_redundant_back_to_back_go_to", s_collapse_redundant_go_to))

        # 11. Origin: with origin near A, A is scheduled first.
        def s_origin_biases_first():
            cmd_a = _cmd(_act("go_to", location_to_go="A"), _act("count", target_to_count="x"))
            cmd_b = _cmd(_act("go_to", location_to_go="B"), _act("count", target_to_count="y"))
            coords = {"A": (4.0, 0.0), "B": (20.0, 0.0)}
            plan_near_a = merge([cmd_a, cmd_b], locator=_make_locator(coords), origin=(5.0, 0.0))
            plan_near_b = merge([cmd_a, cmd_b], locator=_make_locator(coords), origin=(19.0, 0.0))
            first_a = plan_near_a.actions[0].action.location_to_go
            first_b = plan_near_b.actions[0].action.location_to_go
            return first_a == "A" and first_b == "B", plan_near_a

        scenarios.append(("origin_biases_first_segment", s_origin_biases_first))

        # 12. origin=None: no origin bias; tiebreaker keeps cmd0 first when
        # all real distances are equal.
        def s_origin_none_no_bias():
            cmd_a = _cmd(_act("go_to", location_to_go="A"), _act("count", target_to_count="x"))
            cmd_b = _cmd(_act("go_to", location_to_go="B"), _act("count", target_to_count="y"))
            # Mirror-image positions across the (hypothetical) (0,0); under
            # origin=None the DP should not favour either based on map origin.
            coords = {"A": (-5.0, 0.0), "B": (5.0, 0.0)}
            plan = merge([cmd_a, cmd_b], locator=_make_locator(coords), origin=None)
            return plan.actions[0].action.location_to_go == "A", plan

        scenarios.append(("origin_none_no_map_origin_bias", s_origin_none_no_bias))

        # 13. All-unknown locations → per-command sequential by tiebreaker.
        def s_all_unknown_sequential():
            cmd_a = _cmd(
                _act("go_to", location_to_go="ghost1"),
                _act("count", target_to_count="x"),
            )
            cmd_b = _cmd(
                _act("go_to", location_to_go="ghost2"),
                _act("count", target_to_count="y"),
            )
            plan = merge([cmd_a, cmd_b], locator=_make_locator({}))
            order = [pa.source_cmd for pa in plan.actions if pa.action.action == "go_to"]
            return order == [0, 1] and per_command_order_preserved(plan.actions, n_cmds=2), plan

        scenarios.append(("all_unknown_locations_sequential", s_all_unknown_sequential))

        # 14. LARGE_M penalty: known segments are not pushed behind unknowns.
        def s_known_before_unknown():
            cmd_known = _cmd(
                _act("go_to", location_to_go="A"),
                _act("count", target_to_count="x"),
            )
            cmd_unknown = _cmd(
                _act("go_to", location_to_go="ghost"),
                _act("count", target_to_count="y"),
            )
            coords = {"A": (0.0, 0.0)}  # ghost unresolved
            plan = merge(
                [cmd_known, cmd_unknown],
                locator=_make_locator(coords),
                origin=(0.0, 0.0),
            )
            first_loc = plan.actions[0].action.location_to_go
            return first_loc == "A", plan

        scenarios.append(("large_m_keeps_known_first", s_known_before_unknown))

        # 15. M > 16 → fallback to per-command sequential without raising.
        def s_m_cap_fallback():
            cmds = [
                _cmd(
                    _act("go_to", location_to_go=f"loc{i}"),
                    _act("count", target_to_count="x"),
                )
                for i in range(17)
            ]
            coords = {f"loc{i}": (float(i), 0.0) for i in range(17)}
            plan = merge(cmds, locator=_make_locator(coords))
            order = [pa.source_cmd for pa in plan.actions if pa.action.action == "go_to"]
            return order == list(range(17)) and len(plan.actions) == 34, plan

        scenarios.append(("m_cap_fallback_to_sequential", s_m_cap_fallback))

        # 16. Regression: the three canonical GPSR utterances merged as a batch
        # must not drop any command or any command's work. This guards the
        # reported "loses the first part of the first command" symptom: the
        # merger is deterministic and preserves every command, so a real loss
        # can only come from upstream LLM decomposition, never from merge().
        def _example_commands():
            c_escort = _cmd(
                _act("go_to", location_to_go="chairs"),
                _act("find_person", attribute_value="wearing a black blouse"),
                _act("guide_person_to", destination_room="bathroom"),
            )
            c_tell = _cmd(
                _act("go_to", location_to_go="side_tables"),
                _act("get_visual_info", measure="biggest", object_category="object"),
                _act("go_to", location_to_go="start_location"),
                _act(
                    "say_with_context", user_instruction="biggest object", previous_command_info=[]
                ),
            )
            c_meet = _cmd(
                _act("go_to", location_to_go="entrance"),
                _act("find_person_by_name", name="adel"),
                _act("follow_person_until", destination="waste_basket"),
            )
            coords = {
                "chairs": (0.0, 0.0),
                "bathroom": (2.0, 1.0),
                "side_tables": (5.0, 5.0),
                "start_location": (0.0, 0.0),
                "entrance": (-3.0, 2.0),
                "waste_basket": (-5.0, 4.0),
            }
            return [c_escort, c_tell, c_meet], coords

        def s_example_batch_no_loss():
            cmds, coords = _example_commands()
            plan = merge(cmds, locator=_make_locator(coords), origin=(0.0, 0.0))
            ok = (
                non_goto_actions_preserved(plan, cmds)
                and fallback_preserves_all(plan, cmds)
                and gripper_invariant_holds(plan.actions)
                and per_command_order_preserved(plan.actions, n_cmds=len(cmds))
                # the user's FIRST command's first action survives into the plan
                and any(pa.source_cmd == 0 and pa.source_idx == 0 for pa in plan.actions)
            )
            return ok, plan

        scenarios.append(("example_batch_no_command_lost", s_example_batch_no_loss))

        # 17. Same three commands, origin=None. The schedule may reorder which
        # command runs first (no map-origin bias), but still nothing is lost.
        def s_example_batch_origin_none_no_loss():
            cmds, coords = _example_commands()
            plan = merge(cmds, locator=_make_locator(coords), origin=None)
            ok = (
                non_goto_actions_preserved(plan, cmds)
                and fallback_preserves_all(plan, cmds)
                and per_command_order_preserved(plan.actions, n_cmds=len(cmds))
            )
            return ok, plan

        scenarios.append(("example_batch_origin_none_no_loss", s_example_batch_origin_none_no_loss))

        # 18. Shared start location: the tiebreak must protect cmd0's leading
        # go_to so it is the surviving visit and cmd0's first part is never the
        # one collapsed away (only the later command's redundant go_to is).
        def s_shared_start_first_command_protected():
            cmd0 = _cmd(
                _act("go_to", location_to_go="chairs"),
                _act("find_person", attribute_value="black blouse"),
                _act("guide_person_to", destination_room="bathroom"),
            )
            cmd1 = _cmd(
                _act("go_to", location_to_go="chairs"),
                _act("count", target_to_count="cups"),
            )
            coords = {"chairs": (10.0, 0.0), "bathroom": (50.0, 50.0)}
            plan = merge([cmd0, cmd1], locator=_make_locator(coords), origin=(9.0, 0.0))
            chairs_gotos = [
                pa
                for pa in plan.actions
                if pa.action.action == "go_to" and pa.action.location_to_go == "chairs"
            ]
            first_goto = next((pa for pa in plan.actions if pa.action.action == "go_to"), None)
            ok = (
                non_goto_actions_preserved(plan, [cmd0, cmd1])
                and len(chairs_gotos) == 1
                # the one surviving go_to chairs belongs to cmd0, not cmd1
                and chairs_gotos[0].source_cmd == 0
                and first_goto is not None
                and first_goto.source_cmd == 0
            )
            return ok, plan

        scenarios.append(
            ("shared_start_first_command_goto_protected", s_shared_start_first_command_protected)
        )

        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"merger_{date_str}.csv")

        results = []
        passed = 0
        for name, fn in scenarios:
            cap["cmds"] = []
            cap["coords"] = {}
            cap["origins"] = []
            self.get_logger().info(f"Merger scenario: {name}")

            plan = None
            exc = None
            try:
                ok, plan = fn()
            except Exception as e:
                ok = False
                exc = e

            if cap["cmds"]:
                self.get_logger().info("  commands:")
                for i, c in enumerate(cap["cmds"]):
                    self.get_logger().info(f"    c{i}: {_describe_cmd(c)}")
                cmds_str = " | ".join(
                    f"c{i}: {_describe_cmd(c)}" for i, c in enumerate(cap["cmds"])
                )
            else:
                self.get_logger().info("  commands: (none)")
                cmds_str = "(none)"

            if cap["coords"]:
                pos_str = ", ".join(f"{k}={v}" for k, v in cap["coords"].items())
            else:
                pos_str = "(no resolvable positions)"
            if cap["origins"]:
                origins = cap["origins"]
                pos_str += f" | origin={origins[0] if len(origins) == 1 else origins}"
            self.get_logger().info(f"  positions: {pos_str}")

            if exc is not None:
                self.get_logger().error(f"  EXCEPTION: {exc}")
                results.append([name, False, "EXCEPTION", cmds_str, pos_str, str(exc)])
                continue

            ordering = _describe_plan(plan.actions) if plan and plan.actions else "(empty)"
            self.get_logger().info(f"  ordering: {ordering}")
            if ok:
                passed += 1
                self.get_logger().info(f"  PASS ({len(plan.actions)} actions)")
            else:
                self.get_logger().error(f"  FAIL ({len(plan.actions)} actions)")
            results.append([name, ok, len(plan.actions), cmds_str, pos_str, ordering])

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["scenario", "passed", "plan_size", "commands", "positions", "ordering_or_error"]
            )
            writer.writerows(results)

        self.get_logger().info(f"Merger results: {passed}/{len(scenarios)} passed")
        self.get_logger().info(f"Saved to {output_file}")

    def test_command_interpreter_baml(self):
        # Prepare output file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"command_interpreter_baml_{date_str}.txt")

        self.get_logger().info("Running baml-cli test for command interpreter")
        self.get_logger().info("This may take a minute...")
        result = subprocess.run(
            ["baml-cli", "test"],
            cwd="/workspace/src/task_manager/task_manager/utils/",
            capture_output=True,
            text=True,
        )

        # Print to terminal
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr)

        # Save to file
        with open(output_file, "w") as f:
            f.write("=== STDOUT ===\n")
            f.write(result.stdout or "")
            f.write("\n=== STDERR ===\n")
            f.write(result.stderr or "")
            f.write(f"\n=== RETURN CODE: {result.returncode} ===\n")

        self.get_logger().info(f"Results saved to {output_file}")


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

#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import csv
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Union

import rclpy
from rclpy.node import Node
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
BENCHMARK_DIR = "/workspace/src/hri/benchmarks/nlp"

COMMAND_INTERPRETER_SUCCESS_THRESHOLD = 0.9  # Higher than 1 for exact match only

# --- TEST_NLP mode: env-driven benchmark via the real HRI pipeline ---
# When TEST_NLP=true, hardcoded TEST_* booleans below are ignored. Tasks come
# from NLP_TASKS (comma-separated), and a benchmark-style JSON is emitted in
# addition to the per-task CSVs.
TEST_NLP = os.getenv("TEST_NLP", "false").lower() == "true"
NLP_MODEL_ALIAS = os.getenv("NLP_MODEL_ALIAS", "")
NLP_OLLAMA_URL = os.getenv("NLP_OLLAMA_URL", "")
NLP_TASKS = [t for t in os.getenv("NLP_TASKS", "").split(",") if t]
NLP_RUNS = int(os.getenv("NLP_RUNS", "3"))
NLP_RESULTS_DIR = os.getenv("NLP_RESULTS_DIR", OUTPUT_DIR)

# Choose which tests to perform (used only when TEST_NLP is unset)
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


class TestHriManager(Node):
    def __init__(self):
        super().__init__("test_hri_task_manager")
        self.hri_manager = HRITasks(self, task=Task.DEBUG, mock_data=False)
        rclpy.spin_once(self, timeout_sec=1.0)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        if TEST_NLP:
            self.run_nlp_benchmark()
            exit(0)

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

        # Load isolated test objects data to ensure stability
        test_objects_file = os.path.join(DATA_DIR, "test_objects.json")
        with open(test_objects_file, "r") as f:
            self.hri_manager.objects_data = json.load(f)

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

                        # Handle category validation
                        actual_cats = categorized_shelves[shelve]
                        expected_cats = expected[shelve]["category"]

                        if set(actual_cats) != set(expected_cats):
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
        cases = []
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
            cases.append(
                {
                    "input": input_text,
                    "expected": expected_output,
                    "got": actual_output,
                    "passed": success,
                }
            )
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["test_number", "input", "expected_output", "actual_output", "success"])
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")
        return cases

    def test_is_negative(self):
        test_cases_file = os.path.join(DATA_DIR, "is_negative.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"is_negative_{date_str}.csv")

        results = []
        cases = []
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
            cases.append(
                {
                    "input": input_text,
                    "expected": expected_output,
                    "got": actual_output,
                    "passed": success,
                }
            )
            self.get_logger().info("-" * 50)

        # Write results to CSV
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["test_number", "input", "expected_output", "actual_output", "success"])
            writer.writerows(results)

        self.get_logger().info(f"Results saved to {output_file}")
        self.get_logger().info(f"{passed_tests} out of {len(test_cases)} passed")
        return cases

    def test_data_extractor(self):
        test_cases_file = os.path.join(DATA_DIR, "data_extractor.json")
        with open(test_cases_file, "r") as f:
            test_cases = json.load(f)

        # Prepare output directory and file
        date_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_file = os.path.join(OUTPUT_DIR, f"data_extractor_{date_str}.csv")

        results = []
        cases = []
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
            cases.append(
                {
                    "input": [input_text, query, context],
                    "expected": expected_output,
                    "got": actual_output,
                    "passed": success,
                }
            )
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
        return cases

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

    # ─────────────────────────────────────────────────────────────────────
    # TEST_NLP mode: drive selected tasks via the real HRI pipeline, then
    # run a direct-HTTP perf side-channel against llama-server for tok/s,
    # and emit a benchmark-style JSON via the existing benchmark/report.py.
    # ─────────────────────────────────────────────────────────────────────

    _TASK_DISPATCH = {
        "is_positive": "test_is_positive",
        "is_negative": "test_is_negative",
        "extract_data": "test_data_extractor",
    }

    def run_nlp_benchmark(self):
        if not NLP_MODEL_ALIAS:
            self.get_logger().error("TEST_NLP set but NLP_MODEL_ALIAS is empty.")
            return
        if not NLP_TASKS:
            self.get_logger().error("TEST_NLP set but NLP_TASKS is empty.")
            return

        self.get_logger().info(
            f"TEST_NLP mode: model={NLP_MODEL_ALIAS} tasks={NLP_TASKS} "
            f"ollama={NLP_OLLAMA_URL or '(perf disabled)'}"
        )

        model_results = {}
        for task_name in NLP_TASKS:
            method_name = self._TASK_DISPATCH.get(task_name)
            if not method_name:
                self.get_logger().warn(
                    f"Unknown NLP task '{task_name}' — supported: "
                    f"{list(self._TASK_DISPATCH.keys())}"
                )
                continue
            self.get_logger().info(f"── Running accuracy: {task_name}")
            cases = getattr(self, method_name)()
            task_r = {"cases": cases or []}

            perf = self._run_perf_side_channel(task_name)
            if perf:
                task_r.update(perf)

            model_results[task_name] = task_r

        self._emit_benchmark_report({NLP_MODEL_ALIAS: model_results})

    def _run_perf_side_channel(self, task_name: str) -> dict:
        """Direct HTTP call to llama-server for tok/s + TTFT. Bypasses ROS."""
        if not NLP_OLLAMA_URL:
            return {}
        try:
            if BENCHMARK_DIR not in sys.path:
                sys.path.insert(0, BENCHMARK_DIR)
            from openai import OpenAI
            from tasks import TASK_REGISTRY, run_perf
        except ImportError as e:
            self.get_logger().warn(f"Perf side-channel skipped (missing dep): {e}")
            return {}

        task_cls = TASK_REGISTRY.get(task_name)
        if task_cls is None:
            self.get_logger().warn(f"No perf task class for '{task_name}', skipping perf.")
            return {}

        try:
            client = OpenAI(base_url=NLP_OLLAMA_URL, api_key="ollama")
            self.get_logger().info(f"   perf: {NLP_RUNS} run(s) against {NLP_OLLAMA_URL}")
            perf = run_perf(client, NLP_MODEL_ALIAS, task_cls, NLP_RUNS)
            ttft = perf.get("avg_ttft_ms")
            tps = perf.get("avg_tokens_per_s")
            self.get_logger().info(
                f"   TTFT={ttft:.0f}ms tok/s={tps:.1f}" if ttft and tps else "   (no perf data)"
            )
            return perf
        except Exception as e:
            self.get_logger().warn(f"Perf side-channel failed: {e}")
            return {}

    def _emit_benchmark_report(self, all_results: dict) -> None:
        try:
            if BENCHMARK_DIR not in sys.path:
                sys.path.insert(0, BENCHMARK_DIR)
            import report as rpt
        except ImportError as e:
            self.get_logger().warn(f"Could not import benchmark report module: {e}")
            return

        os.makedirs(NLP_RESULTS_DIR, exist_ok=True)
        path = rpt.save_json(all_results, NLP_RESULTS_DIR)
        self.get_logger().info(f"Benchmark JSON written to: {path}")
        try:
            for model, task_results in all_results.items():
                rpt.print_model_table(model, task_results)
        except Exception as e:
            self.get_logger().warn(f"print_model_table failed: {e}")


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

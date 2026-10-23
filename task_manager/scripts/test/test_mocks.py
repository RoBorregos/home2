#!/usr/bin/env python3

"""Validate that every area is fully mocked: ros2 run task_manager test_mocks.py"""

import inspect
import typing
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from task_manager.utils import decorators
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import AREAS, SubtaskManager
from task_manager.utils.task import Task

# Public methods that do not need @mockable: callbacks, setup, pure helpers and composites
EXEMPT = {
    "vision": {
        "setup_services",
        "follow_callback",
        "person_list_callback",
        "person_name_callback",
        "get_follow_face",
        "isPerson",
        "visual_info",
        "count_objects",
        "describe_person",
        "get_labels",
    },
    "navigation": {
        "setup_backup_map",
        "setup_services",
        "move_to_point",
        "rotate_in_place",
        "face_point",
        "is_ahead_of",
    },
    "manipulation": {
        "open_gripper",
        "close_gripper",
        "get_named_target",
        "pan_to",
        "point",
        "check_lower",
        "check_upper",
        "move_to_position",
    },
    "hri": {
        "setup_services",
        "execute_command",
        "feedback_callback",
        "parse_plan_to_text",
        "take_order",
        "add_command_history",
        "add_item",
        "add_location",
        "query_location",
        "get_items_embeddings",
        "get_hand_items",
        "query_command_history",
        "categorize_objects",
        "deterministic_categorization",
        "start_button_callback",
        "reset_task_status",
    },
}

# Arguments for mocks that build their value from the call arguments
ARGS = {
    "to_map_point": (PointStamped(),),
    "interpret_keyword": (["yes"], 1.0),
    "refactor_text": ("mocked text",),
}


class TestMocks(Node):
    def __init__(self):
        super().__init__("test_mocks")
        self.logs = self.declare_parameter("clear_logs", True).value

        # Skip mock delays
        decorators.time = SimpleNamespace(sleep=lambda _: None)

        self.subtask_manager = SubtaskManager(self, task=Task.DEBUG, mock_areas=list(AREAS))
        self.areas = {
            "vision": self.subtask_manager.vision,
            "navigation": self.subtask_manager.nav,
            "manipulation": self.subtask_manager.manipulation,
            "hri": self.subtask_manager.hri,
        }

        self.tests_funcs = {"Unknown area is rejected": {"func": self.check_unknown_area}}
        for area in self.areas:
            self.tests_funcs[f"{area}: coverage"] = {"func": self.check_coverage, "area": area}
            self.tests_funcs[f"{area}: mocked values"] = {"func": self.check_values, "area": area}

        print(f"\n{Logger.BOLD}Testing {len(self.tests_funcs)} mock checks..... \n")
        self.run()

    def public_methods(self, area: str):
        """Return the public bound methods of an area as (name, method)"""
        methods = inspect.getmembers(self.areas[area], inspect.ismethod)
        return [(name, method) for name, method in methods if not name.startswith("_")]

    def check_unknown_area(self):
        try:
            SubtaskManager.validate_areas(["nav"])
        except ValueError:
            return
        raise AssertionError("Unknown area was accepted")

    def check_coverage(self, area: str):
        missing = [
            name
            for name, method in self.public_methods(area)
            if not getattr(method, "mockable", False) and name not in EXEMPT[area]
        ]
        assert not missing, f"Not mockable nor exempt: {missing}"

    def check_values(self, area: str):
        errors = []
        for name, method in self.public_methods(area):
            if not getattr(method, "mockable", False):
                continue
            try:
                value = method(*ARGS.get(name, ()))
            except Exception as e:
                errors.append(f"{name} raised {e!r}")
                continue

            status = value[0] if isinstance(value, tuple) and value else value
            if isinstance(status, Status) and status != Status.EXECUTION_SUCCESS:
                errors.append(f"{name} returned {status}")

            annotation = inspect.signature(method).return_annotation
            expected = typing.get_args(annotation)
            if typing.get_origin(annotation) is tuple and Ellipsis not in expected:
                if not isinstance(value, tuple) or len(value) != len(expected):
                    errors.append(f"{name} returned {value!r}, expected {annotation}")
        assert not errors, "; ".join(errors)

    def run(self):
        passed = 0
        failed = 0
        for command, test in self.tests_funcs.items():
            kwargs = {key: value for key, value in test.items() if key != "func"}
            if Logger.run_test(f"Test - {command}", test["func"], clear_logs=self.logs, **kwargs):
                passed += 1
            else:
                failed += 1
        print()
        if failed == 0:
            print(f"  {Logger.GREEN}{Logger.BOLD}All {passed} tests passed!{Logger.RESET}\n")
        else:
            print(
                f"  {Logger.GREEN}{passed} passed{Logger.RESET}, "
                f"{Logger.RED}{failed} failed{Logger.RESET}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestMocks()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

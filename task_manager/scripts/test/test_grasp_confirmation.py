#!/usr/bin/env python3
"""Hermetic test for pick/place confirmation by re-detection (no ROS).
python3 task_manager/scripts/test/test_grasp_confirmation.py"""

import importlib.util
import os

_MOD = os.path.join(
    os.path.dirname(__file__),
    "..",
    "..",
    "task_manager",
    "utils",
    "grasp_confirmation.py",
)
_spec = importlib.util.spec_from_file_location("grasp_confirmation", _MOD)
gc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gc)


def test_count_by_class():
    assert gc.count_by_class(["spoon", "Spoon", "fork"]) == {"spoon": 2, "fork": 1}
    assert gc.count_by_class([None, "", "spoon"]) == {"spoon": 1}


def test_pick_with_duplicates():
    # 2 spoons before, 1 after -> one spoon was removed.
    before = gc.count_by_class(["spoon", "spoon", "fork"])
    after = gc.count_by_class(["spoon", "fork"])
    assert gc.picked_ok(before, after, "spoon") is True
    assert gc.picked_ok(before, after, "fork") is False


def test_pick_failed():
    before = gc.count_by_class(["spoon", "fork"])
    after = gc.count_by_class(["spoon", "fork"])
    assert gc.picked_ok(before, after, "spoon") is False


def test_not_confused_with_other_object():
    # Spoon gone, an unrelated fork appears: spoon verdict unaffected by the fork.
    before = gc.count_by_class(["spoon", "fork"])
    after = gc.count_by_class(["fork", "fork"])
    assert gc.picked_ok(before, after, "spoon") is True
    assert gc.picked_ok(before, after, "fork") is False


def test_place():
    before = gc.count_by_class(["cup"])
    after = gc.count_by_class(["cup", "spoon"])
    assert gc.placed_ok(before, after, "spoon") is True
    assert gc.placed_ok(before, after, "cup") is False


if __name__ == "__main__":
    test_count_by_class()
    test_pick_with_duplicates()
    test_pick_failed()
    test_not_confused_with_other_object()
    test_place()
    print("test_grasp_confirmation: OK")

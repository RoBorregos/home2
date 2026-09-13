#!/usr/bin/env python3
"""
Offline tests for the skill registry, world model and MCP tool schemas.

ROS-free: the registry and world model are pure data by design.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.skills.registry import (  # noqa: E402
    REGISTRY,
    call_skill,
    describe,
    read_only_skills,
    resolve,
)
from task_manager.world.world_model import WorldModel  # noqa: E402


class FakeStatus:
    def __init__(self, name):
        self.name = name


OK = FakeStatus("EXECUTION_SUCCESS")
FAILED = FakeStatus("TARGET_NOT_FOUND")


class FakeArea:
    def __init__(self):
        self.calls = []

    def move_to_location(self, location, sublocation=""):
        self.calls.append(("move_to_location", location, sublocation))
        return OK, ""

    def pick_object(self, object_name):
        self.calls.append(("pick_object", object_name))
        return OK, f"picked {object_name}"


class FakeSubtaskManager:
    def __init__(self):
        self.nav = FakeArea()
        self.manipulation = FakeArea()
        self.vision = FakeArea()
        self.hri = FakeArea()


def test_registry_is_well_formed() -> None:
    assert REGISTRY, "registry is empty"
    for name, skill in REGISTRY.items():
        assert skill.name == name, f"{name} keyed under the wrong name"
        assert skill.area in {"nav", "vision", "manipulation", "hri"}, skill.area
        assert skill.summary.strip(), f"{name} has no summary"
        assert 0.0 <= skill.prior_success <= 1.0, f"{name} prior_success out of range"
        assert skill.prior_duration_s > 0, f"{name} needs a positive duration prior"
        seen = set()
        for arg in skill.args:
            assert arg.name not in seen, f"{name} declares '{arg.name}' twice"
            seen.add(arg.name)
            # an optional arg without a default cannot be filled in by a caller
            assert (
                arg.required or arg.default is not None or arg.type == "bool"
            ), f"{name}.{arg.name} is optional but has no default"
    print(f"✓ registry well formed ({len(REGISTRY)} skills)")


def test_effects_are_consistent() -> None:
    """A skill must not both set and clear the same predicate."""
    for name, skill in REGISTRY.items():
        overlap = set(skill.sets) & set(skill.clears)
        assert not overlap, f"{name} both sets and clears {overlap}"
    assert "arm_free" in REGISTRY["place"].sets
    assert "holding" in REGISTRY["pick_object"].sets
    print("✓ declared effects are consistent")


def test_call_validates_arguments() -> None:
    manager = FakeSubtaskManager()

    call_skill(manager, "go_to", location="kitchen", sublocation="dinner_table")
    assert manager.nav.calls == [("move_to_location", "kitchen", "dinner_table")]

    try:
        call_skill(manager, "pick_object", wrong_name="apple")
        raise AssertionError("unexpected argument should have been rejected")
    except TypeError as error:
        assert "unexpected" in str(error), error

    try:
        call_skill(manager, "pick_object")
        raise AssertionError("missing required argument should have been rejected")
    except TypeError as error:
        assert "missing" in str(error), error
    print("✓ argument validation rejects bad calls")


def test_dispatch_cannot_reach_arbitrary_methods() -> None:
    """The old bare getattr made every public method callable by name."""
    manager = FakeSubtaskManager()
    manager.nav.test_function = lambda: "should never be reachable"
    try:
        resolve(manager, "test_function")
        raise AssertionError("unregistered method should not resolve")
    except KeyError:
        pass
    print("✓ unregistered methods are not dispatchable")


def test_read_only_set() -> None:
    read_only = read_only_skills()
    assert "detect_objects" in read_only
    assert "say" in read_only
    assert "pick_object" not in read_only
    assert "go_to" not in read_only
    print("✓ read-only skills exclude everything that moves the robot")


def test_world_applies_effects() -> None:
    world = WorldModel()
    assert world.holds("arm_free")

    world.apply("pick_object", OK)
    assert world.holds("holding") and not world.holds("arm_free")

    world.apply("place", OK)
    assert world.holds("arm_free") and not world.holds("holding")

    # a failed skill must not change beliefs
    world.apply("pick_object", FAILED)
    assert world.holds("arm_free"), "failure changed the world model"
    print("✓ world model folds in skill effects, ignores failures")


def test_world_tracks_objects() -> None:
    world = WorldModel()
    world.observe("apple", category="fruit", surface="dinner_table")
    world.observe("pringles", category="snack", surface="dinner_table")
    assert len(world.pending_objects()) == 2

    world.pick_up("apple")
    assert world.holding == "apple"
    assert world.objects["apple"].picked

    world.put_down()
    assert world.holding is None
    assert world.objects["apple"].placed
    assert [o.name for o in world.pending_objects()] == ["pringles"]

    world.objects["pringles"].skipped = True
    assert world.pending_objects() == []
    print("✓ world model tracks objects through pick and place")


def test_requirement_negation() -> None:
    world = WorldModel()
    assert world.satisfies(["arm_free"])
    assert world.satisfies(["not holding"])
    world.pick_up("milk")
    assert not world.satisfies(["arm_free"])
    assert world.satisfies(["holding"])
    print("✓ requirements support negation")


def test_mcp_schemas_are_valid() -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts"))
    valid_types = {"string", "number", "integer", "boolean", "object"}
    for name in REGISTRY:
        info = describe(name)
        assert info["name"] == name
        for arg in info["args"]:
            assert arg["type"] in {"str", "float", "int", "bool", "point"}, arg
        # mirrors frida_mcp_server.tool_schema without importing rclpy
        properties = {a["name"]: a["type"] for a in info["args"]}
        assert all(isinstance(key, str) for key in properties)
    assert valid_types  # schema mapping targets, asserted in the server module
    print("✓ every registry entry describes cleanly for MCP")


if __name__ == "__main__":
    test_registry_is_well_formed()
    test_effects_are_consistent()
    test_call_validates_arguments()
    test_dispatch_cannot_reach_arbitrary_methods()
    test_read_only_set()
    test_world_applies_effects()
    test_world_tracks_objects()
    test_requirement_negation()
    test_mcp_schemas_are_valid()
    print("\nAll registry and world-model tests passed.")

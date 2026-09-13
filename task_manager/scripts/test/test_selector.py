#!/usr/bin/env python3
"""
Offline tests for the brief parser and the score-aware selector.

This is the plan's primary gate: the selector is a pure function, so its decisions can
be checked without a robot. ROS-free; needs only PyYAML.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.planner.brief import (  # noqa: E402
    BriefError,
    Step,
    briefs_dir,
    load_brief,
    parse_brief,
    parse_step,
)
from task_manager.planner.manifest import Manifest  # noqa: E402
from task_manager.planner.selector import (  # noqa: E402
    choose,
    constant_travel,
    explain,
    rank,
)
from task_manager.world.world_model import WorldModel  # noqa: E402


def minimal(**overrides) -> dict:
    data = {
        "task": "unit",
        "budget_s": 420,
        "objectives": [{"id": "greet", "points": 10, "template": ["say(hello)"]}],
    }
    data.update(overrides)
    return data


# ---------------- brief parsing ----------------


def test_step_parsing() -> None:
    step = parse_step("go_to(kitchen, dinner_table)")
    assert step.skill == "go_to"
    assert step.args == {"location": "kitchen", "sublocation": "dinner_table"}

    step = parse_step("place(close_to=bowl, is_trash=true)")
    assert step.args == {"close_to": "bowl", "is_trash": True}

    step = parse_step("detect_objects()")
    assert step.args == {}

    step = parse_step('say("I am out of time, stopping.")')
    assert step.args == {"text": "I am out of time, stopping."}

    step = parse_step("place_on_shelf(2)")
    assert step.args == {"plane_height": 2}
    print("✓ step syntax parses positionals, keywords and quoted text")


def test_step_rejects_bad_input() -> None:
    for bad in ["teleport(mars)", "go_to(a, b, c)", "pick_object(x, object_name=y)"]:
        try:
            parse_step(bad)
            raise AssertionError(f"should have rejected {bad!r}")
        except BriefError:
            pass
    print("✓ unknown skills and bad arity are rejected at parse time")


def test_placeholder_binding() -> None:
    class Obj:
        name = "pringles"
        surface = "dinner_table"
        category = "snack"

    step = parse_step('pick_object("{obj.name}")')
    assert step.bind({"obj": Obj()}) == {"object_name": "pringles"}

    step = parse_step("go_to(kitchen, {obj.surface})")
    assert step.bind({"obj": Obj()})["sublocation"] == "dinner_table"
    print("✓ placeholders bind against the world model's objects")


def test_brief_validation() -> None:
    for bad, reason in [
        ({"budget_s": 1, "objectives": []}, "missing task"),
        (minimal(objectives=[]), "no objectives"),
        (minimal(risk_posture="reckless"), "bad posture"),
        (
            minimal(
                objectives=[
                    {"id": "a", "points": 1, "template": ["say(x)"]},
                    {"id": "a", "points": 1, "template": ["say(y)"]},
                ]
            ),
            "duplicate id",
        ),
        (
            minimal(triggers=[{"on": "doorbell", "objective": "nope"}]),
            "trigger targets unknown objective",
        ),
    ]:
        try:
            parse_brief(bad)
            raise AssertionError(f"should have rejected: {reason}")
        except BriefError:
            pass
    print("✓ malformed briefs are rejected with a reason")


def test_real_brief_loads() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))
    assert brief.task == "pick_and_place"
    assert brief.budget_s == 420
    ids = {objective.id for objective in brief.objectives}
    assert {"pick_place", "serve_cereal", "open_dishwasher"} <= ids, ids
    assert brief.on_deadline, "every brief must leave the arm safe"
    print(f"✓ pick_and_place brief loads ({len(brief.objectives)} objectives)")


# ---------------- selection ----------------


def build_world(objects=("pringles", "apple")) -> WorldModel:
    world = WorldModel()
    for name in objects:
        world.observe(name, category="snack", surface="dinner_table", destination="cabinet")
    world.predicates.add("objects_known")
    world.arrive("dinner_table")
    return world


def test_ev_ordering_prefers_dense_points() -> None:
    """Pouring is worth 4x a pick+place cycle per second; it must outrank it."""
    brief = parse_brief(
        minimal(
            objectives=[
                {
                    "id": "pick_place",
                    "points": 90,
                    "repeat_for": "table_objects",
                    "requires": ["arm_free"],
                    "template": ["pick_object({obj.name})", "place()"],
                },
                {
                    "id": "pour_milk",
                    "points": 400,
                    "requires": ["arm_free"],
                    "template": ["pick_object(milk)", "pour(milk, bowl)"],
                },
            ]
        )
    )
    world = build_world()
    ranked = rank(brief, world, Manifest(), 420, constant_travel())
    assert ranked[0].objective.id == "pour_milk", explain(ranked)
    print("✓ dense point clusters outrank cheap repetitive work")


def test_deadline_excludes_unfinishable_work() -> None:
    brief = parse_brief(
        minimal(
            objectives=[
                {
                    "id": "slow",
                    "points": 400,
                    "template": ["place_on_shelf(2)"],
                    "requires": ["holding"],
                },
                {"id": "quick", "points": 20, "template": ["say(done)"]},
            ]
        )
    )
    world = build_world()
    world.pick_up("pringles")

    assert any(c.objective.id == "slow" for c in rank(brief, world, Manifest(), 420))
    # with 30 s left, a 90 s shelf place is not startable
    late = rank(brief, world, Manifest(), 30)
    assert all(c.objective.id != "slow" for c in late), explain(late)
    print("✓ work that cannot finish in the remaining budget is excluded")


def test_preconditions_gate_candidates() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))
    world = build_world()

    empty_handed = {c.objective.id for c in rank(brief, world, Manifest(), 420)}
    assert "shelf_place" not in empty_handed, "cannot shelf-place with nothing held"

    world.pick_up("pringles")
    holding = {c.objective.id for c in rank(brief, world, Manifest(), 420)}
    assert "shelf_place" in holding
    assert "pick_place" not in holding, "cannot pick with a full gripper"
    print("✓ preconditions gate what is selectable")


def test_risk_posture_blocks_penalty_work() -> None:
    """Pouring can spill for -100, so a safe run needs high confidence."""
    objectives = [
        {
            "id": "pour",
            "points": 400,
            "penalty_risk": -100,
            "template": ["pour(milk, bowl)"],
            "requires": ["holding"],
        }
    ]
    shaky = Manifest({"pour": {"default": {"p": 0.5, "p50": 35, "n": 40}}})
    reliable = Manifest({"pour": {"default": {"p": 0.9, "p50": 35, "n": 40}}})

    world = build_world()
    world.pick_up("milk")

    safe = parse_brief(minimal(objectives=objectives, risk_posture="safe"))
    aggressive = parse_brief(minimal(objectives=objectives, risk_posture="aggressive"))

    assert not rank(safe, world, shaky, 420), "safe posture should refuse a coin-flip pour"
    assert rank(aggressive, world, shaky, 420), "aggressive posture should attempt it"
    assert rank(safe, world, reliable, 420), "safe posture allows a reliable pour"
    print("✓ risk posture gates objectives that can score negative")


def test_free_fallback_is_taken_when_cheaper() -> None:
    """The dishwasher-door decision from the plan, end to end."""
    brief = parse_brief(
        minimal(
            objectives=[
                {
                    "id": "open_dishwasher",
                    "points": 400,
                    "requires": ["arm_free"],
                    "fallback": "ask_help",
                    "template": ["move_arm_to(dishwasher_open)"],
                },
                {
                    "id": "ask_help",
                    "points": 0,
                    "template": ["say(please open it)"],
                },
                {
                    "id": "pick_place",
                    "points": 90,
                    "repeat_for": "table_objects",
                    "requires": ["arm_free"],
                    "template": ["pick_object({obj.name})", "place()"],
                },
            ]
        )
    )
    world = build_world()

    def candidate_for(ranked, objective_id):
        return next(c for c in ranked if c.objective.id == objective_id)

    # measured p=0.30: below the confidence bar, so the door is opened the cheap way
    unreliable = Manifest(
        {
            "move_arm_to": {"default": {"p": 0.30, "p50": 65, "n": 40}},
            "pick_object": {"default": {"p": 0.85, "p50": 30, "n": 40}},
            "place": {"default": {"p": 0.9, "p50": 15, "n": 40}},
        }
    )
    ranked = rank(brief, world, unreliable, 420, constant_travel())
    door = candidate_for(ranked, "open_dishwasher")
    assert door.use_fallback, "should defer to the free referee assist"
    assert door.points == 0, "falling back forfeits the 400-point bonus"
    # forfeiting the bonus sinks it below real scoring work, which is the point
    assert choose(brief, world, unreliable, 420, constant_travel()).objective.id == "pick_place"

    # at p=0.80 doing it ourselves clears the bar and outranks everything
    reliable = Manifest(
        {
            "move_arm_to": {"default": {"p": 0.80, "p50": 65, "n": 40}},
            "pick_object": {"default": {"p": 0.85, "p50": 30, "n": 40}},
            "place": {"default": {"p": 0.9, "p50": 15, "n": 40}},
        }
    )
    picked = choose(brief, world, reliable, 420, constant_travel())
    assert picked.objective.id == "open_dishwasher", explain(ranked)
    assert not picked.use_fallback, "should attempt it when reliable enough"
    print("✓ free fallbacks are taken exactly when doing it ourselves is worse")


def test_zero_point_enabler_is_reachable() -> None:
    """
    A 0-point assist has EV 0, so it must still be selectable once it is the only
    admissible option — otherwise a precondition could never be met.
    """
    brief = parse_brief(
        minimal(
            objectives=[
                {
                    "id": "ask_help",
                    "points": 0,
                    "template": ["say(please open the dishwasher)"],
                },
            ]
        )
    )
    world = build_world()
    picked = choose(brief, world, Manifest(), 420, constant_travel())
    assert picked is not None and picked.objective.id == "ask_help"
    assert picked.ev == 0.0
    print("✓ zero-point enablers are still reachable when nothing else is")


def test_travel_cost_batches_nearby_work() -> None:
    """Including travel is what kills the measured 465 s of aggregate navigation."""
    brief = parse_brief(
        minimal(
            objectives=[
                {
                    "id": "here",
                    "points": 90,
                    "at": "dinner_table",
                    "template": ["pick_object(apple)"],
                    "requires": ["arm_free"],
                },
                {
                    "id": "far",
                    "points": 95,
                    "at": "bedroom",
                    "template": ["pick_object(apple)"],
                    "requires": ["arm_free"],
                },
            ]
        )
    )
    world = build_world()
    world.arrive("dinner_table")
    ranked = rank(brief, world, Manifest(), 420, constant_travel(60.0))
    assert ranked[0].objective.id == "here", explain(ranked)
    assert ranked[0].travel_s == 0.0
    print("✓ travel cost keeps the robot from ping-ponging across the arena")


def test_once_and_progress_are_respected() -> None:
    once_only = parse_brief(
        minimal(
            objectives=[
                {"id": "greet", "points": 10, "once": True, "template": ["say(hello)"]},
            ]
        )
    )
    world = build_world()
    assert rank(once_only, world, Manifest(), 420)
    world.mark_done("greet")
    assert rank(once_only, world, Manifest(), 420) == [], "a once objective must retire"

    # a non-repeating objective also gives up after max_attempts
    retryable = parse_brief(
        minimal(
            objectives=[
                {"id": "tidy", "points": 10, "max_attempts": 2, "template": ["say(ok)"]},
            ]
        )
    )
    fresh = build_world()
    assert rank(retryable, fresh, Manifest(), 420)
    fresh.mark_done("tidy")
    assert rank(retryable, fresh, Manifest(), 420), "one attempt used, one left"
    fresh.mark_done("tidy")
    assert rank(retryable, fresh, Manifest(), 420) == [], "attempts exhausted"
    print("✓ once-only objectives retire, retries are capped")


def test_selector_returns_none_when_nothing_fits() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))
    world = build_world()
    assert choose(brief, world, Manifest(), 1.0, constant_travel()) is None
    print("✓ selector reports 'nothing worth doing' instead of guessing")


if __name__ == "__main__":
    test_step_parsing()
    test_step_rejects_bad_input()
    test_placeholder_binding()
    test_brief_validation()
    test_real_brief_loads()
    test_ev_ordering_prefers_dense_points()
    test_deadline_excludes_unfinishable_work()
    test_preconditions_gate_candidates()
    test_risk_posture_blocks_penalty_work()
    test_free_fallback_is_taken_when_cheaper()
    test_zero_point_enabler_is_reachable()
    test_travel_cost_batches_nearby_work()
    test_once_and_progress_are_respected()
    test_selector_returns_none_when_nothing_fits()
    print("\nAll selector tests passed.")
    _ = Step

#!/usr/bin/env python3
"""
Offline tests for event-driven briefs (HRIC, Restaurant) and every shipped brief.

Trigger handling is tested against the same logic BriefRunner uses, without ROS.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.planner.brief import briefs_dir, load_brief, parse_brief  # noqa: E402
from task_manager.planner.manifest import Manifest  # noqa: E402
from task_manager.planner.selector import constant_travel, rank  # noqa: E402
from task_manager.skills.registry import REGISTRY  # noqa: E402
from task_manager.world.world_model import WorldModel  # noqa: E402

SHIPPED = ["pick_and_place", "doing_laundry", "hric", "restaurant"]


class TriggerQueue:
    """Mirrors BriefRunner's trigger bookkeeping without needing a ROS node."""

    def __init__(self, brief, world):
        self.brief = brief
        self.world = world
        self.fired = []

    def fire(self, event: str) -> bool:
        if event not in {trigger.on for trigger in self.brief.triggers}:
            return False
        if event not in self.fired:
            self.fired.append(event)
        return True

    def next_objective(self):
        for event in list(self.fired):
            for trigger in self.brief.triggers:
                if trigger.on != event or not self.world.satisfies(trigger.when):
                    continue
                objective = self.brief.objective(trigger.objective)
                if objective is None:
                    continue
                if objective.once and self.world.times_done(objective.id) > 0:
                    self.fired.remove(event)
                    break
                return event, objective
        return None, None


def test_every_shipped_brief_is_valid() -> None:
    for name in SHIPPED:
        brief = load_brief(os.path.join(briefs_dir(), f"{name}.yaml"))
        assert brief.objectives, f"{name} has no objectives"
        assert brief.on_deadline, f"{name} must leave the arm safe when time runs out"
        for objective in brief.objectives:
            for step in objective.template:
                assert step.skill in REGISTRY, f"{name}.{objective.id} uses {step.skill}"
        print(f"  {name:16} {len(brief.objectives):2} objectives, {len(brief.triggers)} triggers")
    print("✓ every shipped brief parses and only uses registered skills")


def test_budgets_match_the_rulebook() -> None:
    expected = {
        "pick_and_place": 420,  # 7:00
        "doing_laundry": 420,  # 7:00
        "hric": 420,  # 7:00
        "restaurant": 900,  # 15:00
    }
    for name, budget in expected.items():
        brief = load_brief(os.path.join(briefs_dir(), f"{name}.yaml"))
        assert brief.budget_s == budget, f"{name}: {brief.budget_s} != {budget}"
    print("✓ brief budgets match the rulebook time limits")


def test_trigger_fires_and_is_consumed() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "hric.yaml"))
    world = WorldModel()
    world.predicates.add("at_start_position")
    queue = TriggerQueue(brief, world)

    assert queue.next_objective() == (None, None), "nothing pending before an event"

    assert queue.fire("doorbell")
    event, objective = queue.next_objective()
    assert objective is not None and objective.id == "receive_guest", objective
    queue.fired.remove(event)
    assert queue.next_objective() == (None, None), "a handled trigger must not repeat"
    print("✓ a fired trigger yields its objective once")


def test_unknown_event_is_rejected() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "hric.yaml"))
    queue = TriggerQueue(brief, WorldModel())
    assert not queue.fire("smoke_alarm"), "undeclared events must not queue"
    print("✓ undeclared events are rejected")


def test_trigger_conditions_are_respected() -> None:
    """The doorbell only counts while the robot waits at the start position."""
    brief = load_brief(os.path.join(briefs_dir(), "hric.yaml"))
    world = WorldModel()
    queue = TriggerQueue(brief, world)
    queue.fire("doorbell")

    assert queue.next_objective() == (None, None), "not at the start position yet"
    world.predicates.add("at_start_position")
    _, objective = queue.next_objective()
    assert objective is not None and objective.id == "receive_guest"
    print("✓ trigger conditions gate when an event is honoured")


def test_triggered_objective_outranks_scoring_work() -> None:
    """
    A waving customer is an obligation, not an option.

    Expected value alone would keep collecting items, because a person is worth fewer
    points per second than a pick. BriefRunner answers the trigger first; this checks
    the ranking really would have chosen otherwise, so the override is doing work.
    """
    brief = load_brief(os.path.join(briefs_dir(), "restaurant.yaml"))
    world = WorldModel()
    for name in ("coca_cola", "pringles"):
        world.observe(name, surface="bar", destination="table")
    world.arrive("kitchen_bar")

    # make spotting a customer slow and unreliable so pure EV would rather keep picking
    grudging = Manifest(
        {
            "get_customer": {"default": {"p": 0.35, "p50": 60, "n": 40}},
            "say": {"default": {"p": 0.99, "p50": 4, "n": 40}},
            "pick_object": {"default": {"p": 0.9, "p50": 20, "n": 40}},
            "go_to": {"default": {"p": 0.95, "p50": 10, "n": 40}},
        }
    )
    ranked = rank(brief, world, grudging, 900, constant_travel())
    assert ranked, "expected some candidates"
    assert (
        ranked[0].objective.id != "approach_customer"
    ), f"EV should have preferred routine work here, got {ranked[0].objective.id}"

    # the trigger overrides that ranking: a waving customer is an obligation
    queue = TriggerQueue(brief, world)
    queue.fire("customer_calling")
    _, objective = queue.next_objective()
    assert objective.id == "approach_customer"
    print("✓ triggered work pre-empts higher-scoring routine work")


def test_laundry_protects_the_fold() -> None:
    """Folding is 800 points, the densest objective in that test; it must rank first."""
    brief = load_brief(os.path.join(briefs_dir(), "doing_laundry.yaml"))
    world = WorldModel()
    world.observe("shirt", surface="laundry_table", destination="laundry_table")
    world.arrive("laundry_table")

    reliable = Manifest({skill: {"default": {"p": 0.9, "p50": 10, "n": 40}} for skill in REGISTRY})
    ranked = rank(brief, world, reliable, 420, constant_travel())
    assert ranked[0].objective.id == "fold_shirt", [c.objective.id for c in ranked[:3]]
    print("✓ laundry prioritises the 800-point fold")


def test_penalty_objectives_are_marked() -> None:
    """Anything the scoresheet can score negative must declare penalty_risk."""
    risky = {
        "doing_laundry": {"fold_shirt", "deliver_to_table"},
        "restaurant": {"serve_item"},
        "hric": {"deliver_bag", "describe_first_guest"},
    }
    for name, expected in risky.items():
        brief = load_brief(os.path.join(briefs_dir(), f"{name}.yaml"))
        declared = {o.id for o in brief.objectives if o.penalty_risk}
        assert expected <= declared, f"{name}: {expected - declared} missing penalty_risk"
    print("✓ objectives that can score negative declare it")


if __name__ == "__main__":
    test_every_shipped_brief_is_valid()
    test_budgets_match_the_rulebook()
    test_trigger_fires_and_is_consumed()
    test_unknown_event_is_rejected()
    test_trigger_conditions_are_respected()
    test_triggered_objective_outranks_scoring_work()
    test_laundry_protects_the_fold()
    test_penalty_objectives_are_marked()
    print("\nAll trigger and brief tests passed.")
    _ = parse_brief

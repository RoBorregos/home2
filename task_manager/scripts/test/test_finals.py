#!/usr/bin/env python3
"""
Offline tests for the Finals brief and the repetition scoring it depends on.

Finals is the one task with no fixed sequence, so the thing worth testing is that the
selector spreads its work across problem categories instead of farming the easiest one.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from task_manager.planner.brief import briefs_dir, load_brief, parse_brief  # noqa: E402
from task_manager.planner.manifest import Manifest  # noqa: E402
from task_manager.planner.selector import choose, constant_travel, rank  # noqa: E402
from task_manager.skills.registry import REGISTRY  # noqa: E402
from task_manager.world.world_model import WorldModel  # noqa: E402


def finals():
    return load_brief(os.path.join(briefs_dir(), "finals.yaml"))


def reliable() -> Manifest:
    return Manifest({skill: {"default": {"p": 0.9, "p50": 10, "n": 40}} for skill in REGISTRY})


def world_with(objects=("banana_peel", "cup")) -> WorldModel:
    world = WorldModel()
    for name in objects:
        world.observe(name, surface="living_room", destination="kitchen")
    world.predicates.add("objects_known")
    world.facts["rooms"] = ["living_room", "kitchen", "bedroom"]
    world.arrive("living_room")
    return world


def test_finals_brief_is_valid() -> None:
    brief = finals()
    assert brief.budget_s == 600, brief.budget_s
    assert brief.risk_posture == "aggressive"
    assert brief.repeat_penalties == (300.0, 500.0), brief.repeat_penalties
    for objective in brief.objectives:
        for step in objective.template:
            assert step.skill in REGISTRY, f"{objective.id} uses unknown {step.skill}"
    categories = {o.category for o in brief.objectives if o.category}
    assert len(categories) >= 5, f"finals needs variety, got {categories}"
    print(
        f"✓ finals brief valid ({len(brief.objectives)} objectives, {len(categories)} categories)"
    )


def test_repeat_penalty_reduces_points() -> None:
    brief = finals()
    world = world_with()

    def points_for(objective_id):
        return next(
            c.points
            for c in rank(brief, world, reliable(), 600, constant_travel())
            if c.objective.id == objective_id
        )

    assert points_for("floor_trash") == 650
    world.solve_category("trash")
    assert points_for("floor_trash") == 350, "second solve is docked 300"
    world.solve_category("trash")
    assert points_for("floor_trash") == 150, "third solve is docked 500"
    world.solve_category("trash")
    assert points_for("floor_trash") == 150, "stays at the third-onward rate"
    print("✓ repeated problem categories are worth progressively less")


def test_selector_spreads_across_categories() -> None:
    """After solving trash once, a different category should outrank more trash."""
    brief = finals()
    world = world_with(("banana_peel", "wrapper", "cup"))

    first = choose(brief, world, reliable(), 600, constant_travel())
    assert first is not None

    world.solve_category("trash")
    world.solve_category("trash")

    after = rank(brief, world, reliable(), 600, constant_travel())
    assert after, "expected candidates to remain"
    assert (
        after[0].objective.category != "trash"
    ), f"should have moved on from trash, picked {after[0].objective.id}"
    print("✓ the selector moves on instead of farming one category")


def test_no_penalties_without_declaration() -> None:
    """Briefs that do not declare repeat_penalties must be unaffected."""
    brief = parse_brief(
        {
            "task": "unit",
            "budget_s": 420,
            "objectives": [
                {"id": "tidy", "points": 100, "category": "trash", "template": ["say(ok)"]}
            ],
        }
    )
    world = WorldModel()
    world.solve_category("trash")
    world.solve_category("trash")
    candidate = rank(brief, world, reliable(), 420, constant_travel())[0]
    assert candidate.points == 100, candidate.points
    print("✓ repetition scoring only applies where the brief declares it")


def test_explicit_problems_are_priced_from_the_scoresheet() -> None:
    brief = finals()
    expected = {
        "close_dishwasher": 300,
        "move_laundry_basket": 600,
        "welcome_guest": 600,
    }
    for objective_id, points in expected.items():
        objective = brief.objective(objective_id)
        assert objective is not None, objective_id
        assert objective.points == points, f"{objective_id}: {objective.points} != {points}"
        assert objective.once, f"{objective_id} can only be solved once"
    print("✓ the three named finals problems carry their scoresheet values")


def test_people_are_reachable_by_trigger() -> None:
    brief = finals()
    events = {trigger.on for trigger in brief.triggers}
    assert "person_raised_hand" in events, events
    target = {t.objective for t in brief.triggers if t.on == "person_raised_hand"}
    assert target == {"help_person"}, target
    print("✓ a person raising their hand has a trigger path")


if __name__ == "__main__":
    test_finals_brief_is_valid()
    test_repeat_penalty_reduces_points()
    test_selector_spreads_across_categories()
    test_no_penalties_without_declaration()
    test_explicit_problems_are_priced_from_the_scoresheet()
    test_people_are_reachable_by_trigger()
    print("\nAll finals tests passed.")

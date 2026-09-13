#!/usr/bin/env python3
"""
Replay past runs through the selector and compare its choices to what the FSM did.

This is the gate before any of this touches the robot: if the selector's ordering does
not beat the recorded ordering on the same runs, the idea is wrong and no amount of
on-robot debugging will save it.

Scoring here is a projection, not a referee's sheet: each objective's declared points
times its measured success probability. It is meant for comparing two orderings under
the same assumptions, not for predicting an absolute competition score.

Usage:
    python3 replay_selector.py --brief pick_and_place [--runs ~/frida_runs]
    python3 replay_selector.py --brief pick_and_place --posture aggressive --verbose
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fit_capabilities import build_manifest, iter_skill_rows  # noqa: E402

from task_manager.planner.brief import Brief, briefs_dir, load_brief  # noqa: E402
from task_manager.planner.manifest import Manifest  # noqa: E402
from task_manager.planner.selector import (  # noqa: E402
    as_dict,
    choose,
    constant_travel,
)
from task_manager.world.world_model import WorldModel  # noqa: E402

SUCCESS = "EXECUTION_SUCCESS"
# objects the pick-and-place arena puts on the table, used when a run log has no
# detection rows to seed from
DEFAULT_OBJECTS = ("blue_cereal_box", "pringles", "apple", "spoon", "red_plate", "coca_cola")


def seed_world(rows: list) -> WorldModel:
    """Build a starting world from whatever the run actually perceived."""
    world = WorldModel()
    seen = []
    for row in rows:
        name = row.get("context", {}).get("object")
        if name and name not in seen:
            seen.append(name)
    for name in seen or DEFAULT_OBJECTS:
        world.observe(name, surface="dinner_table", destination="cabinet")
    world.predicates.add("objects_known")
    world.arrive("dinner_table")
    return world


def simulate(brief: Brief, manifest: Manifest, world: WorldModel, verbose: bool = False) -> dict:
    """
    Run the selector to exhaustion against the measured manifest.

    Deterministic: an objective is assumed to take its expected time and to earn its
    points weighted by measured probability. No sampling, so two runs compare cleanly.
    """
    remaining = brief.budget_s
    travel = constant_travel()
    projected = 0.0
    order = []

    while remaining > 0:
        candidate = choose(brief, world, manifest, remaining, travel)
        if candidate is None:
            break

        remaining -= candidate.total_s
        projected += candidate.p * candidate.points
        order.append(as_dict(candidate))
        if verbose:
            print(
                f"  {brief.budget_s - remaining:6.0f}s  {candidate.label:28} "
                f"ev={candidate.ev:5.2f}  +{candidate.p * candidate.points:6.1f}"
            )

        _apply(candidate, world)

    return {
        "projected_points": round(projected, 1),
        "seconds_used": round(brief.budget_s - remaining, 1),
        "objectives": order,
    }


def _apply(candidate, world: WorldModel) -> None:
    """Advance the world as if the chosen objective completed."""
    objective = candidate.objective
    world.mark_done(objective.id)

    target = candidate.binding.get("obj")
    if target is not None:
        target.placed = True
        target.picked = True

    # mirror the template's declared effects so preconditions stay honest
    from task_manager.skills.registry import REGISTRY

    for step in objective.template:
        skill = REGISTRY.get(step.skill)
        if skill is None:
            continue
        world.predicates.update(skill.sets)
        world.predicates.difference_update(skill.clears)

    if objective.at:
        world.arrive(objective.at)


def _score_one_run(rows: list, budget_s: float) -> dict:
    """
    Score a single recorded run, counting only work that finished inside the budget.

    The FSM never looked at the clock, so a recorded run routinely overruns; crediting
    points it scored after the buzzer would make the baseline unbeatable and meaningless.
    """
    picks = places = pours = 0
    attempted = {"pick": 0, "place": 0, "pour": 0}
    elapsed = 0.0

    for row in rows:
        elapsed += row["duration_s"]
        if elapsed > budget_s:
            break
        skill, ok = row["skill"], row["status"] == SUCCESS
        if skill == "pick_object":
            attempted["pick"] += 1
            picks += ok
        elif skill in {"place", "place_on_shelf", "place_on_floor"}:
            attempted["place"] += 1
            places += ok
        elif skill == "pour":
            attempted["pour"] += 1
            pours += ok

    # scoresheet values: 50 per pick, 40 per place, 200 per pour, +100 first pick
    points = picks * 50 + places * 40 + pours * 200 + (100 if picks else 0)
    return {
        "points": points,
        "picks": f"{picks}/{attempted['pick']}",
        "places": f"{places}/{attempted['place']}",
        "pours": f"{pours}/{attempted['pour']}",
        "seconds": round(min(elapsed, budget_s), 1),
        "overran": elapsed > budget_s,
    }


def fsm_baseline(rows: list, budget_s: float) -> dict:
    """Best single recorded run — each task runs on three days and only the best counts."""
    by_run: dict = {}
    for row in rows:
        by_run.setdefault(row.get("run_id", "unknown"), []).append(row)

    scored = [_score_one_run(run_rows, budget_s) for run_rows in by_run.values()]
    best = max(scored, key=lambda entry: entry["points"])
    best["runs"] = len(scored)
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--brief", default="pick_and_place", help="brief name or path")
    parser.add_argument("--runs", default=os.path.expanduser("~/frida_runs"))
    parser.add_argument("--posture", default="", choices=["", "safe", "aggressive"])
    parser.add_argument("--verbose", action="store_true", help="print each choice")
    args = parser.parse_args()

    path = args.brief
    if not os.path.exists(path):
        path = os.path.join(briefs_dir(), f"{args.brief}.yaml")
    brief = load_brief(path)
    if args.posture:
        brief = Brief(
            task=brief.task,
            budget_s=brief.budget_s,
            objectives=brief.objectives,
            risk_posture=args.posture,
            triggers=brief.triggers,
            on_start=brief.on_start,
            on_deadline=brief.on_deadline,
        )

    rows = list(iter_skill_rows(args.runs))
    if not rows:
        print(f"No run logs in {args.runs}; using registry priors only.")
    manifest = Manifest(build_manifest(rows, "object")) if rows else Manifest()

    print(f"Brief: {brief.task}  budget={brief.budget_s:.0f}s  posture={brief.risk_posture}")
    print(f"Runs:  {len(rows)} skill calls from {args.runs}\n")

    if args.verbose:
        print("Selector plan:")
    result = simulate(brief, manifest, seed_world(rows), verbose=args.verbose)

    print(
        f"\nSelector projection: {result['projected_points']:.0f} pts "
        f"in {result['seconds_used']:.0f}s over {len(result['objectives'])} objectives"
    )

    if rows:
        baseline = fsm_baseline(rows, brief.budget_s)
        overran = " (run overran the budget; later work not counted)" if baseline["overran"] else ""
        print(
            f"Best recorded run:   {baseline['points']} pts "
            f"in {baseline['seconds']}s of {len(rows)} calls across {baseline['runs']} run(s)  "
            f"(picks {baseline['picks']}, places {baseline['places']}, pours {baseline['pours']})"
            f"{overran}"
        )
        verdict = "BEATS" if result["projected_points"] > baseline["points"] else "does NOT beat"
        print(f"\nSelector {verdict} the best recorded ordering.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

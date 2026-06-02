"""Test-only assertions for merger plans.

These helpers used to live in ``task_manager.gpsr.merger`` but were only
referenced by ``test_hri_manager.py``, so they were moved here to keep the
production module focused on planning.
"""

from typing import Any, Iterable, Sequence

from task_manager.gpsr.merger import InterleavedPlan, PlanAction


def gripper_invariant_holds(plan: Iterable[PlanAction]) -> bool:
    held = False
    for pa in plan:
        if pa.requires_gripper:
            if held:
                return False
            held = True
        if pa.releases_gripper:
            if not held:
                return False
            held = False
    return True


def per_command_order_preserved(plan: Iterable[PlanAction], n_cmds: int) -> bool:
    last_seen = [-1] * n_cmds
    for pa in plan:
        if pa.source_idx < last_seen[pa.source_cmd]:
            return False
        last_seen[pa.source_cmd] = pa.source_idx
    return True


def non_goto_actions_preserved(plan: InterleavedPlan, commands: Sequence[Any]) -> bool:
    """Every non-``go_to`` action of every command must survive into the merged
    plan. ``go_to`` actions are exempt because the collapse pass legitimately
    dedups redundant back-to-back navigation. This is the core "no command (or
    part of one) is silently dropped during merge" invariant — a leading
    ``go_to`` may be collapsed when the robot is already there, but the actual
    work of every command must always be scheduled.
    """
    for cmd_idx, command in enumerate(commands):
        actions = list(getattr(command, "commands", []))
        want = {i for i, a in enumerate(actions) if getattr(a, "action", "") != "go_to"}
        got = {
            pa.source_idx
            for pa in plan.actions
            if pa.source_cmd == cmd_idx and getattr(pa.action, "action", "") != "go_to"
        }
        if want != got:
            return False
    return True


def fallback_preserves_all(plan: InterleavedPlan, commands: Sequence[Any]) -> bool:
    """Each per-command fallback list reproduces that command's actions in full,
    in original order (modulo redundant-go_to collapse within the command)."""
    if len(plan.fallback) != len(commands):
        return False
    for cmd_idx, command in enumerate(commands):
        actions = list(getattr(command, "commands", []))
        want = {i for i, a in enumerate(actions) if getattr(a, "action", "") != "go_to"}
        got = {
            pa.source_idx
            for pa in plan.fallback[cmd_idx]
            if getattr(pa.action, "action", "") != "go_to"
        }
        if want != got:
            return False
    return True

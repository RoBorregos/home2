"""Test-only assertions for merger plans.

These helpers used to live in ``task_manager.gpsr.merger`` but were only
referenced by ``test_hri_manager.py``, so they were moved here to keep the
production module focused on planning.
"""

from typing import Iterable

from task_manager.gpsr.merger import PlanAction


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

"""Test-only assertions for merger plans.

These helpers used to live in ``task_manager.gpsr.merger`` but were only
referenced by ``test_hri_manager.py``, so they were moved here to keep the
production module focused on planning.
"""

from types import SimpleNamespace
from typing import Any, Iterable, List, Sequence, Tuple

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


def gripper_walk_strict(plan_actions: Iterable[PlanAction]) -> bool:
    """Stricter than ``gripper_invariant_holds``: also asserts a pick is released
    by the *same* command that acquired it, so no other command's gripper action
    crosses an open pick->drop block."""
    held_by = None
    for pa in plan_actions:
        if pa.requires_gripper:
            if held_by is not None:
                return False
            held_by = pa.source_cmd
        if pa.releases_gripper:
            if held_by is None or held_by != pa.source_cmd:
                return False
            held_by = None
    return True


def build_commands(commands_json: Sequence[Sequence[dict]]) -> List[Any]:
    """Turn the JSON command spec into BAML-like stand-ins the merger accepts.

    Each command is a list of action dicts; the ``action`` key holds the kind
    and every other key becomes an attribute (location_to_go, object_to_pick,
    name, attribute_value, info_type, previous_command_info, ...).
    """
    commands = []
    for actions in commands_json:
        objs = []
        for raw in actions:
            fields = dict(raw)
            kind = fields.pop("action")
            objs.append(SimpleNamespace(action=kind, **fields))
        commands.append(SimpleNamespace(commands=objs))
    return commands


def make_locator(coords: dict):
    """Build a locator from a ``{name: [x, y]}`` map; unknown names resolve to
    ``None`` (i.e. an unresolved/unknown location)."""
    table = {k: tuple(v) for k, v in coords.items()}
    return lambda name: table.get(name)


def _go_tos(plan: InterleavedPlan) -> List[PlanAction]:
    return [pa for pa in plan.actions if getattr(pa.action, "action", "") == "go_to"]


def _label(action: Any) -> str:
    kind = getattr(action, "action", "?")
    for arg in (
        "location_to_go",
        "object_to_pick",
        "name",
        "attribute_value",
        "destination",
        "destination_room",
        "info_type",
        "target_to_count",
        "object_category",
    ):
        v = getattr(action, arg, None)
        if v:
            return f"{kind}({v})"
    return kind


def describe_plan(plan: InterleavedPlan) -> str:
    if not plan.actions:
        return "(empty)"
    return " -> ".join(
        f"[c{pa.source_cmd}.{pa.source_idx}] {_label(pa.action)}" for pa in plan.actions
    )


def evaluate_expectations(
    plan: InterleavedPlan, commands: Sequence[Any], expect: dict
) -> Tuple[bool, List[str]]:
    """Run the declarative checks in ``expect`` against a merged ``plan``.

    Returns ``(ok, failures)`` where ``failures`` names each check that did not
    hold. Every key is optional so a scenario only asserts what it cares about.
    """
    n_cmds = len(commands)
    A = plan.actions
    failures: List[str] = []

    def fail(msg: str) -> None:
        failures.append(msg)

    if expect.get("empty_plan") and not (A == [] and plan.fallback == []):
        fail("empty_plan")

    if "plan_size" in expect and len(A) != expect["plan_size"]:
        fail(f"plan_size: got {len(A)} want {expect['plan_size']}")

    if "plan_kinds" in expect:
        got = [pa.action.action for pa in A]
        if got != expect["plan_kinds"]:
            fail(f"plan_kinds: got {got}")

    if "source_idx_order" in expect:
        got = [pa.source_idx for pa in A]
        if got != expect["source_idx_order"]:
            fail(f"source_idx_order: got {got}")

    if "source_cmd_order" in expect:
        got = [pa.source_cmd for pa in A]
        if got != expect["source_cmd_order"]:
            fail(f"source_cmd_order: got {got}")

    if "all_source_cmd" in expect and not all(
        pa.source_cmd == expect["all_source_cmd"] for pa in A
    ):
        fail("all_source_cmd")

    if "go_to_order" in expect:
        got = [pa.action.location_to_go for pa in _go_tos(plan)]
        if got != expect["go_to_order"]:
            fail(f"go_to_order: got {got}")

    if "go_to_source_cmds" in expect:
        got = [pa.source_cmd for pa in _go_tos(plan)]
        if got != expect["go_to_source_cmds"]:
            fail(f"go_to_source_cmds: got {got}")

    if "first_go_to_location" in expect:
        g = _go_tos(plan)
        if not g or g[0].action.location_to_go != expect["first_go_to_location"]:
            fail("first_go_to_location")

    if "first_go_to_source_cmd" in expect:
        g = _go_tos(plan)
        if not g or g[0].source_cmd != expect["first_go_to_source_cmd"]:
            fail("first_go_to_source_cmd")

    for loc, want in expect.get("location_visit_counts", {}).items():
        got = sum(1 for pa in _go_tos(plan) if pa.action.location_to_go == loc)
        if got != want:
            fail(f"location_visit_counts[{loc}]: got {got} want {want}")

    for loc, cmd in expect.get("go_to_location_source_cmd", {}).items():
        matches = [pa for pa in _go_tos(plan) if pa.action.location_to_go == loc]
        if not matches or any(pa.source_cmd != cmd for pa in matches):
            fail(f"go_to_location_source_cmd[{loc}] != {cmd}")

    for kind, want in expect.get("action_count", {}).items():
        got = sum(1 for pa in A if pa.action.action == kind)
        if got != want:
            fail(f"action_count[{kind}]: got {got} want {want}")

    for kind, cmds in expect.get("action_source_cmds", {}).items():
        got = sorted({pa.source_cmd for pa in A if pa.action.action == kind})
        if got != sorted(cmds):
            fail(f"action_source_cmds[{kind}]: got {got}")

    if "kinds_before_location" in expect:
        spec = expect["kinds_before_location"]
        idx = next(
            (
                i
                for i, pa in enumerate(A)
                if pa.action.action == "go_to" and pa.action.location_to_go == spec["location"]
            ),
            None,
        )
        if idx is None:
            fail(f"kinds_before_location: no go_to {spec['location']}")
        else:
            before = [pa for pa in A[:idx] if pa.action.action == spec["kind"]]
            if "count" in spec and len(before) != spec["count"]:
                fail(f"kinds_before_location count: got {len(before)}")
            if "source_cmds" in spec and sorted({pa.source_cmd for pa in before}) != sorted(
                spec["source_cmds"]
            ):
                fail("kinds_before_location source_cmds")

    for spec in expect.get("precedence_per_cmd", []):
        ca = [pa for pa in A if pa.source_cmd == spec["cmd"]]
        bi = next((i for i, pa in enumerate(ca) if pa.action.action == spec["before"]), None)
        ai = next((i for i, pa in enumerate(ca) if pa.action.action == spec["after"]), None)
        if bi is None or ai is None or bi >= ai:
            fail(f"precedence_per_cmd c{spec['cmd']}: {spec['before']}<{spec['after']}")

    if "contiguous_source_cmd" in expect:
        pos = [i for i, pa in enumerate(A) if pa.source_cmd == expect["contiguous_source_cmd"]]
        if pos and pos != list(range(min(pos), max(pos) + 1)):
            fail("contiguous_source_cmd")

    for cmd, idx in expect.get("contains_source", []):
        if (cmd, idx) not in {(pa.source_cmd, pa.source_idx) for pa in A}:
            fail(f"contains_source ({cmd},{idx})")

    if expect.get("gripper_invariant") and not gripper_invariant_holds(A):
        fail("gripper_invariant")
    if expect.get("gripper_walk_strict") and not gripper_walk_strict(A):
        fail("gripper_walk_strict")
    if expect.get("per_command_order") and not per_command_order_preserved(A, n_cmds):
        fail("per_command_order")
    if expect.get("non_goto_preserved") and not non_goto_actions_preserved(plan, commands):
        fail("non_goto_preserved")
    if expect.get("fallback_preserves_all") and not fallback_preserves_all(plan, commands):
        fail("fallback_preserves_all")

    if "fallback_size" in expect and len(plan.fallback) != expect["fallback_size"]:
        fail(f"fallback_size: got {len(plan.fallback)}")
    if "fallback_source_idx" in expect:
        got = [[pa.source_idx for pa in fb] for fb in plan.fallback]
        if got != expect["fallback_source_idx"]:
            fail(f"fallback_source_idx: got {got}")
    if "fallback_first_kind" in expect:
        got = [fb[0].action.action if fb else None for fb in plan.fallback]
        if got != expect["fallback_first_kind"]:
            fail(f"fallback_first_kind: got {got}")

    return (len(failures) == 0, failures)

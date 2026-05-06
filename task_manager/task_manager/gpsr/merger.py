"""GPSR multi-command interleaving merger.

Single-item, drop-before-pick model. Takes a list of parsed BAML
``CommandListLLM`` objects (one per spoken utterance) and returns a single
``InterleavedPlan`` whose ``actions`` list executes the commands in an
order that:

  * preserves intra-command sequencing,
  * preserves precedence edges across atomic gripper blocks,
  * never holds more than one object at a time,
  * minimizes 2-D travel between locations using greedy nearest-neighbour
    plus a precedence-safe 2-opt pass.

The merger is intentionally framework-free: it talks to BAML objects via
``getattr`` so unit tests can pass simple stand-ins.
"""

from dataclasses import dataclass
from typing import Any, Callable, Iterable, List, Optional, Sequence, Tuple

# Action-kind classifications.
_GRIPPER_ACQUIRES = frozenset({"pick_object"})
_GRIPPER_RELEASES = frozenset({"give_object", "place_object"})


@dataclass
class PlanAction:
    """An action with its provenance + gripper effect."""

    action: Any
    source_cmd: int
    source_idx: int
    location: Optional[str]
    requires_gripper: bool
    releases_gripper: bool


@dataclass
class InterleavedPlan:
    actions: List[PlanAction]
    fallback: List[List[PlanAction]]


Locator = Callable[[str], Optional[Tuple[float, float]]]


@dataclass
class _Segment:
    cmd: int
    indices: List[int]  # action indices within the source command
    location: Optional[str]  # the navigation target (from a leading go_to)
    acquires: bool
    releases: bool


def _kind(action: Any) -> str:
    return getattr(action, "action", "") or ""


def _decompose(cmd_idx: int, command: Any) -> List[_Segment]:
    """Split a command's action list into go_to-led segments."""
    actions = list(getattr(command, "commands", []))
    segments: List[_Segment] = []
    cur_loc: Optional[str] = None
    cur_indices: List[int] = []
    cur_acq = False
    cur_rel = False

    def flush() -> None:
        nonlocal cur_indices, cur_acq, cur_rel, cur_loc
        if cur_indices:
            segments.append(
                _Segment(
                    cmd=cmd_idx,
                    indices=list(cur_indices),
                    location=cur_loc,
                    acquires=cur_acq,
                    releases=cur_rel,
                )
            )
        cur_indices = []
        cur_acq = False
        cur_rel = False

    for i, action in enumerate(actions):
        kind = _kind(action)
        if kind == "go_to":
            flush()
            cur_loc = getattr(action, "location_to_go", None)
            cur_indices.append(i)
        else:
            cur_indices.append(i)
            if kind in _GRIPPER_ACQUIRES:
                cur_acq = True
            if kind in _GRIPPER_RELEASES:
                cur_rel = True
    flush()
    return segments


def _build_units(segments: Sequence[_Segment]) -> List[List[int]]:
    """Group consecutive segments of one command into atomic units.

    A unit is either a single free segment (gripper-neutral) or a contiguous
    pick → … → give|place block.  Units are ordered as they appear in the
    source command, so per-command precedence is preserved by construction.
    Segment indices are global into ``segments``.
    """
    units: List[List[int]] = []
    in_block = False
    cur_block: List[int] = []
    for gi, seg in enumerate(segments):
        if not in_block:
            if seg.acquires and seg.releases:
                units.append([gi])  # pick + drop in one segment
            elif seg.acquires:
                in_block = True
                cur_block = [gi]
            else:
                units.append([gi])
        else:
            cur_block.append(gi)
            if seg.releases:
                units.append(list(cur_block))
                in_block = False
                cur_block = []
    if cur_block:
        # Unmatched pick (parser anomaly): treat trailing tail as one unit
        # to preserve correctness — gripper invariant won't be enforceable
        # but we don't lose actions.
        units.append(list(cur_block))
    return units


def _euclidean(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> float:
    if a is None or b is None:
        return 0.0
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5


def _unit_first_xy(
    unit: Sequence[int], segments: Sequence[_Segment], locator: Locator
) -> Optional[Tuple[float, float]]:
    for gi in unit:
        loc = segments[gi].location
        if loc:
            xy = locator(loc)
            if xy is not None:
                return xy
    return None


def _unit_last_xy(
    unit: Sequence[int], segments: Sequence[_Segment], locator: Locator
) -> Optional[Tuple[float, float]]:
    last: Optional[Tuple[float, float]] = None
    for gi in unit:
        loc = segments[gi].location
        if loc:
            xy = locator(loc)
            if xy is not None:
                last = xy
    return last


def _greedy_schedule(
    units: Sequence[Sequence[int]],
    unit_cmd: Sequence[int],
    unit_pos: Sequence[int],
    n_cmds: int,
    segments: Sequence[_Segment],
    locator: Locator,
) -> List[int]:
    n = len(units)
    next_pos = [0] * n_cmds
    scheduled: List[int] = []
    cur_xy: Tuple[float, float] = (0.0, 0.0)
    while len(scheduled) < n:
        eligible = [
            ui for ui in range(n) if ui not in scheduled and unit_pos[ui] == next_pos[unit_cmd[ui]]
        ]
        if not eligible:
            break
        eligible.sort(
            key=lambda ui: (
                _euclidean(cur_xy, _unit_first_xy(units[ui], segments, locator)),
                unit_cmd[ui],
                unit_pos[ui],
            )
        )
        chosen = eligible[0]
        scheduled.append(chosen)
        next_pos[unit_cmd[chosen]] += 1
        end_xy = _unit_last_xy(units[chosen], segments, locator)
        if end_xy is not None:
            cur_xy = end_xy
    return scheduled


def _path_length(
    sequence: Sequence[int],
    units: Sequence[Sequence[int]],
    segments: Sequence[_Segment],
    locator: Locator,
) -> float:
    cur: Tuple[float, float] = (0.0, 0.0)
    total = 0.0
    for ui in sequence:
        first = _unit_first_xy(units[ui], segments, locator)
        last = _unit_last_xy(units[ui], segments, locator)
        if first is not None:
            total += _euclidean(cur, first)
            cur = last if last is not None else first
    return total


def _reversal_preserves_order(
    sequence: Sequence[int], lo: int, hi: int, unit_cmd: Sequence[int]
) -> bool:
    """A reversal of sequence[lo+1..hi] is safe iff no command appears twice
    in that slice — otherwise reversing would swap two of its units."""
    seen = set()
    for k in range(lo + 1, hi + 1):
        c = unit_cmd[sequence[k]]
        if c in seen:
            return False
        seen.add(c)
    return True


def _two_opt(
    sequence: List[int],
    units: Sequence[Sequence[int]],
    unit_cmd: Sequence[int],
    segments: Sequence[_Segment],
    locator: Locator,
) -> List[int]:
    """Precedence-safe 2-opt smoothing at the unit level."""
    n = len(sequence)
    if n < 4:
        return sequence
    improved = True
    best = list(sequence)
    best_len = _path_length(best, units, segments, locator)
    while improved:
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n):
                if not _reversal_preserves_order(best, i, j, unit_cmd):
                    continue
                candidate = best[: i + 1] + best[i + 1 : j + 1][::-1] + best[j + 1 :]
                cand_len = _path_length(candidate, units, segments, locator)
                if cand_len + 1e-9 < best_len:
                    best = candidate
                    best_len = cand_len
                    improved = True
        # restart from scratch on improvement to allow chained gains
    return best


def merge(commands: Sequence[Any], locator: Optional[Locator] = None) -> InterleavedPlan:
    """Merge N parsed commands into one interleaved plan.

    ``commands`` is a sequence of BAML ``CommandListLLM`` objects.
    ``locator`` resolves a free-text location string to ``(x, y)``; pass
    ``None`` to disable distance-based ordering (useful in tests).
    """
    loc_fn: Locator = locator if locator is not None else (lambda _: None)

    if not commands:
        return InterleavedPlan(actions=[], fallback=[])

    # Decompose each command into segments and assemble a flat segment list.
    flat_segments: List[_Segment] = []
    cmd_seg_offsets: List[int] = []
    for cmd_idx, command in enumerate(commands):
        cmd_seg_offsets.append(len(flat_segments))
        flat_segments.extend(_decompose(cmd_idx, command))

    # Build per-command units (lists of global segment indices).
    units: List[List[int]] = []
    unit_cmd: List[int] = []
    unit_pos: List[int] = []
    for cmd_idx in range(len(commands)):
        start = cmd_seg_offsets[cmd_idx]
        end = cmd_seg_offsets[cmd_idx + 1] if cmd_idx + 1 < len(commands) else len(flat_segments)
        local = flat_segments[start:end]
        local_units = _build_units(local)
        for pos, u in enumerate(local_units):
            units.append([li + start for li in u])
            unit_cmd.append(cmd_idx)
            unit_pos.append(pos)

    if not units:
        return InterleavedPlan(actions=[], fallback=_build_fallback(commands))

    schedule = _greedy_schedule(units, unit_cmd, unit_pos, len(commands), flat_segments, loc_fn)
    schedule = _two_opt(schedule, units, unit_cmd, flat_segments, loc_fn)

    actions: List[PlanAction] = []
    for ui in schedule:
        for gi in units[ui]:
            seg = flat_segments[gi]
            for ai in seg.indices:
                action = commands[seg.cmd].commands[ai]
                actions.append(_to_plan_action(action, seg.cmd, ai))

    return InterleavedPlan(actions=actions, fallback=_build_fallback(commands))


def _to_plan_action(action: Any, cmd_idx: int, action_idx: int) -> PlanAction:
    kind = _kind(action)
    return PlanAction(
        action=action,
        source_cmd=cmd_idx,
        source_idx=action_idx,
        location=getattr(action, "location_to_go", None),
        requires_gripper=(kind in _GRIPPER_ACQUIRES),
        releases_gripper=(kind in _GRIPPER_RELEASES),
    )


def _build_fallback(commands: Sequence[Any]) -> List[List[PlanAction]]:
    out: List[List[PlanAction]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd: List[PlanAction] = []
        for ai, action in enumerate(getattr(command, "commands", [])):
            per_cmd.append(_to_plan_action(action, cmd_idx, ai))
        out.append(per_cmd)
    return out


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

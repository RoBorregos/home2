"""GPSR multi-command interleaving merger.

Single-item, drop-before-pick model. Takes a list of parsed BAML
``CommandListLLM`` objects (one per spoken utterance) and returns a single
``InterleavedPlan`` whose ``actions`` list executes the commands in an
order that:

  * preserves intra-command sequencing,
  * preserves precedence edges across atomic gripper blocks,
  * never holds more than one object at a time,
  * minimizes 2-D travel between locations using a precedence-constrained
    Held-Karp DP (optimal under the Euclidean cost model).

Pipeline:
  1. Decompose each command into go_to-led segments.
  2. Contract pick→…→give|place chains into atomic units; gripper-neutral
     segments stay as single-segment units.
  3. Held-Karp DP over the units with a per-command prefix-order
     eligibility constraint produces the globally optimal schedule.
  4. Walk units back into a flat PlanAction list.

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


def _optimal_schedule(
    units: Sequence[Sequence[int]],
    unit_cmd: Sequence[int],
    unit_pos: Sequence[int],
    n_cmds: int,
    segments: Sequence[_Segment],
    locator: Locator,
) -> List[int]:
    """Held-Karp DP with per-command prefix-order eligibility.

    State (S, last):
        S    — bitmask of units already scheduled
        last — index of the most recently scheduled unit

    f(S, last) is the minimum total Euclidean travel for any schedule that
    realises S, ends at `last`, and respects the per-command prefix order
    (units of command k must be scheduled in their original positions).

    Recurrence:
        f({u},  u) = dist(origin, start_xy[u])              if unit_pos[u] == 0
        f(S|{u}, u) = min over last in S, u not in S, u eligible:
                        f(S, last) + dist(end_xy[last], start_xy[u])

    Eligibility for adding unit u to S: the count of units of unit_cmd[u]
    already in S must equal unit_pos[u] — i.e. all earlier siblings of u
    are scheduled and u is the next one due.

    Complexity: O(2^M · M²). With our atomic-block decomposition M is
    typically 6–9 and bounded by ~16 in batches, so this is sub-millisecond.
    Returns the optimal schedule under the Euclidean cost model.
    """
    M = len(units)
    if M == 0:
        return []

    start_xy = [_unit_first_xy(units[u], segments, locator) for u in range(M)]
    end_xy = [_unit_last_xy(units[u], segments, locator) for u in range(M)]

    cmd_mask = [0] * n_cmds
    for u in range(M):
        cmd_mask[unit_cmd[u]] |= 1 << u

    INF = float("inf")
    full = (1 << M) - 1

    # f[S][u] = min cost to reach subset S ending at unit u.
    # parent[S][u] = previous unit (or -1 if u was first, came from origin).
    f = [[INF] * M for _ in range(1 << M)]
    parent = [[-1] * M for _ in range(1 << M)]

    origin = (0.0, 0.0)
    for u in range(M):
        if unit_pos[u] != 0:
            continue
        f[1 << u][u] = _euclidean(origin, start_xy[u])
        parent[1 << u][u] = -1

    # Forward DP. Iterating S in integer order is a valid topological
    # order: any subset that adds a bit only references smaller subsets.
    for S in range(1, 1 << M):
        for last in range(M):
            if not (S >> last) & 1:
                continue
            cur_cost = f[S][last]
            if cur_cost == INF:
                continue
            for u in range(M):
                if (S >> u) & 1:
                    continue
                cnt = bin(S & cmd_mask[unit_cmd[u]]).count("1")
                if cnt != unit_pos[u]:
                    continue
                cand = cur_cost + _euclidean(end_xy[last], start_xy[u])
                new_S = S | (1 << u)
                if cand < f[new_S][u]:
                    f[new_S][u] = cand
                    parent[new_S][u] = last

    best_last = -1
    best_cost = INF
    for u in range(M):
        if f[full][u] < best_cost:
            best_cost = f[full][u]
            best_last = u

    if best_last < 0:
        # Pathological — DP could not realise the full schedule.
        # Fall back to per-command sequential order, which is always feasible.
        return sorted(range(M), key=lambda u: (unit_cmd[u], unit_pos[u]))

    schedule: List[int] = []
    S = full
    last = best_last
    while last != -1:
        schedule.append(last)
        prev = parent[S][last]
        S ^= 1 << last
        last = prev
    schedule.reverse()
    return schedule


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

    schedule = _optimal_schedule(units, unit_cmd, unit_pos, len(commands), flat_segments, loc_fn)

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

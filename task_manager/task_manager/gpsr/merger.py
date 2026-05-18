"""GPSR multi-command interleaving merger.

Single-item, drop-before-pick model. Takes a list of parsed BAML
``CommandListLLM`` objects (one per spoken utterance) and returns a single
``InterleavedPlan`` whose ``actions`` list executes the commands in an
order that:

  * preserves intra-command sequencing,
  * keeps the gripper invariant (one item at a time, drop before pick),
  * minimizes 2-D travel between locations using a precedence-constrained
    Held-Karp DP at the *segment* granularity, allowing gripper-neutral
    segments of any command to interleave inside another command's atomic
    block,
  * dedups back-to-back same-location ``go_to`` actions in the final plan.

Pipeline:
  1. Decompose each command into go_to-led segments.
  2. Held-Karp DP over the segments with two-part eligibility:
       - per-command prefix order (cmd k's segments scheduled in their
         original positions),
       - gripper-state check (only neutral segments may interleave while
         another command is mid-block).
  3. Walk the schedule back into a flat PlanAction list, then collapse
     consecutive same-location go_tos.

The merger is intentionally framework-free: it talks to BAML objects via
``getattr`` so unit tests can pass simple stand-ins.
"""

from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence, Tuple

# Action-kind classifications.
_GRIPPER_ACQUIRES = frozenset({"pick_object"})
_GRIPPER_RELEASES = frozenset({"give_object", "place_object"})

# Sentinel cost for any edge touching an unresolved (None) coordinate. Well
# above any realistic indoor navigation distance, so the DP avoids unknown
# locations unless the per-command prefix forces them into a slot.
LARGE_M = 1000.0

# Cap on the number of segments before we fall back to per-command
# sequential. Held-Karp is O(2^M * M^2); 16 keeps the worst case sub-second.
_M_CAP = 16

# Tiebreaker weight: tiny per-cross-command-switch penalty so equal-cost
# schedules prefer per-command sequential order. Must stay well below any
# real distance difference.
_TIEBREAK_EPSILON = 1e-6


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


def _euclidean(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> float:
    if a is None or b is None:
        return LARGE_M
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5


def _segment_xy(seg: _Segment, locator: Locator) -> Optional[Tuple[float, float]]:
    if seg.location is None:
        return None
    return locator(seg.location)


def _seg_kind(seg: _Segment) -> str:
    if seg.acquires:
        return "acquires"
    if seg.releases:
        return "releases"
    return "neutral"


def _build_holding_after(
    segments_per_cmd: Sequence[Sequence[_Segment]],
) -> List[List[bool]]:
    """For each command c, ``holding_after[c][p]`` is True iff scheduling
    ``p`` segments of cmd c (in source order) leaves the gripper held.

    p ranges 0..len(segments_per_cmd[c]).
    """
    result: List[List[bool]] = []
    for segs in segments_per_cmd:
        held = False
        ha = [False]
        for seg in segs:
            if seg.acquires:
                held = True
            if seg.releases:
                held = False
            ha.append(held)
        result.append(ha)
    return result


# Dont really understand
def _gripper_holder(
    S: int,
    n_cmds: int,
    cmd_seg_mask: Sequence[int],
    holding_after: Sequence[Sequence[bool]],
) -> Optional[int]:
    """Return the cmd currently holding the gripper given subset S, or None.

    By the single-item invariant, at most one command can be the holder at
    any time, so we return the first match.
    """
    for c in range(n_cmds):
        p = (S & cmd_seg_mask[c]).bit_count()
        if holding_after[c][p]:
            return c
    return None


def _optimal_schedule(
    segments: Sequence[_Segment],
    seg_cmd: Sequence[int],
    seg_pos: Sequence[int],
    cmd_seg_mask: Sequence[int],
    holding_after: Sequence[Sequence[bool]],
    seg_kinds: Sequence[str],
    n_cmds: int,
    locator: Locator,
    origin: Optional[Tuple[float, float]],
) -> List[int]:
    """Held-Karp DP at segment granularity.

    State (S, last):
        S    — bitmask of segments already scheduled
        last — index of the most recently scheduled segment

    Eligibility for adding segment u to S:
      1. Per-command prefix: bin(S & cmd_seg_mask[seg_cmd[u]]).count("1")
         must equal seg_pos[u].
      2. Gripper-state: if some command c' currently holds the gripper and
         c' != seg_cmd[u], then seg_kinds[u] must be "neutral".

    Cost: f(S|{u}, u) = f(S, last) + dist(seg_xy[last], seg_xy[u])
                       + epsilon * (seg_cmd[u] != seg_cmd[last]).
    Seed: f({u}, u) = dist(origin, seg_xy[u]) + epsilon * seg_cmd[u].

    The epsilon term breaks ties (e.g. all-unknown locations where every
    edge costs LARGE_M) toward per-command sequential order. It is small
    enough to never override a real distance difference.

    Complexity: O(2^M · M²). M is capped at _M_CAP by the caller.
    """
    M = len(segments)
    if M == 0:
        return []

    seg_xy = [_segment_xy(segments[u], locator) for u in range(M)]

    INF = float("inf")
    full = (1 << M) - 1

    f = [[INF] * M for _ in range(1 << M)]
    parent = [[-1] * M for _ in range(1 << M)]

    for u in range(M):
        if seg_pos[u] != 0:
            continue
        seed = 0.0 if origin is None else _euclidean(origin, seg_xy[u])
        f[1 << u][u] = seed + _TIEBREAK_EPSILON * seg_cmd[u]
        parent[1 << u][u] = -1

    for S in range(1, 1 << M):
        holder = _gripper_holder(S, n_cmds, cmd_seg_mask, holding_after)
        for last in range(M):
            if not (S >> last) & 1:
                continue
            cur_cost = f[S][last]
            if cur_cost == INF:
                continue
            for u in range(M):
                if (S >> u) & 1:
                    continue
                cnt = (S & cmd_seg_mask[seg_cmd[u]]).bit_count()
                if cnt != seg_pos[u]:
                    continue
                if holder is not None and holder != seg_cmd[u]:
                    if seg_kinds[u] != "neutral":
                        continue
                cand = (
                    cur_cost
                    + _euclidean(seg_xy[last], seg_xy[u])
                    + (_TIEBREAK_EPSILON if seg_cmd[u] != seg_cmd[last] else 0.0)
                )
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
        # Per-command sequential order is always feasible.
        return _sequential_schedule(seg_cmd, seg_pos)

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


def _sequential_schedule(seg_cmd: Sequence[int], seg_pos: Sequence[int]) -> List[int]:
    """Per-command sequential order — feasible by construction (preserves
    each command's prefix and never interleaves across atomic blocks)."""
    return sorted(range(len(seg_cmd)), key=lambda u: (seg_cmd[u], seg_pos[u]))


def merge(
    commands: Sequence[Any],
    locator: Optional[Locator] = None,
    origin: Optional[Tuple[float, float]] = None,
) -> InterleavedPlan:
    """Merge N parsed commands into one interleaved plan.

    ``commands`` is a sequence of BAML ``CommandListLLM`` objects.
    ``locator`` resolves a free-text location string to ``(x, y)``; pass
    ``None`` to disable distance-based ordering (useful in tests).
    ``origin`` is the robot's current ``(x, y)`` at planning time; pass
    ``None`` to seed the DP with no origin edge (avoids the misleading
    bias toward the SLAM map origin when the real pose is unavailable).
    """
    loc_fn: Locator = locator if locator is not None else (lambda _: None)

    if not commands:
        return InterleavedPlan(actions=[], fallback=[])

    flat_segments: List[_Segment] = []
    segments_per_cmd: List[List[_Segment]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd = _decompose(cmd_idx, command)
        segments_per_cmd.append(per_cmd)
        flat_segments.extend(per_cmd)

    n_cmds = len(commands)
    M = len(flat_segments)
    if M == 0:
        return InterleavedPlan(actions=[], fallback=_build_fallback(commands))

    seg_cmd: List[int] = []
    seg_pos: List[int] = []
    cmd_seg_mask: List[int] = [0] * n_cmds
    gi = 0
    for c, per_cmd in enumerate(segments_per_cmd):
        for p in range(len(per_cmd)):
            seg_cmd.append(c)
            seg_pos.append(p)
            cmd_seg_mask[c] |= 1 << gi
            gi += 1

    seg_kinds = [_seg_kind(seg) for seg in flat_segments]
    holding_after = _build_holding_after(segments_per_cmd)

    if M > _M_CAP:
        schedule = _sequential_schedule(seg_cmd, seg_pos)
    else:
        schedule = _optimal_schedule(
            flat_segments,
            seg_cmd,
            seg_pos,
            cmd_seg_mask,
            holding_after,
            seg_kinds,
            n_cmds,
            loc_fn,
            origin,
        )

    actions: List[PlanAction] = []
    for si in schedule:
        seg = flat_segments[si]
        for ai in seg.indices:
            action = commands[seg.cmd].commands[ai]
            actions.append(_to_plan_action(action, seg.cmd, ai))

    actions = _collapse_redundant_go_tos(actions)
    fallback = [_collapse_redundant_go_tos(per) for per in _build_fallback(commands)]

    return InterleavedPlan(actions=actions, fallback=fallback)


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


def _collapse_redundant_go_tos(
    actions: Sequence[PlanAction],
) -> List[PlanAction]:
    """Drop a leading ``go_to X`` whenever the previous action ended at X."""
    last_loc: Optional[str] = None
    out: List[PlanAction] = []
    for pa in actions:
        if _kind(pa.action) == "go_to":
            tgt = getattr(pa.action, "location_to_go", None)
            if tgt is not None and tgt == last_loc:
                continue
            if tgt is not None:
                last_loc = tgt
        out.append(pa)
    return out

"""GPSR multi-command interleaving merger.

Single-item, drop-before-pick model. Takes a list of parsed BAML
``CommandListLLM`` objects (one per spoken utterance) and returns a single
``InterleavedPlan`` whose ``actions`` list executes the commands in an
order that:

  * preserves intra-command sequencing,
  * keeps the gripper invariant (one item at a time, drop before pick),
  * keeps conflicting segments of different commands in issue order,
  * minimizes travel between locations (2-D Euclidean by default, or any
    caller-supplied ``distance`` such as a planner's path length) using a
    precedence-constrained
    DP over per-command progress vectors at the *segment* granularity,
    allowing gripper-neutral segments of any command to interleave inside
    another command's atomic block,
  * prices each hop from where the previous segment *ends* (an escort's
    destination) and runs actions before a command's first ``go_to`` at the
    robot's start position (driving back there when ``origin_name`` is given),
  * optionally prices the final drive back to the instruction point (``return_to``),
  * dedups back-to-back same-location ``go_to`` actions in the final plan.

Pipeline:
  1. Decompose each command into go_to-led segments.
  2. DP over progress vectors k = (k_0, ..., k_{n-1}), k_c = how many of
     command c's segments are done, with two-part eligibility:
       - per-command prefix order (implicit in k),
       - gripper-state check (only neutral segments may interleave while
         another command is mid-block),
       - cross-command dependencies (see _build_needs).
  3. Walk the schedule back into a flat PlanAction list, then collapse
     consecutive same-location go_tos.

The merger is intentionally framework-free: it talks to BAML objects via
``getattr`` so unit tests can pass simple stand-ins.
"""

from dataclasses import dataclass, field, replace
from math import isfinite
from types import SimpleNamespace
from typing import Any, Callable, Dict, FrozenSet, Mapping, List, Optional, Sequence, Tuple

# Action-kind classifications.
_GRIPPER_ACQUIRES = frozenset({"pick_object"})
_GRIPPER_RELEASES = frozenset({"give_object", "place_object"})
# Escorts end at their destination: kind -> field holding it.
_ESCORT_DESTINATION = {
    "follow_person_until": "destination",
    "guide_person_to": "destination_room",
}
# Stands for the robot's start position in the cost matrix.
_ORIGIN = object()
# Sentinel cost for any edge touching an unresolved (None) coordinate. Well
# above any realistic indoor navigation distance, so the DP avoids unknown
# locations unless the per-command prefix forces them into a slot.
LARGE_M = 1000.0

# Max DP states (n * prod(L_c + 1)) before falling back to sequential.
_STATE_CAP = 200_000


class InfeasibleResidual(ValueError):
    """A legal residual schedule could not be produced within the planning bound."""


class NoInterleavingSchedule(ValueError):
    """No feasible complete schedule has the requested terminal bonus bit."""


@dataclass
class PlanAction:
    """An action with its provenance + gripper effect."""

    action: Any
    source_cmd: int
    source_idx: int
    location: Optional[str]
    requires_gripper: bool
    releases_gripper: bool
    purpose: str = "plan"
    navigation_goals: Tuple[Tuple[int, int], ...] = ()


@dataclass
class InterleavedPlan:
    actions: List[PlanAction]
    fallback: List[List[PlanAction]]
    origin_name: Optional[str] = None


Locator = Callable[[str], Optional[Tuple[float, float]]]
# (from_location or None for the robot origin, to_location) -> meters, or None
# if unknown. A non-finite return (unreachable) also costs LARGE_M.
DistanceFn = Callable[[Optional[str], str], Optional[float]]
# location name -> containing room ("couch" -> "living room"), or None.
RoomFn = Callable[[str], Optional[str]]
_Var = Tuple[str, Optional[str]]  # (domain, place)


@dataclass
class _Segment:
    cmd: int
    indices: List[int]  # action indices within the source command
    location: Optional[str]  # the navigation target (from a leading go_to)
    # Gripper state after the segment, set by its *last* gripper action
    # (True = held, False = empty); None if it has no gripper action,
    # i.e. the segment is gripper-neutral.
    ends_holding: Optional[bool] = None
    # Where the robot stands after the segment if an escort moved it.
    exit: Optional[str] = None
    # True for actions before the command's first go_to: they run at the origin.
    at_origin: bool = False
    # Escorts with a destination, in order: (gripper state set earlier in the
    # segment, None if the segment's start state still holds; destination).
    escorts: List[Tuple[Optional[bool], str]] = field(default_factory=list)
    held_dwell_m: Tuple[float, float] = (0.0, 0.0)
    reads: FrozenSet[_Var] = frozenset()
    writes: FrozenSet[_Var] = frozenset()


def _kind(action: Any) -> str:
    return getattr(action, "action", "") or ""


def _decompose(
    cmd_idx: int, command: Any, dwell_m: Optional[Mapping[str, float]] = None
) -> List[_Segment]:
    """Split a command's action list into go_to-led segments."""
    actions = list(getattr(command, "commands", []))
    segments: List[_Segment] = []
    cur_loc: Optional[str] = None
    cur_indices: List[int] = []
    cur_end: Optional[bool] = None
    cur_exit: Optional[str] = None
    cur_escorts: List[Tuple[Optional[bool], str]] = []
    seen_go_to = False

    def flush() -> None:
        nonlocal cur_indices, cur_end, cur_exit, cur_escorts
        if cur_indices:
            held_dwell = [0.0, 0.0]
            if dwell_m:
                for initial in (0, 1):
                    held = bool(initial)
                    for idx in cur_indices:
                        kind = _kind(actions[idx])
                        if held and kind != "go_to":
                            held_dwell[initial] += dwell_m.get(kind, 0.0)
                        if kind in _GRIPPER_ACQUIRES:
                            held = True
                        elif kind in _GRIPPER_RELEASES:
                            held = False
            segments.append(
                _Segment(
                    cmd=cmd_idx,
                    indices=list(cur_indices),
                    location=cur_loc,
                    ends_holding=cur_end,
                    exit=cur_exit,
                    at_origin=not seen_go_to,
                    escorts=cur_escorts,
                    held_dwell_m=tuple(held_dwell),
                )
            )
        cur_indices = []
        cur_end = None
        cur_exit = None
        cur_escorts = []

    for i, action in enumerate(actions):
        kind = _kind(action)
        if kind == "go_to":
            flush()
            seen_go_to = True
            cur_loc = getattr(action, "location_to_go", None)
            cur_indices.append(i)
        else:
            cur_indices.append(i)
            if kind in _GRIPPER_ACQUIRES:
                cur_end = True
            if kind in _GRIPPER_RELEASES:
                cur_end = False
            if kind in _ESCORT_DESTINATION:
                dest = getattr(action, _ESCORT_DESTINATION[kind], None)
                if isinstance(dest, str) and dest.strip():
                    cur_exit = dest
                    cur_escorts.append((cur_end, dest))
    flush()
    return segments


# Object and person poses are keyed by place. Attention is a single shared
# variable: finding someone at any place changes whom the robot attends to.
_OBJECTS = "objects"
_PEOPLE = "people"
_ATTENTION = "attention"

# kind -> (domains read, domains written) at the current place.
# Unlisted kinds touch no shared state.
_EFFECTS = {
    "pick_object": ({_OBJECTS}, {_OBJECTS}),
    "place_object": (set(), {_OBJECTS}),
    "give_object": ({_PEOPLE}, {_OBJECTS}),
    "count": ({_OBJECTS, _PEOPLE}, set()),
    "get_visual_info": ({_OBJECTS}, set()),
    "find_person": ({_PEOPLE}, {_ATTENTION}),
    "find_person_by_name": ({_PEOPLE}, {_ATTENTION}),
    "get_person_info": ({_PEOPLE, _ATTENTION}, set()),
    # Escorts also write the destination; see _action_effects.
    "follow_person_until": ({_PEOPLE, _ATTENTION}, {_PEOPLE}),
    "guide_person_to": ({_PEOPLE, _ATTENTION}, {_PEOPLE}),
}


def _norm_location(loc: Any) -> Optional[str]:
    if not isinstance(loc, str):
        return None
    loc = loc.strip().lower().replace("_", " ")
    return loc or None


def _action_effects(action: Any, here: Optional[str]) -> Tuple[set, set, Optional[str]]:
    """``(reads, writes, here_after)`` for one action performed at ``here``."""
    kind = _kind(action)
    if kind == "go_to":
        return set(), set(), _norm_location(getattr(action, "location_to_go", None))
    read_domains, write_domains = _EFFECTS.get(kind, (set(), set()))
    reads = {(d, None if d == _ATTENTION else here) for d in read_domains}
    writes = {(d, None if d == _ATTENTION else here) for d in write_domains}
    here_after = here
    if kind in _ESCORT_DESTINATION:
        here_after = _norm_location(getattr(action, _ESCORT_DESTINATION[kind], None))
        writes.add((_PEOPLE, here_after))
    return reads, writes, here_after


def _annotate_effects(
    segments: Sequence[_Segment], command: Any, start: Optional[str] = None
) -> None:
    """Read/write sets per segment; actions before the first go_to act at ``start``."""
    actions = list(getattr(command, "commands", []))
    here: Optional[str] = _norm_location(start)
    for seg in segments:
        reads: set = set()
        writes: set = set()
        for i in seg.indices:
            r, w, here = _action_effects(actions[i], here)
            reads |= r
            writes |= w
        seg.reads = frozenset(reads)
        seg.writes = frozenset(writes)


def _room_table(
    segments: Sequence[_Segment], room_of: Optional[RoomFn]
) -> Dict[str, Optional[str]]:
    places = {v[1] for seg in segments for v in seg.reads | seg.writes if v[1] is not None}
    if room_of is None:
        return {p: None for p in places}
    return {p: _norm_location(room_of(p)) for p in places}


def _places_overlap(a: Optional[str], b: Optional[str], room: Dict[str, Optional[str]]) -> bool:
    if a is None or b is None or a == b:
        return True
    return room.get(a) == b or room.get(b) == a


def _interferes(s: _Segment, t: _Segment, room: Dict[str, Optional[str]]) -> bool:
    def clash(writes: FrozenSet[_Var], others: FrozenSet[_Var]) -> bool:
        return any(
            w[0] == o[0] and _places_overlap(w[1], o[1], room) for w in writes for o in others
        )

    return clash(s.writes, t.reads | t.writes) or clash(t.writes, s.reads)


def _build_needs(
    segments_per_cmd: Sequence[Sequence[_Segment]],
    room: Dict[str, Optional[str]],
) -> List[List[Tuple[Tuple[int, int], ...]]]:
    """``needs[c][j]`` = ``((d, m), ...)``: segment j of command c waits until
    an earlier command d (d < c) has done m segments."""
    needs: List[List[Tuple[Tuple[int, int], ...]]] = []
    for c, segs in enumerate(segments_per_cmd):
        per_cmd = []
        for seg in segs:
            req = []
            for d in range(c):
                m = 0
                for i, other in enumerate(segments_per_cmd[d]):
                    if _interferes(other, seg, room):
                        m = i + 1
                if m:
                    req.append((d, m))
            per_cmd.append(tuple(req))
        needs.append(per_cmd)
    return needs


def _euclidean(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> float:
    if a is None or b is None:
        return LARGE_M
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5


def _euclidean_distance(locator: Locator, origin: Optional[Tuple[float, float]]) -> DistanceFn:
    def dist(a: Optional[str], b: str) -> float:
        return _euclidean(origin if a is None else locator(a), locator(b))

    return dist


def _pricer(distance: DistanceFn, has_origin: bool) -> Callable[[Any, Any], float]:
    """``cost(a, b)`` between names or ``_ORIGIN``, querying ``distance`` once per
    distinct pair. Unresolved or unreachable locations cost LARGE_M."""
    memo: dict = {}

    def cost(a: Any, b: Any) -> float:
        if a is None or b is None:
            return LARGE_M
        if a == b:
            return 0.0
        if a is _ORIGIN or b is _ORIGIN:
            if not has_origin:
                # Unknown origin: leaving it is free, returning to it is not.
                return 0.0 if a is _ORIGIN else LARGE_M
            a, b = None, (b if a is _ORIGIN else a)
        if (a, b) not in memo:
            d = distance(a, b)
            memo[(a, b)] = LARGE_M if d is None or not d < LARGE_M else d
        return memo[(a, b)]

    return cost


def _cost_matrix(
    segments: Sequence[_Segment],
    distance: DistanceFn,
    has_origin: bool,
    origin_name: Optional[str] = None,
) -> Tuple[List[float], List[List[float]]]:
    """Seed costs and segment-to-segment costs."""
    cost = _pricer(distance, has_origin)
    M = len(segments)
    start = _ORIGIN if origin_name is None else origin_name
    entry = [start if seg.at_origin else seg.location for seg in segments]
    leave = [entry[u] if seg.exit is None else seg.exit for u, seg in enumerate(segments)]
    seed = [
        cost(_ORIGIN, entry[u]) if has_origin and not seg.at_origin else 0.0
        for u, seg in enumerate(segments)
    ]
    edge = [[cost(leave[a], entry[b]) for b in range(M)] for a in range(M)]
    return seed, edge


def _carried_inside(
    segments: Sequence[_Segment], cost: Callable[[Any, Any], float], start: Any
) -> List[Tuple[float, float]]:
    """Per segment, escort metres driven while holding: (starting empty, starting
    held). Actions before the first go_to start at ``start``; unknown legs count 0."""
    out = []
    for seg in segments:
        here = start if seg.at_origin else seg.location
        carried = [0.0, 0.0]
        for held_now, dest in seg.escorts:
            d = cost(here, dest)
            for start_held in (0, 1):
                if (start_held if held_now is None else held_now) and d < LARGE_M:
                    carried[start_held] += d
            here = dest
        out.append((carried[0], carried[1]))
    return out


def _build_holding_after(
    segments_per_cmd: Sequence[Sequence[_Segment]],
    initial_holder: Optional[int] = None,
) -> List[List[bool]]:
    """For each command c, ``holding_after[c][p]`` is True iff scheduling
    ``p`` segments of cmd c (in source order) leaves the gripper held.

    p ranges 0..len(segments_per_cmd[c]).
    """
    result: List[List[bool]] = []
    for cmd, segs in enumerate(segments_per_cmd):
        held = cmd == initial_holder
        ha = [held]
        for seg in segs:
            # Order within the segment matters: "place, pick" ends held.
            if seg.ends_holding is not None:
                held = seg.ends_holding
            ha.append(held)
        result.append(ha)
    return result


def _gripper_holder(
    k: Sequence[int],
    holding_after: Sequence[Sequence[bool]],
) -> Optional[int]:
    """The command holding the gripper at progress vector ``k``, or None."""
    for c, kc in enumerate(k):
        if holding_after[c][kc]:
            return c
    return None


def _state_count(seg_lens: Sequence[int]) -> int:
    """Upper bound on DP states: n * prod(L_c + 1)."""
    total = 1
    for length in seg_lens:
        total *= length + 1
    return total * max(len(seg_lens), 1)


def _optimal_schedule(
    segments: Sequence[_Segment],
    seg_ids: Sequence[Sequence[int]],
    holding_after: Sequence[Sequence[bool]],
    distance: DistanceFn,
    has_origin: bool,
    needs: Optional[Sequence[Sequence[Sequence[Tuple[int, int]]]]] = None,
    origin_name: Optional[str] = None,
    holding_penalty: float = 0.0,
    *,
    navigation_overhead: float = 0.0,
    greedy: bool = False,
    interleaved: Optional[bool] = None,
    return_to: Optional[str] = None,
) -> List[int]:
    """DP over per-command progress vectors.

    State (k, last): k[c] = segments of command c done, last = command that
    ran the latest segment (-1 at the start). ``seg_ids[c][j]`` is the flat
    index of command c's j-th segment. States are expanded in layers of equal
    sum(k). O(n^2 * prod(L_c + 1)). Until the robot moves, steps cost ``seed``.
    Every metre driven while holding an object, on a hop or escorting inside a
    segment, costs ``holding_penalty`` extra.
    Optionally charge a fixed cost for driving to a segment.
    ``return_to`` prices the drive there after the last segment like a hop.
    """
    M = len(segments)
    if M == 0:
        return []

    seed, edge = _cost_matrix(segments, distance, has_origin, origin_name)
    cost = _pricer(distance, has_origin)
    start_place = _ORIGIN if origin_name is None else origin_name
    back = [
        cost(seg.exit or (start_place if seg.at_origin else seg.location), return_to)
        if return_to is not None
        else 0.0
        for seg in segments
    ]
    inside = _carried_inside(segments, cost, _ORIGIN if origin_name is None else origin_name)
    inside_unmoved = _carried_inside(segments, cost, _ORIGIN)
    still = (
        {u for u, seg in enumerate(segments) if seg.at_origin and seg.exit is None}
        if origin_name is not None
        else set()
    )
    seg_lens = [len(ids) for ids in seg_ids]
    n = len(seg_lens)

    INF = (float("inf"), 0, 0)
    start = (tuple([0] * n), -1, False)
    f = {start: (0.0, 0, 0)}
    parent: dict = {start: None}

    layer = [start]
    for _ in range(M):
        next_layer: dict = {}  # insertion-ordered set of states in layer + 1
        for state in layer:
            k, last, bit = state
            cur_cost, preference, switches = f[state]
            holder = _gripper_holder(k, holding_after)
            here = seg_ids[last][k[last] - 1] if last >= 0 else -1
            unmoved = here < 0 or (
                here in still
                and all(k[d] == 0 or (k[d] == 1 and seg_ids[d][0] in still) for d in range(n))
            )
            row = seed if unmoved else edge[here]
            carried = inside_unmoved if unmoved else inside
            for c in range(n):
                if k[c] == seg_lens[c]:
                    continue
                u = seg_ids[c][k[c]]
                if holder is not None and holder != c and segments[u].ends_holding is not None:
                    continue
                if needs is not None and any(k[d] < m for d, m in needs[c][k[c]]):
                    continue
                step = row[u]
                if navigation_overhead and 0 < row[u] < LARGE_M:
                    step += navigation_overhead
                if holder is not None:
                    step += holding_penalty * row[u]
                step += holding_penalty * (
                    carried[u][holder is not None] + segments[u].held_dwell_m[holder is not None]
                )
                next_bit = bit
                if interleaved is not None and not bit:
                    next_bit = any(0 < k[d] < seg_lens[d] for d in range(n) if d != c)
                    if next_bit and not interleaved:
                        continue
                nxt = (k[:c] + (k[c] + 1,) + k[c + 1 :], c, next_bit)
                switched = int(last >= 0 and c != last)
                # Preserve the historical issue-order/switch preference only on a cost tie.
                cand = (
                    cur_cost + step,
                    preference + (c if last < 0 else switched),
                    switches + switched,
                )
                if cand < f.get(nxt, INF):
                    f[nxt] = cand
                    parent[nxt] = state
                    next_layer[nxt] = None
        layer = list(next_layer)
        if greedy and layer:
            layer = [min(layer, key=f.__getitem__)]

    full = tuple(seg_lens)
    end_weight = 1.0 + holding_penalty * (_gripper_holder(full, holding_after) is not None)
    best = None
    best_cost = INF
    for c in range(n):
        terminal = (full, c, bool(interleaved))
        if terminal not in f or (greedy and terminal not in layer):
            continue
        ret = back[seg_ids[c][-1]]
        total, *ties = f[terminal]
        total = (
            total + end_weight * ret + (navigation_overhead if 0 < ret < LARGE_M else 0.0),
            *ties,
        )
        if total < best_cost:
            best_cost = total
            best = terminal

    if best is None:
        if interleaved is not None:
            raise NoInterleavingSchedule(f"no feasible schedule with interleaved={interleaved}")
        # Only when 2+ commands pick without ever releasing (a parse error).
        return _sequential_schedule(segments)

    schedule: List[int] = []
    state = best
    while state is not None and state[1] >= 0:
        k, last, _ = state
        schedule.append(seg_ids[last][k[last] - 1])
        state = parent[state]
    schedule.reverse()
    return schedule


def _sequential_schedule(segments: Sequence[_Segment]) -> List[int]:
    """Per-command sequential order — feasible by construction (preserves
    each command's prefix and never interleaves across atomic blocks).
    The sort is stable, so each command keeps its own segment order."""
    return sorted(range(len(segments)), key=lambda u: segments[u].cmd)


def merge(
    commands: Sequence[Any],
    locator: Optional[Locator] = None,
    origin: Optional[Tuple[float, float]] = None,
    distance: Optional[DistanceFn] = None,
    dependencies: bool = True,
    room_of: Optional[RoomFn] = None,
    origin_name: Optional[str] = None,
    holding_penalty: float = 0.0,
    *,
    navigation_overhead: float = 0.0,
    skill_durations_s: Optional[Mapping[str, float]] = None,
    speed_m_s: float = 1.0,
    initial_holder: Optional[int] = None,
    greedy: bool = False,
    interleaved: Optional[bool] = None,
    return_to: Optional[str] = None,
    scheduler: Optional[Callable[..., List[int]]] = None,
) -> InterleavedPlan:
    """Merge N parsed commands into one interleaved plan.

    ``commands`` is a sequence of BAML ``CommandListLLM`` objects.
    ``locator`` resolves a free-text location string to ``(x, y)``; pass
    ``None`` to disable distance-based ordering (useful in tests).
    ``origin`` is the robot's current ``(x, y)`` at planning time; pass
    ``None`` to seed the DP with no origin edge (avoids the misleading
    bias toward the SLAM map origin when the real pose is unavailable).
    ``distance`` overrides the straight-line cost (e.g. a planner's path
    length) and takes location names, ``None`` meaning the robot origin;
    it is only asked about the origin when ``origin`` is not None.
    ``dependencies=False`` disables cross-command conflict ordering.
    ``room_of`` maps a location to its room, so a room overlaps the places
    inside it; without it only identical names conflict.
    ``origin_name`` names the robot's start; a ``go_to`` to it is inserted
    before actions that precede a command's first ``go_to``, and it prices
    the origin when ``origin`` is None. It is ignored if ``distance`` cannot
    reach it.
    ``holding_penalty`` (the paper's lambda) makes every metre driven while
    holding an object cost ``1 + holding_penalty``; 0 minimises plain travel.
    ``navigation_overhead`` adds a nonnegative distance-equivalent cost to
    each reachable, positive-distance go_to hop in the command plan.
    skill_durations_s models stationary skill time. An action changes possession
    at completion. Multiply occupied seconds by speed_m_s to price them in metres.
    initial_holder identifies a verified carry at the start of a residual plan.
    greedy selects the cheapest eligible next step for the benchmark baseline.
    interleaved requests the exact minimum with the paper's terminal bit fixed
    to True or False. None uses the ordinary unrestricted objective.
    ``return_to`` names the instruction point the robot drives back to after
    the last action; that drive is priced like a hop. It is ignored if
    ``distance`` cannot reach it.
    """
    if not isfinite(speed_m_s) or speed_m_s <= 0:
        raise ValueError("speed_m_s must be finite and positive")
    dwell_m = {}
    for kind, duration in (skill_durations_s or {}).items():
        if not isfinite(duration) or duration < 0:
            raise ValueError(f"invalid skill duration for {kind}")
        dwell_m[kind] = duration * speed_m_s
    if initial_holder is not None and not 0 <= initial_holder < len(commands):
        raise ValueError("initial_holder must identify a command")
    if not isfinite(navigation_overhead) or navigation_overhead < 0:
        raise ValueError("navigation_overhead must be finite and nonnegative")
    if distance is None:
        loc_fn: Locator = locator if locator is not None else (lambda _: None)
        distance = _euclidean_distance(loc_fn, origin)
    if origin_name is not None and not _reachable(
        origin_name, commands, distance, origin is not None
    ):
        origin_name = None
    if return_to is not None and not _reachable(return_to, commands, distance, origin is not None):
        return_to = None
    if origin is None and origin_name is not None:
        base = distance

        def distance(a: Optional[str], b: str) -> Optional[float]:
            return base(origin_name if a is None else a, b)

    has_origin = origin is not None or origin_name is not None

    if scheduler is not None and (greedy or interleaved is not None or return_to is not None):
        raise ValueError("a custom scheduler cannot be combined with built-in variants")
    if interleaved is not None and greedy:
        raise ValueError("a fixed interleaving bit requires exact scheduling")

    flat_segments: List[_Segment] = []
    segments_per_cmd: List[List[_Segment]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd = _decompose(cmd_idx, command, dwell_m)
        _annotate_effects(per_cmd, command, start=origin_name)
        segments_per_cmd.append(per_cmd)
        flat_segments.extend(per_cmd)

    if not flat_segments:
        if interleaved:
            raise NoInterleavingSchedule("an empty batch cannot interleave")
        return InterleavedPlan(actions=[], fallback=_build_fallback(commands, origin_name))

    # seg_ids[c][j] = flat index of command c's j-th segment.
    seg_ids: List[List[int]] = []
    gi = 0
    for per_cmd in segments_per_cmd:
        seg_ids.append(list(range(gi, gi + len(per_cmd))))
        gi += len(per_cmd)

    holding_after = _build_holding_after(segments_per_cmd, initial_holder)
    needs = (
        _build_needs(segments_per_cmd, _room_table(flat_segments, room_of))
        if dependencies
        else None
    )

    bit_states = 2 if interleaved else 1
    above_cap = bit_states * _state_count([len(ids) for ids in seg_ids]) > _STATE_CAP
    if above_cap:
        if interleaved is not None:
            raise ValueError("exact interleaving search exceeds the state bound")
        if initial_holder is not None:
            raise InfeasibleResidual("residual DP exceeds the state bound")
        schedule = _sequential_schedule(flat_segments)
    else:
        variant = (
            dict(greedy=greedy, interleaved=interleaved, return_to=return_to)
            if scheduler is None
            else {}
        )
        schedule = (scheduler or _optimal_schedule)(
            flat_segments,
            seg_ids,
            holding_after,
            distance,
            has_origin,
            needs,
            origin_name,
            holding_penalty,
            navigation_overhead=navigation_overhead,
            **variant,
        )
        if schedule is None:
            schedule = _sequential_schedule(flat_segments)

    actions = _flatten(schedule, flat_segments, commands, origin_name)
    if initial_holder is not None:
        owner = initial_holder
        for step in actions:
            if step.requires_gripper:
                if owner is not None:
                    raise InfeasibleResidual("remaining pick requires an empty gripper")
                owner = step.source_cmd
            elif step.releases_gripper:
                if owner != step.source_cmd:
                    raise InfeasibleResidual("remaining release requires its original holder")
                owner = None
    fallback = [_collapse_redundant_go_tos(per) for per in _build_fallback(commands, origin_name)]

    return InterleavedPlan(actions=actions, fallback=fallback, origin_name=origin_name)


def issued_order_plan(plan: InterleavedPlan) -> InterleavedPlan:
    """Execute the original commands consecutively through the batch executor."""
    actions = [action for command in plan.fallback for action in command]
    return replace(plan, actions=_collapse_redundant_go_tos(actions, start=plan.origin_name))


def replan_remaining(commands: Sequence[Sequence[PlanAction]], **options) -> List[PlanAction]:
    """Solve residual commands with explicit entry drives, retaining source identities."""
    raw = [SimpleNamespace(commands=[step.action for step in command]) for command in commands]
    plan = merge(raw, **options)
    result = []
    for step in plan.actions:
        if step.source_idx < 0:
            result.append(step)
            continue
        original = commands[step.source_cmd][step.source_idx]
        goals = tuple(
            goal
            for cmd, idx in step.navigation_goals
            for goal in commands[cmd][idx].navigation_goals
        )
        result.append(replace(original, navigation_goals=goals))
    return result


def _reachable(
    origin_name: str, commands: Sequence[Any], distance: DistanceFn, has_pose: bool
) -> bool:
    """Whether ``distance`` can price a drive to ``origin_name``: from the robot's
    pose if known, otherwise from the start to some place in the batch."""
    if has_pose:
        costs = [distance(None, origin_name)]
    else:
        places = {
            getattr(a, "location_to_go", None)
            for cmd in commands
            for a in getattr(cmd, "commands", [])
        }
        costs = [
            distance(origin_name, p) for p in places if isinstance(p, str) and p != origin_name
        ]
    return any(d is not None and d < LARGE_M for d in costs)


def _flatten(
    schedule: Sequence[int],
    segments: Sequence[_Segment],
    commands: Sequence[Any],
    origin_name: Optional[str],
) -> List[PlanAction]:
    """Segments -> PlanActions, inserting go_to(origin_name) and collapsing go_tos."""
    actions: List[PlanAction] = []
    for si in schedule:
        seg = segments[si]
        if seg.at_origin and origin_name is not None:
            actions.append(_go_to_origin(origin_name, seg.cmd))
        for ai in seg.indices:
            actions.append(_command_plan_action(commands[seg.cmd].commands, seg.cmd, ai))
    return _collapse_redundant_go_tos(actions, start=origin_name)


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


def _command_plan_action(actions: Sequence[Any], cmd_idx: int, action_idx: int) -> PlanAction:
    pa = _to_plan_action(actions[action_idx], cmd_idx, action_idx)
    if _kind(pa.action) == "go_to" and (
        action_idx + 1 == len(actions) or _kind(actions[action_idx + 1]) == "go_to"
    ):
        pa.navigation_goals = ((cmd_idx, action_idx),)
    return pa


def _go_to_origin(origin_name: str, cmd_idx: int) -> PlanAction:
    """Inserted ``go_to`` back to the start; ``source_idx=-1`` marks it."""
    return _to_plan_action(SimpleNamespace(action="go_to", location_to_go=origin_name), cmd_idx, -1)


def _build_fallback(
    commands: Sequence[Any], origin_name: Optional[str] = None
) -> List[List[PlanAction]]:
    out: List[List[PlanAction]] = []
    for cmd_idx, command in enumerate(commands):
        per_cmd: List[PlanAction] = []
        actions = list(getattr(command, "commands", []))
        if origin_name is not None and actions and _kind(actions[0]) != "go_to":
            per_cmd.append(_go_to_origin(origin_name, cmd_idx))
        for ai in range(len(actions)):
            per_cmd.append(_command_plan_action(actions, cmd_idx, ai))
        out.append(per_cmd)
    return out


def _collapse_redundant_go_tos(
    actions: Sequence[PlanAction],
    start: Optional[str] = None,
) -> List[PlanAction]:
    """Drop a leading ``go_to X`` whenever the previous action ended at X
    (``start`` is where the robot is before the first action)."""
    last_loc: Optional[str] = start
    last_drive = None
    out: List[PlanAction] = []
    for pa in actions:
        if _kind(pa.action) in _ESCORT_DESTINATION:
            last_loc = None  # the escort moved the robot
            last_drive = None
        if _kind(pa.action) == "go_to":
            tgt = getattr(pa.action, "location_to_go", None)
            if tgt is not None and tgt == last_loc:
                if pa.navigation_goals and last_drive is not None:
                    prior = out[last_drive]
                    out[last_drive] = replace(
                        prior, navigation_goals=prior.navigation_goals + pa.navigation_goals
                    )
                continue
            if tgt is not None:
                last_loc = tgt
                last_drive = len(out)
        out.append(pa)
    return out

"""Nominal deadline planning for already-parsed remaining GPSR commands."""

from dataclasses import dataclass
from functools import lru_cache
import math

from task_manager.gpsr.merger import (
    _ESCORT_DESTINATION,
    _GRIPPER_ACQUIRES,
    _GRIPPER_RELEASES,
    _STATE_CAP,
    _annotate_effects,
    _build_holding_after,
    _build_needs,
    _decompose,
    _flatten,
    _gripper_holder,
    _kind,
    _room_table,
)


@dataclass
class EndgamePlan:
    actions: list
    segment_order: tuple
    completed: tuple
    return_to: str
    return_s: float
    elapsed_s: float  # includes the terminal return


def plan_endgame(
    commands,
    *,
    start,
    home,
    travel_s,
    skill_durations_s,
    remaining_s,
    bin_s=1,
    initial_holder=None,
    room_of=None,
):
    """Maximize completed residual commands, then minimize time including return.

    travel_s(a, b) gives nominal seconds for the whole leg or None if unreachable.
    Each selected segment and the return round up separately for the time budget.
    Durations in the result are unrounded. A None result means even the best
    schedule cannot return in budget. Earlier observations and possession must
    have been verified by the caller before supplying residual commands.
    """
    if not math.isfinite(remaining_s) or remaining_s < 0 or not math.isfinite(bin_s) or bin_s <= 0:
        raise ValueError("remaining_s must be nonnegative and bin_s positive and finite")
    if not start or not home:
        raise ValueError("start and home must be named locations")
    if initial_holder is not None and not 0 <= initial_holder < len(commands):
        raise ValueError("initial_holder must identify a command")
    if any(not math.isfinite(t) or t < 0 for t in skill_durations_s.values()):
        raise ValueError("skill durations must be finite and nonnegative")

    @lru_cache(None)
    def drive(a, b):
        if a == b:
            return 0.0
        seconds = travel_s(a, b)
        if seconds is None or seconds == math.inf:
            return math.inf
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("invalid travel time")
        return seconds

    per_cmd = [_decompose(c, cmd) for c, cmd in enumerate(commands)]
    flat = [seg for chain in per_cmd for seg in chain]
    for cmd, chain in zip(commands, per_cmd):
        _annotate_effects(chain, cmd, start=start)
    needs = _build_needs(per_cmd, _room_table(flat, room_of))
    holding = _build_holding_after(per_cmd, initial_holder)
    lengths = tuple(map(len, per_cmd))
    details = {}
    for idx, seg in enumerate(flat):
        c = seg.cmd
        entry = start if seg.at_origin else seg.location
        if not entry:
            raise ValueError("navigation segments require named destinations")
        pose = entry
        seconds = 0.0
        gripper = []
        for i in seg.indices:
            a = commands[c].commands[i]
            kind = _kind(a)
            if kind == "go_to":
                continue
            if kind not in skill_durations_s:
                raise ValueError(f"missing nominal duration for {kind}")
            seconds += skill_durations_s[kind]
            if kind in _GRIPPER_ACQUIRES or kind in _GRIPPER_RELEASES:
                gripper.append(kind in _GRIPPER_ACQUIRES)
            if kind in _ESCORT_DESTINATION:
                dest = getattr(a, _ESCORT_DESTINATION[kind], None)
                if dest:
                    seconds += drive(pose, dest)
                    pose = dest
        key = (c, seg.indices[0])
        details[key] = (idx, entry, pose, seconds, gripper)

    visited = 0

    @lru_cache(None)
    def solve(k, place, left):
        nonlocal visited
        visited += 1
        if visited > _STATE_CAP:
            raise ValueError("deadline search exceeds the state bound")
        # Stopping now is a real decision, even with unfinished commands.
        back = drive(place, home)
        goals = sum(length > 0 and done == length for done, length in zip(k, lengths))
        best = (-goals, back, ()) if back <= left * bin_s else None
        holder = _gripper_holder(k, holding)
        for c, done in enumerate(k):
            if done == lengths[c] or any(k[d] < m for d, m in needs[c][done]):
                continue
            seg = per_cmd[c][done]
            key = (c, seg.indices[0])
            idx, entry, exit_site, inside, gripper = details[key]
            owner = holder
            for acquire in gripper:
                if (acquire and owner is not None) or (not acquire and owner != c):
                    break
                owner = c if acquire else None
            else:
                step = drive(place, entry) + inside
                if step > left * bin_s:
                    continue
                next_k = k[:c] + (done + 1,) + k[c + 1 :]
                suffix = solve(next_k, exit_site, left - math.ceil(step / bin_s))
                if suffix is not None:
                    score, duration, order = suffix
                    candidate = (score, step + duration, (idx,) + order)
                    if best is None or candidate < best:
                        best = candidate
        return best

    result = solve((0,) * len(commands), start, math.floor(remaining_s / bin_s))
    if result is None:
        return None
    _, elapsed, schedule = result
    last = flat[schedule[-1]] if schedule else None
    final_place = (last.exit or last.location or start) if last else start
    counts = [0] * len(commands)
    order = []
    for idx in schedule:
        c = flat[idx].cmd
        order.append((c, counts[c]))
        counts[c] += 1
    completed = tuple(c for c, done in enumerate(counts) if done and done == lengths[c])
    return EndgamePlan(
        _flatten(schedule, flat, commands, start),
        tuple(order),
        completed,
        home,
        drive(final_place, home),
        elapsed,
    )

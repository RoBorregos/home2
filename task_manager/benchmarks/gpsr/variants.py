"""Paper-only merger variants: the greedy baseline and the fixed interleaving bit."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_manager.gpsr.merger import (  # noqa: E402
    _STATE_CAP,
    _decompose,
    _expander,
    _state_count,
    _tie_cost,
    _walk_back,
    merge,
)


class NoInterleavingSchedule(ValueError):
    """No feasible complete schedule has the requested terminal bonus bit."""


def variant_schedule(
    segments,
    seg_ids,
    holding_after,
    distance,
    has_origin,
    needs=None,
    origin_name=None,
    holding_penalty=0.0,
    *,
    navigation_overhead=0.0,
    greedy=False,
    interleaved=None,
):
    """merger._optimal_schedule plus greedy pruning and the terminal interleaving bit."""
    if not segments:
        return []
    expand = _expander(
        segments,
        seg_ids,
        holding_after,
        distance,
        has_origin,
        needs,
        origin_name,
        holding_penalty,
        navigation_overhead,
    )
    seg_lens = [len(ids) for ids in seg_ids]
    n = len(seg_lens)

    INF = (float("inf"), 0, 0)
    start = (tuple([0] * n), -1, False)
    f = {start: (0.0, 0, 0)}
    parent = {start: None}
    layer = [start]
    for _ in range(len(segments)):
        next_layer = {}
        for state in layer:
            k, last, bit = state
            for c, step in expand(k, last):
                next_bit = bit
                if interleaved is not None and not bit:
                    # Interleaving = starting a segment while another command is mid-way.
                    next_bit = any(0 < k[d] < seg_lens[d] for d in range(n) if d != c)
                    if next_bit and not interleaved:
                        continue
                nxt = (k[:c] + (k[c] + 1,) + k[c + 1 :], c, next_bit)
                cand = _tie_cost(f[state], c, last, step)
                if cand < f.get(nxt, INF):
                    f[nxt] = cand
                    parent[nxt] = state
                    next_layer[nxt] = None
        layer = list(next_layer)
        if greedy and layer:
            layer = [min(layer, key=f.__getitem__)]

    full = tuple(seg_lens)
    best, best_cost = None, INF
    for c in range(n):
        terminal = (full, c, bool(interleaved))
        if greedy and terminal not in layer:
            continue
        total = f.get(terminal, INF)
        if total < best_cost:
            best, best_cost = terminal, total

    if best is None:
        return None  # merge() retries with issue-order constraints
    return _walk_back(parent, best, seg_ids)


def merge_variant(commands, *, greedy=False, interleaved=None, **options):
    """merger.merge with the greedy baseline or the interleaving bit fixed to True/False."""
    if interleaved is not None and greedy:
        raise ValueError("a fixed interleaving bit requires exact scheduling")
    if interleaved is not None:
        lens = [len(_decompose(c, cmd)) for c, cmd in enumerate(commands)]
        if not any(lens):
            if interleaved:
                raise NoInterleavingSchedule("an empty batch cannot interleave")
        elif (2 if interleaved else 1) * _state_count(lens) > _STATE_CAP:
            raise ValueError("exact interleaving search exceeds the state bound")
    found = []

    def scheduler(*args, **kwargs):
        schedule = variant_schedule(*args, greedy=greedy, interleaved=interleaved, **kwargs)
        found.append(schedule is not None)
        return schedule

    plan = merge(commands, scheduler=scheduler, **options)
    if interleaved is not None and found and not any(found):
        raise NoInterleavingSchedule(f"no feasible schedule with interleaved={interleaved}")
    return plan

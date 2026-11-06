"""Patrol route ordering over tagged viewpoints (Euclidean, grouped by area)."""

from __future__ import annotations

import math

from .surfaces import Viewpoint

# Larger groups fall back to nearest-neighbour (Held-Karp is 2^n)
MAX_EXACT_POINTS = 16


def _distance(a: Viewpoint, b: Viewpoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _nearest_neighbour(points: list[Viewpoint], start: tuple[float, float]) -> list[Viewpoint]:
    remaining = list(points)
    order: list[Viewpoint] = []
    cx, cy = start
    while remaining:
        nxt = min(remaining, key=lambda v: v.distance_to(cx, cy))
        remaining.remove(nxt)
        order.append(nxt)
        cx, cy = nxt.x, nxt.y
    return order


def _held_karp(points: list[Viewpoint], start: tuple[float, float]) -> list[Viewpoint]:
    """Shortest open path from `start` through every point, exact (O(2^n * n^2))."""
    n = len(points)
    if n < 2:
        return list(points)
    dist = [[_distance(a, b) for b in points] for a in points]
    # best[mask][last] = (length, previous)
    best = [[(math.inf, -1)] * n for _ in range(1 << n)]
    for i, p in enumerate(points):
        best[1 << i][i] = (p.distance_to(*start), -1)
    for mask in range(1, 1 << n):
        for last in range(n):
            length = best[mask][last][0]
            if length == math.inf:
                continue
            for nxt in range(n):
                if mask & (1 << nxt):
                    continue
                cand = length + dist[last][nxt]
                nmask = mask | (1 << nxt)
                if cand < best[nmask][nxt][0]:
                    best[nmask][nxt] = (cand, last)
    mask = (1 << n) - 1
    last = min(range(n), key=lambda i: best[mask][i][0])
    order = []
    while last != -1:
        order.append(points[last])
        mask, last = mask ^ (1 << last), best[mask][last][1]
    return order[::-1]


def _order_group(points: list[Viewpoint], start: tuple[float, float]) -> list[Viewpoint]:
    if len(points) <= MAX_EXACT_POINTS:
        return _held_karp(points, start)
    return _nearest_neighbour(points, start)


def route_length(order: list[Viewpoint], start: tuple[float, float]) -> float:
    """Total Euclidean path length from `start` through every viewpoint."""
    if not order:
        return 0.0
    total = order[0].distance_to(*start)
    for a, b in zip(order, order[1:]):
        total += _distance(a, b)
    return total


def order_route(
    viewpoints: list[Viewpoint],
    start: tuple[float, float],
    group_by_area: bool = True,
) -> tuple[list[Viewpoint], float]:
    """Order `viewpoints` into a patrol route starting at `start`.

    With `group_by_area` each area is finished before the next (nearest first).
    Returns ``(ordered, total_length)``.
    """
    if not viewpoints:
        return [], 0.0

    if not group_by_area:
        order = _order_group(viewpoints, start)
        return order, route_length(order, start)

    by_area: dict[str, list[Viewpoint]] = {}
    for vp in viewpoints:
        by_area.setdefault(vp.area, []).append(vp)

    ordered: list[Viewpoint] = []
    cursor = start
    pending = dict(by_area)
    while pending:
        area = min(
            pending,
            key=lambda a: min(v.distance_to(*cursor) for v in pending[a]),
        )
        group = _order_group(pending.pop(area), cursor)
        ordered.extend(group)
        cursor = (group[-1].x, group[-1].y)

    return ordered, route_length(ordered, start)


def order_by_staleness(
    viewpoints: list[Viewpoint],
    last_scanned: dict[str, float],
    now: float,
    start: tuple[float, float],
    never_scanned_age: float = 1e9,
) -> tuple[list[Viewpoint], float]:
    """Revisit order: stalest first, ties broken by distance from `start`.

    `last_scanned` maps ``"area/name"`` to its last scan time; missing = never scanned.
    """
    def age(vp: Viewpoint) -> float:
        stamp = last_scanned.get(vp.key)
        return never_scanned_age if stamp is None else now - stamp

    ordered = sorted(viewpoints, key=lambda v: (-age(v), v.distance_to(*start)))
    return ordered, route_length(ordered, start)

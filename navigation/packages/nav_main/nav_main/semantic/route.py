"""Patrol route ordering over tagged viewpoints.

Nearest-neighbour seed plus 2-opt refinement on Euclidean distances, grouped by
area so the robot finishes a room before crossing a doorway (door crossings are
the expensive part, and Euclidean distance under-counts them).

Euclidean is deliberate: an arena has ~20 viewpoints, and `NavQuery`
(`ComputePathToPose`, `TIMEOUT_NAV_QUERY = 10 s`) is far too slow to rank that
many candidates. Validate the chosen route with `NavQuery` if an exact length is
needed.
"""

from __future__ import annotations

import math

from .surfaces import Viewpoint


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


def _two_opt(order: list[Viewpoint], start: tuple[float, float], max_passes: int = 20) -> list[Viewpoint]:
    """Classic 2-opt on an open path that begins at `start`."""
    if len(order) < 4:
        return order

    def leg(i: int, j: int) -> float:
        """Distance between consecutive stops, where index -1 is `start`."""
        a = order[i] if i >= 0 else None
        b = order[j]
        return _distance(a, b) if a is not None else b.distance_to(*start)

    for _ in range(max_passes):
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                before = leg(i - 1, i) + (
                    _distance(order[j], order[j + 1]) if j + 1 < len(order) else 0.0
                )
                after = leg(i - 1, j) + (
                    _distance(order[i], order[j + 1]) if j + 1 < len(order) else 0.0
                )
                if after + 1e-9 < before:
                    order[i : j + 1] = reversed(order[i : j + 1])
                    improved = True
        if not improved:
            break
    return order


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

    With `group_by_area` every area is finished before moving to the next one;
    the areas themselves are visited nearest-first from `start`.
    Returns ``(ordered, total_euclidean_length)``.
    """
    if not viewpoints:
        return [], 0.0

    if not group_by_area:
        order = _two_opt(_nearest_neighbour(viewpoints, start), start)
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
        group = _two_opt(_nearest_neighbour(pending.pop(area), cursor), cursor)
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

    `last_scanned` maps ``"area/name"`` to the timestamp of the last scan; a
    missing key means never scanned, which sorts first.
    """
    def age(vp: Viewpoint) -> float:
        stamp = last_scanned.get(vp.key)
        return never_scanned_age if stamp is None else now - stamp

    ordered = sorted(viewpoints, key=lambda v: (-age(v), v.distance_to(*start)))
    return ordered, route_length(ordered, start)

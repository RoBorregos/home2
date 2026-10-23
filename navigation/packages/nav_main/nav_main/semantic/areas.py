"""Point -> room / furniture lookup over the active map's areas JSON.

The single implementation behind the ``GetAreaForPoint`` service. It lives here,
outside the node, so it can be tested offline against the real
``areas_<MAP_NAME>.json`` and reused by the tagger — the repo already carries
three divergent copies of point-in-polygon (`area_check.py:60`,
`point_transformer.py:182`, and this one), and only one of them reads the map
that is actually loaded.
"""

from __future__ import annotations

import math

# Not placement sublocations: the room outline and nav's own standoff pose.
NON_SUBLOCATION_KEYS = ("polygon", "safe_place")
META_SUFFIX = "_meta"
DEFAULT_SUBLOCATION_DISTANCE = 1.5


def point_in_polygon(x: float, y: float, polygon) -> bool:
    """Ray casting: is (x, y) inside `polygon` (a list of [x, y] vertices)?"""
    if not polygon or len(polygon) < 3:
        return False
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0][0], polygon[0][1]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n][0], polygon[i % n][1]
        if min(p1y, p2y) < y <= max(p1y, p2y) and x <= max(p1x, p2x):
            if p1y != p2y:
                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                if p1x == p2x or x <= xinters:
                    inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def areas_containing(areas_data: dict, x: float, y: float) -> list[str]:
    """Every area whose polygon contains (x, y).

    Areas without a polygon — ``start_area``, ``start_location``, ``entrance``,
    ``exit``, ``inspection_point`` — are referee waypoints, not rooms, and can
    never match.
    """
    hits = []
    for name, data in (areas_data or {}).items():
        if not isinstance(data, dict):
            continue
        if point_in_polygon(x, y, data.get("polygon") or []):
            hits.append(name)
    return hits


def nearest_sublocations(
    areas_data: dict,
    area: str,
    x: float,
    y: float,
    max_distance: float = DEFAULT_SUBLOCATION_DISTANCE,
) -> tuple[list[str], list[float]]:
    """(names, distances) of `area`'s sublocations, nearest first, within range."""
    ranked = []
    for name, pose in (areas_data or {}).get(area, {}).items():
        if name in NON_SUBLOCATION_KEYS or name.endswith(META_SUFFIX):
            continue
        if not isinstance(pose, (list, tuple)) or len(pose) < 2:
            continue
        try:
            distance = math.hypot(float(pose[0]) - x, float(pose[1]) - y)
        except (TypeError, ValueError):
            continue
        if distance <= max_distance:
            ranked.append((distance, name))
    ranked.sort()
    return [name for _, name in ranked], [float(d) for d, _ in ranked]


def area_for_point(
    areas_data: dict,
    x: float,
    y: float,
    max_distance: float = DEFAULT_SUBLOCATION_DISTANCE,
) -> tuple[str, list[str], list[float], bool]:
    """(area, sublocations, distances, in_house) for a map-frame point.

    `area` is ``""`` when the point falls in no polygon, which is a normal answer:
    the polygons do not tile the map. Overlapping polygons are broken by the
    nearest sublocation.
    """
    hits = areas_containing(areas_data, x, y)
    in_house = bool(hits)
    if not hits:
        return "", [], [], False
    if len(hits) > 1:
        hits.sort(
            key=lambda a: (nearest_sublocations(areas_data, a, x, y, float("inf"))[1] or [1e9])[0]
        )
    area = hits[0]
    names, distances = nearest_sublocations(areas_data, area, x, y, max_distance)
    return area, names, distances, in_house

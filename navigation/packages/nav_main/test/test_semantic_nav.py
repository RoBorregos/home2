"""Unit tests for nav_main.semantic (pure Python, no ROS needed).

Run: python3 -m pytest navigation/packages/nav_main/test/test_semantic_nav.py
"""

import glob
import json
import math
import os
import sys

import pytest

NAV_MAIN = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, NAV_MAIN)

from nav_main.semantic.route import (  # noqa: E402
    order_by_staleness,
    order_route,
    route_length,
)
from nav_main.semantic.staleness import ScanLog, angle_difference  # noqa: E402
from nav_main.semantic.surfaces import (  # noqa: E402
    Viewpoint,
    classify,
    load_viewpoints,
)
from nav_main.semantic.viewpoints import (  # noqa: E402
    Grid,
    is_area_free,
    validate_viewpoints,
)

AREAS_DIR = os.path.join(NAV_MAIN, "..", "map_context", "maps", "areas")

RADIUS = 1.0
YAW_TOL = math.radians(35.0)
STILL = 0.15
FRACTION = 0.7


def vp(name="table", area="kitchen", x=0.0, y=0.0, yaw=0.0, dwell_s=2.5):
    return Viewpoint(
        area=area,
        name=name,
        x=x,
        y=y,
        qz=math.sin(yaw / 2.0),
        qw=math.cos(yaw / 2.0),
        dwell_s=dwell_s,
    )


def observe(log, viewpoints, x, y, yaw, stamp):
    return log.observe(viewpoints, x, y, yaw, stamp, RADIUS, YAW_TOL, STILL, FRACTION)


def grid_with_block(x0, x1, y0, y1, size=10.0, res=0.05, origin=-5.0):
    n = int(size / res)
    data = [0] * (n * n)
    for row in range(n):
        for col in range(n):
            wx, wy = origin + col * res, origin + row * res
            if x0 <= wx <= x1 and y0 <= wy <= y1:
                data[row * n + col] = 100
    return Grid(data, n, n, res, origin, origin)


# ------------------------------------------------------------------- surfaces


@pytest.mark.parametrize(
    "name,expected",
    [
        ("bedside_table", "low_surface"),  # before the generic "table" rule
        ("dining_table", "surface"),
        ("bookshelf", "shelf"),
        ("trash_bin", "floor"),
        ("dishwasher", "appliance"),
        ("house_entry", "waypoint"),
        ("mystery_object", "unknown"),
    ],
)
def test_classify(name, expected):
    assert classify(name) == expected


def test_load_viewpoints_skips_non_furniture_and_areas_without_polygon():
    pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    areas = {
        "kitchen": {
            "polygon": [[0, 0], [1, 0], [1, 1]],
            "safe_place": pose,
            "table": pose,
            "entrance": pose,
            "malformed": [1.0, 2.0],
        },
        "start_area": {"table": pose},
    }
    assert [v.key for v in load_viewpoints(areas)] == ["kitchen/table"]


def test_load_viewpoints_filters_by_area_ignoring_case():
    pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    poly = [[0, 0], [1, 0], [1, 1]]
    areas = {
        "kitchen": {"polygon": poly, "table": pose},
        "bedroom": {"polygon": poly, "bed": pose},
    }
    assert [v.area for v in load_viewpoints(areas, areas=["KITCHEN "])] == ["kitchen"]


@pytest.mark.parametrize(
    "path", sorted(glob.glob(os.path.join(AREAS_DIR, "areas_*.json")))
)
def test_every_committed_map_yields_viewpoints(path):
    with open(path) as handle:
        viewpoints = load_viewpoints(json.load(handle))
    assert viewpoints, f"{os.path.basename(path)} has no patrol viewpoints"
    for v in viewpoints:
        assert math.isclose(v.qz**2 + v.qw**2, 1.0, abs_tol=0.01), v.key


# ---------------------------------------------------------------------- route


def test_route_visits_every_point_shortest_first():
    points = [vp("c", x=3.0), vp("a", x=1.0), vp("b", x=2.0)]
    ordered, total = order_route(points, (0.0, 0.0))
    assert [v.name for v in ordered] == ["a", "b", "c"]
    assert total == pytest.approx(3.0)


def test_route_finishes_an_area_before_the_next():
    points = [
        vp("k1", area="kitchen", x=1.0),
        vp("b1", area="bedroom", x=2.0),
        vp("k2", area="kitchen", x=3.0),
    ]
    ordered, _ = order_route(points, (0.0, 0.0))
    assert [v.area for v in ordered] == ["kitchen", "kitchen", "bedroom"]


def test_route_empty_and_large_group_fallback():
    assert order_route([], (0.0, 0.0)) == ([], 0.0)
    many = [vp(f"p{i}", x=float(i)) for i in range(20)]  # past the Held-Karp limit
    ordered, total = order_route(many, (0.0, 0.0))
    assert len(ordered) == 20
    assert total == pytest.approx(19.0)


def test_staleness_order_puts_never_scanned_first_and_fresh_last():
    points = [vp("fresh", x=1.0), vp("never", x=9.0), vp("old", x=2.0)]
    scans = {"kitchen/fresh": 95.0, "kitchen/old": 10.0}
    ordered, total = order_by_staleness(points, scans, 100.0, (0.0, 0.0))
    assert [v.name for v in ordered] == ["never", "old", "fresh"]
    assert total == pytest.approx(route_length(ordered, (0.0, 0.0)))


# ---------------------------------------------------------------- scan marking


def test_stopping_for_the_dwell_marks_the_viewpoint_once():
    log, target = ScanLog(), vp()
    assert observe(log, [target], 0.0, 0.0, 0.0, 10.0) is None
    assert observe(log, [target], 0.0, 0.0, 0.0, 11.0) is None
    assert observe(log, [target], 0.0, 0.0, 0.0, 11.75) == "kitchen/table"
    assert log.scans["kitchen/table"] == 11.75
    assert log.dirty

    # Still there: no new report, but the timestamp keeps refreshing.
    assert observe(log, [target], 0.0, 0.0, 0.0, 15.0) is None
    assert log.scans["kitchen/table"] == 15.0


def test_driving_through_the_radius_does_not_mark():
    """2 m across the scan radius at 0.5 m/s: 4 s in range, longer than the dwell."""
    log, target = ScanLog(), vp()
    for tick in range(17):
        t = tick * 0.25
        x = -1.0 + 0.5 * t
        assert observe(log, [target], x, 0.0, 0.0, 100.0 + t) is None
    assert log.scans == {}


def test_caller_waiting_exactly_the_dwell_gets_the_mark():
    """Stops between ticks, waits dwell_s, leaves: the race the fraction exists for."""
    log, target = ScanLog(), vp(dwell_s=2.5)
    period, arrival, marked = 0.25, 10.0, None
    t = arrival + period  # worst case: the first tick lands a full period late
    while t < arrival + target.dwell_s:
        marked = observe(log, [target], 0.0, 0.0, 0.0, t) or marked
        t += period
    assert marked == "kitchen/table"


def test_wrong_heading_or_out_of_range_never_marks():
    log, target = ScanLog(), vp()
    for t in range(10):
        observe(log, [target], 0.0, 0.0, math.pi, float(t))
        observe(log, [target], 5.0, 0.0, 0.0, float(t))
    assert log.scans == {}


def test_leaving_restarts_the_timer():
    log, target = ScanLog(), vp()
    observe(log, [target], 0.0, 0.0, 0.0, 0.0)
    observe(log, [target], 5.0, 0.0, 0.0, 1.0)  # left
    observe(log, [target], 0.0, 0.0, 0.0, 2.0)  # back
    assert observe(log, [target], 0.0, 0.0, 0.0, 3.0) is None
    assert observe(log, [target], 0.0, 0.0, 0.0, 3.75) == "kitchen/table"


def test_localization_jitter_does_not_restart_the_timer():
    log, target = ScanLog(), vp()
    observe(log, [target], 0.0, 0.0, 0.0, 0.0)
    observe(log, [target], 0.05, -0.05, 0.0, 1.0)
    assert observe(log, [target], -0.05, 0.05, 0.0, 1.75) == "kitchen/table"


def test_closest_viewpoint_wins_when_two_are_in_range():
    log = ScanLog()
    near, far = vp("near", x=0.1), vp("far", x=0.6)
    observe(log, [far, near], 0.0, 0.0, 0.0, 0.0)
    assert observe(log, [far, near], 0.0, 0.0, 0.0, 2.0) == "kitchen/near"
    assert set(log.scans) == {"kitchen/near"}


def test_clock_jumping_back_restarts_instead_of_stalling():
    log, target = ScanLog(), vp()
    observe(log, [target], 0.0, 0.0, 0.0, 100.0)
    observe(log, [target], 0.0, 0.0, 0.0, 1.0)
    assert observe(log, [target], 0.0, 0.0, 0.0, 3.0) == "kitchen/table"


def test_angle_difference_wraps():
    assert angle_difference(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(0.2)


# ---------------------------------------------------------------- persistence


def test_snapshot_roundtrip_adds_time_off(tmp_path):
    path = str(tmp_path / "nested" / "staleness.json")
    log = ScanLog(map_name="lab", path=path)
    log.mark("kitchen/table", 90.0)
    log.save(now=100.0, wall_now=1000.0)
    assert not log.dirty

    # Rebooted: the ROS clock restarted and 50 s passed on the wall clock.
    restored = ScanLog(map_name="lab", path=path)
    count, why = restored.load(now=5.0, wall_now=1050.0)
    assert (count, why) == (1, "")
    assert restored.age("kitchen/table", 5.0) == pytest.approx(60.0)


def test_snapshot_from_another_map_is_rejected(tmp_path):
    path = str(tmp_path / "staleness.json")
    log = ScanLog(map_name="lab", path=path)
    log.mark("kitchen/table", 90.0)
    log.save(now=100.0)
    count, why = ScanLog(map_name="arena", path=path).load(now=0.0)
    assert count == 0
    assert "lab" in why


def test_missing_or_corrupt_snapshot_is_not_fatal(tmp_path):
    path = tmp_path / "staleness.json"
    assert ScanLog(path=str(path)).load(now=0.0) == (0, "no snapshot file")
    path.write_text("{not json")
    count, why = ScanLog(path=str(path)).load(now=0.0)
    assert count == 0
    assert "unreadable" in why


def test_never_scanned_age_is_infinite():
    assert ScanLog().age("kitchen/table", 10.0) == float("inf")


# ----------------------------------------------------------- costmap validation


def test_grid_lookup_and_bounds():
    grid = grid_with_block(0.0, 1.0, 0.0, 1.0)
    assert grid.cost_at(0.5, 0.5) == 100
    assert grid.cost_at(-2.0, -2.0) == 0
    assert grid.cost_at(50.0, 50.0) is None
    assert not is_area_free(grid, 0.5, 0.5)
    assert is_area_free(grid, -2.0, -2.0)


def test_no_costmap_keeps_every_pose():
    points = [vp("a"), vp("b")]
    report = validate_viewpoints(points, None)
    assert report.kept == points
    assert not report.relocated and not report.dropped


def test_free_pose_is_left_untouched():
    point = vp(x=-3.0, y=-3.0)
    report = validate_viewpoints([point], grid_with_block(0.0, 1.0, 0.0, 1.0))
    assert report.kept == [point]
    assert not report.relocated


def test_blocked_pose_moves_to_free_cell_still_facing_the_furniture():
    point = vp(x=0.0, y=0.0, yaw=0.0)  # furniture 0.8 m ahead, at (0.8, 0)
    grid = grid_with_block(-0.3, 0.3, -0.3, 0.3)
    report = validate_viewpoints([point], grid)

    assert [key for key, _ in report.relocated] == ["kitchen/table"]
    moved = report.kept[0]
    assert is_area_free(grid, moved.x, moved.y)
    bearing = math.atan2(0.0 - moved.y, 0.8 - moved.x)
    assert angle_difference(moved.yaw, bearing) < 0.01


def test_pose_with_nowhere_to_go_is_dropped():
    report = validate_viewpoints([vp()], grid_with_block(-5.0, 5.0, -5.0, 5.0))
    assert report.kept == []
    assert report.dropped == ["kitchen/table"]

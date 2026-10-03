"""Unit tests for nav_main.approach_planner (pure Python, no ROS needed).

Run: python3 -m pytest navigation/packages/nav_main/test/test_approach_planner.py
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nav_main import approach_planner as ap  # noqa: E402

FOOTPRINT = [[0.325, 0.25], [0.325, -0.25], [-0.325, -0.25], [-0.325, 0.25]]
FRONT, _, HALF_W = ap.footprint_extent(FOOTPRINT)
TABLE = ap.ReachProfile(gap=0.03, gap_min=0.02, gap_max=0.15)


def empty_grid(size=8.0, res=0.05, origin=(-4.0, -4.0)):
    n = int(size / res)
    return ap.Grid(np.zeros(n * n, dtype=np.int16), n, n, res, *origin)


def fill_box(grid, x0, y0, x1, y1, cost=ap.LETHAL):
    xs = np.arange(x0, x1, grid.resolution / 2)
    ys = np.arange(y0, y1, grid.resolution / 2)
    for x in xs:
        for y in ys:
            mx = int((x - grid.origin_x) / grid.resolution)
            my = int((y - grid.origin_y) / grid.resolution)
            grid.cells[my, mx] = cost


def test_occupancy_to_cost_matches_nav2_table():
    out = ap.occupancy_to_cost([0, 1, 50, 98, 99, 100, -1])
    assert list(out[[0, 4, 5, 6]]) == [0, ap.INSCRIBED, ap.LETHAL, ap.NO_INFORMATION]
    assert 1 <= out[1] < out[2] < out[3] <= 252


def test_footprint_cost_sees_obstacle_under_corner_not_centre():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    # Small obstacle under the front-left corner only (centre cell stays free).
    fill_box(grid, 0.28, 0.20, 0.32, 0.24)
    assert ap.footprint_cost(grid, 0.0, 0.0, 0.0, samples) == ap.LETHAL
    assert grid.lookup(np.array([0.0]), np.array([0.0]))[0] == 0
    # Rotated 90 deg the corner moves away from it.
    assert ap.footprint_cost(grid, 0.0, -0.5, 0.0, samples) == 0


def test_footprint_clearance_signs():
    pts = np.array([[0.425, 0.0], [0.0, 0.0], [0.425, 0.35]])
    d = ap.footprint_clearance(pts, FOOTPRINT)
    assert d[0] == pytest.approx(0.10, abs=1e-6)
    assert d[1] < 0
    assert d[2] == pytest.approx(math.hypot(0.1, 0.1), abs=1e-6)


def test_line_face_prefers_target_and_preferred_gap():
    # Face along y at x=1.0 (robot at origin looking +x); target at y=0.4.
    p1, p2, n = (1.0, -0.6), (1.0, 0.6), (1.0, 0.0)
    cands = ap.line_face_candidates(p1, p2, n, FRONT, HALF_W, TABLE, target=(1.2, 0.4))
    best = ap.rank(cands, TABLE, robot=(0.0, 0.0))[0][0]
    assert best.y == pytest.approx(0.4, abs=0.03)
    assert best.gap == pytest.approx(TABLE.gap)
    assert best.x == pytest.approx(1.0 - FRONT - TABLE.gap, abs=1e-6)
    assert best.yaw == pytest.approx(0.0, abs=1e-6)


def test_line_face_avoids_chair_by_shifting_laterally():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    p1, p2, n = (1.0, -1.0), (1.0, 1.0), (1.0, 0.0)
    # Table cells (the face itself) are ignored; a chair sits right where the
    # target-aligned pose would be.
    fill_box(grid, 1.0, -1.0, 1.8, 1.0)
    fill_box(grid, 0.30, -0.15, 0.50, 0.15)
    ignore = ap.halfplane_ignore(p1, n, margin=0.03)
    cands = ap.line_face_candidates(p1, p2, n, FRONT, HALF_W, TABLE, target=(1.2, 0.0))
    valid, all_c = ap.rank(cands, TABLE, grid=grid, samples=samples, robot=(0.0, 0.0),
                           ignore=ignore)
    assert valid, "a free pose next to the chair must exist"
    best = valid[0]
    assert abs(best.y) >= 0.15 + HALF_W - 0.03  # footprint clears the chair
    assert best.lateral < 0.6
    assert any(not c.valid for c in all_c)


def test_face_ignore_hides_only_the_surface():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    p1, n = (1.0, -1.0), (1.0, 0.0)
    fill_box(grid, 1.0, -1.0, 1.8, 1.0)          # the table being docked at
    ignore = ap.halfplane_ignore(p1, n, margin=0.03)
    x = 1.0 - FRONT - 0.03                        # docked at a 3 cm gap
    assert ap.footprint_cost(grid, x, 0.0, 0.0, samples, ignore) == 0
    fill_box(grid, 0.80, 0.10, 0.90, 0.20)        # chair leg in front of the table
    assert ap.footprint_cost(grid, x, 0.0, 0.0, samples, ignore) == ap.LETHAL


def test_circle_face_faces_centre_and_follows_target():
    prof = ap.ReachProfile(0.03, 0.02, 0.15, "circle")
    centre, r = (0.0, 0.0), 0.46
    cands = ap.circle_face_candidates(centre, r, FRONT, prof, ref_angle=math.pi / 2,
                                      target=(0.3, 0.0))
    best = ap.rank(cands, prof, robot=(0.0, 1.5))[0][0]
    # Target is east of the centre -> stand east, facing west.
    assert best.extra["angle"] == pytest.approx(0.0, abs=math.radians(10))
    assert math.hypot(best.x, best.y) == pytest.approx(r + prof.gap + FRONT, abs=1e-6)
    assert abs(math.cos(best.yaw) + 1.0) < 0.02


def test_ring_candidates_skip_blocked_side():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    prof = ap.ReachProfile(0.65, 0.5, 0.95)
    # Wall between the robot side and the target.
    fill_box(grid, -1.0, -0.9, 1.0, -0.4)
    cands = ap.ring_candidates((0.0, 0.0), (0.0, -2.0), 0.5, 0.65, 0.95)
    valid, _ = ap.rank(cands, prof, grid=grid, samples=samples, robot=(0.0, -2.0))
    best = valid[0]
    assert ap.footprint_cost(grid, best.x, best.y, best.yaw, samples) < ap.LETHAL
    # Facing the target.
    assert math.atan2(-best.y, -best.x) == pytest.approx(best.yaw, abs=1e-6)


def test_ranking_is_deterministic():
    p1, p2, n = (1.0, -0.6), (1.0, 0.6), (1.0, 0.0)
    a = ap.rank(ap.line_face_candidates(p1, p2, n, FRONT, HALF_W, TABLE), TABLE)[0]
    b = ap.rank(ap.line_face_candidates(p1, p2, n, FRONT, HALF_W, TABLE), TABLE)[0]
    assert [(c.x, c.y) for c in a] == [(c.x, c.y) for c in b]


def test_load_profiles():
    profs = ap.load_profiles({"table": {"gap": 0.03, "gap_min": 0.02, "gap_max": 0.15}})
    assert profs["table"].shape == "line" and profs["table"].clamp(0.5) == 0.15


def test_padding_keeps_margin_from_side_obstacles():
    grid = empty_grid()
    fill_box(grid, 0.0, 0.31, 0.05, 0.34)  # next cell beside the footprint's left edge
    bare = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    padded = ap.footprint_samples(FOOTPRINT, grid.resolution / 2, padding=0.06)
    assert ap.footprint_cost(grid, 0.0, 0.0, 0.0, bare) == 0
    assert ap.footprint_cost(grid, 0.0, 0.0, 0.0, padded) == ap.LETHAL
    assert ap.pad_footprint(FOOTPRINT, 0.05)[0] == pytest.approx([0.375, 0.30])


def test_load_weights_per_use():
    w = ap.load_weights({"dock": {"cost": 0.2}, "point": {"lateral": 0.0}}, "dock")
    assert w.cost == 0.2 and w.lateral == ap.Weights().lateral


def test_astar_routes_around_chair_legs_with_full_footprint():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    # Two chair legs straight between start and goal.
    fill_box(grid, -0.25, 0.48, -0.20, 0.53)
    fill_box(grid, 0.20, 0.48, 0.25, 0.53)
    start, goal = (0.0, 0.0), (0.0, 1.2)
    straight_blocked = any(
        ap.footprint_cost(grid, 0.0, y, 0.0, samples) >= ap.LETHAL for y in np.linspace(0, 1.2, 25))
    assert straight_blocked
    path = ap.footprint_astar(grid, start, goal, 0.0, samples)
    assert path is not None and path[0] == pytest.approx(start) and path[-1] == pytest.approx(goal)
    for x, y in path:
        assert ap.footprint_cost(grid, x, y, 0.0, samples) < ap.LETHAL
    assert max(abs(x) for x, _ in path) > 0.4  # it really went around


def test_astar_none_when_goal_blocked():
    grid = empty_grid()
    samples = ap.footprint_samples(FOOTPRINT, grid.resolution / 2)
    fill_box(grid, -0.05, 1.15, 0.05, 1.25)
    assert ap.footprint_astar(grid, (0.0, 0.0), (0.0, 1.2), 0.0, samples) is None

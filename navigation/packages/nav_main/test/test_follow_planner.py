"""Unit tests for nav_main.follow_planner (pure Python, no ROS needed).

Run: python3 -m pytest navigation/packages/nav_main/test/test_follow_planner.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nav_main import approach_planner as ap  # noqa: E402
from nav_main import follow_planner as fp  # noqa: E402


def grid(size=6.0, res=0.05, origin=(-3.0, -3.0)):
    n = int(size / res)
    return ap.Grid(np.zeros(n * n, dtype=np.int16), n, n, res, *origin)


def fill_disc(g, cx, cy, r, cost):
    for my in range(g.height):
        for mx in range(g.width):
            x = g.origin_x + (mx + 0.5) * g.resolution
            y = g.origin_y + (my + 0.5) * g.resolution
            if math.hypot(x - cx, y - cy) <= r:
                g.cells[my, mx] = max(g.cells[my, mx], cost)


def test_lead_target_caps_spike():
    x, y = fp.lead_target(0.0, 0.0, 1.0, 0.0, 0.5)
    assert (x, y) == (0.5, 0.0)
    x, y = fp.lead_target(0.0, 0.0, 10.0, 0.0, 0.5, max_lead=1.0)
    assert math.isclose(x, 1.0) and y == 0.0


def test_follow_goal_in_front_and_behind():
    gx, gy, yaw = fp.follow_goal(0.0, 0.0, 3.0, 0.0, 0.8)
    assert math.isclose(gx, 2.2) and math.isclose(gy, 0.0) and math.isclose(yaw, 0.0)
    # too close: the goal lands behind the robot, still facing the person
    gx, gy, yaw = fp.follow_goal(0.0, 0.0, 0.4, 0.0, 0.8)
    assert gx < 0.0 and math.isclose(yaw, 0.0)
    assert fp.follow_goal(0.0, 0.0, 0.05, 0.0, 0.8) is None


def test_snap_along_line_pulls_goal_toward_robot():
    g = grid()
    # person at (2, 0) marked lethal with a 0.45 m inflated ring
    fill_disc(g, 2.0, 0.0, 0.65, ap.INSCRIBED)
    fill_disc(g, 2.0, 0.0, 0.2, ap.LETHAL)
    x, y, moved = fp.snap_along_line(g, 1.4, 0.0, 0.0, 0.0)
    assert moved > 0.0
    assert x < 1.4 and math.isclose(y, 0.0, abs_tol=1e-9)
    assert fp.cell_cost(g, x, y) < ap.INSCRIBED
    # already fine: untouched
    assert fp.snap_along_line(g, 0.5, 0.0, 0.0, 0.0) == (0.5, 0.0, 0.0)
    # no grid: untouched
    assert fp.snap_along_line(None, 1.4, 0.0, 0.0, 0.0) == (1.4, 0.0, 0.0)


def test_snap_along_line_gives_up_past_max_back():
    g = grid()
    fill_disc(g, 0.0, 0.0, 3.0, ap.LETHAL)
    assert fp.snap_along_line(g, 1.0, 0.0, 0.0, 0.0, max_back=0.5) == (1.0, 0.0, 0.0)


def test_snap_spiral_finds_nearest_free():
    g = grid()
    fill_disc(g, 0.0, 0.0, 0.3, ap.LETHAL)
    x, y = fp.snap_spiral(g, 0.0, 0.0, 20)
    assert 0.25 <= math.hypot(x, y) <= 0.45
    assert fp.cell_cost(g, x, y) < ap.INSCRIBED


def test_search_yaws_fans_out_from_travel_direction():
    yaws = [math.degrees(y) for y in fp.search_yaws(0.0)]
    assert [round(v) for v in yaws] == [0, 60, -60, 120, -120, 180]
    yaws = fp.search_yaws(math.radians(170.0), 90.0)
    assert all(-math.pi < y <= math.pi for y in yaws)
    assert math.isclose(yaws[0], math.radians(170.0))


def test_lost_goal_faces_travel_direction_when_moving():
    gx, gy, yaw = fp.lost_goal(0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.8)
    assert math.isclose(gx, 2.2) and math.isclose(yaw, math.pi / 2)
    # standing still: face them
    gx, gy, yaw = fp.lost_goal(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.8)
    assert math.isclose(yaw, 0.0)
    # already next to them: hold position
    gx, gy, yaw = fp.lost_goal(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.8)
    assert (gx, gy) == (0.0, 0.0)


def test_update_velocity_resets_on_gap_and_clamps():
    vx, vy = fp.update_velocity(1.0, 0.0, (0.0, 0.0), (0.1, 0.0), 0.1, 2.0)
    assert math.isclose(vx, 0.4 * 1.0 + 0.6 * 1.0) and vy == 0.0
    assert fp.update_velocity(1.0, 1.0, (0.0, 0.0), (5.0, 0.0), 0.6, 2.0) == (0.0, 0.0)
    vx, vy = fp.update_velocity(0.0, 0.0, (0.0, 0.0), (5.0, 0.0), 0.1, 2.0)
    assert math.isclose(vx, 0.8)  # 50 m/s clamped to 2 m/s, then 0.4 gain

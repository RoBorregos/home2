"""Pure geometry for person following (no ROS), used by person_goal_smoother.py.

Everything here is a function of plain numbers and an approach_planner.Grid so it
can be unit-tested offline (test/test_follow_planner.py). The smoother node owns
the ROS side: TF, timers, the follow-mode param switch and the status topic.

Conventions: positions are map-frame (x, y) metres, yaws are radians.
"""

import math

from nav_main.approach_planner import INSCRIBED, LETHAL

# Costmap cost at or above this is not a valid goal.
MAX_GOAL_COST = INSCRIBED

# Mirrors frida_constants FOLLOW_STATUS_* (no ROS import needed).
FOLLOWING = "following"
LOST = "lost"
SEARCHING = "searching"
GAVE_UP = "gave_up"
IDLE = "idle"


def lead_target(px, py, vx, vy, lead_time, max_lead=1.0):
    """Where the person will be in lead_time seconds, capped at max_lead metres.

    A walking person stays ahead of the goal by (speed x pipeline latency), and
    turns corners out of the camera view; aiming ahead cuts that lag. The cap
    keeps a velocity spike (tracker jitter) from throwing the goal into a wall.
    """
    dx, dy = vx * lead_time, vy * lead_time
    d = math.hypot(dx, dy)
    if d > max_lead:
        dx, dy = dx / d * max_lead, dy / d * max_lead
    return px + dx, py + dy


def follow_goal(rx, ry, tx, ty, follow_distance, min_dist=0.1):
    """Goal pose (x, y, yaw) follow_distance short of the target on the
    target->robot line, facing the target. None if the robot is on top of it.

    dist > follow_distance -> the goal is between them: the robot drives forward.
    dist < follow_distance -> the goal is BEHIND the robot: it backs up to open
    the gap instead of spinning in place while crowded.
    """
    dx, dy = tx - rx, ty - ry
    dist = math.hypot(dx, dy)
    if dist < min_dist:
        return None
    ux, uy = dx / dist, dy / dist
    gx, gy = tx - follow_distance * ux, ty - follow_distance * uy
    return gx, gy, math.atan2(ty - gy, tx - gx)


def cell_cost(grid, x, y):
    """Costmap value at a world point, or None off-grid / unknown."""
    if grid is None:
        return None
    v = int(grid.lookup([x], [y])[0])
    return None if v < 0 else v


def snap_along_line(grid, gx, gy, rx, ry, max_back=1.0, step=0.05,
                    max_cost=MAX_GOAL_COST):
    """Pull a goal that lands in (inscribed/lethal) cost back toward the robot.

    The person is a lidar obstacle: the goal a follow_distance short of them
    often sits inside the inflated ring around their legs. Walking back along
    the goal->robot line keeps the goal on the robot's side of the person (a
    spiral search would happily pick a cell behind them). Returns (x, y,
    moved_m); unchanged when the goal is already acceptable, there is no grid,
    or nothing acceptable lies within max_back.
    """
    cost = cell_cost(grid, gx, gy)
    if cost is None or cost < max_cost:
        return gx, gy, 0.0
    dx, dy = rx - gx, ry - gy
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return gx, gy, 0.0
    ux, uy = dx / d, dy / d
    s = step
    while s <= max_back + 1e-9:
        x, y = gx + ux * s, gy + uy * s
        c = cell_cost(grid, x, y)
        if c is None or c < max_cost:
            return x, y, s
        s += step
    return gx, gy, 0.0


def snap_spiral(grid, gx, gy, radius_cells, max_cost=MAX_GOAL_COST):
    """Nearest acceptable cell by ring search (fallback when the line snap fails).
    Returns (x, y) — the input when nothing is found."""
    if grid is None:
        return gx, gy
    res = grid.resolution
    mx = int((gx - grid.origin_x) / res)
    my = int((gy - grid.origin_y) / res)
    if 0 <= mx < grid.width and 0 <= my < grid.height and grid.cells[my, mx] < max_cost:
        return gx, gy
    for r in range(1, radius_cells + 1):
        for ddx in range(-r, r + 1):
            for ddy in range(-r, r + 1):
                if abs(ddx) != r and abs(ddy) != r:
                    continue
                nx, ny = mx + ddx, my + ddy
                if 0 <= nx < grid.width and 0 <= ny < grid.height and grid.cells[ny, nx] < max_cost:
                    return nx * res + grid.origin_x + res / 2.0, ny * res + grid.origin_y + res / 2.0
    return gx, gy


def search_yaws(travel_yaw, step_deg=60.0):
    """Yaw sweep to look for a lost person, starting along their direction of
    travel and fanning out: 0, +60, -60, +120, -120, 180 (degrees, relative).

    The base turns in place through these while the arm camera is recentered,
    so the tracker's ReID gets a look in every direction the person could have
    gone. Returns absolute yaws in (-pi, pi]."""
    step = math.radians(step_deg)
    offsets = [0.0]
    k = 1
    while k * step < math.pi - 1e-9:
        offsets += [k * step, -k * step]
        k += 1
    offsets.append(math.pi)
    return [wrap_angle(travel_yaw + o) for o in offsets]


def wrap_angle(a):
    """Wrap to (-pi, pi] (a half turn stays +pi, not -pi)."""
    w = (a + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if math.isclose(w, -math.pi) else w


def lost_goal(rx, ry, px, py, vx, vy, follow_distance, moving_speed=0.3):
    """Goal for the moment the tracker goes silent: follow_distance short of the
    last-known position, facing their direction of travel if they were moving
    (regain line of sight around the corner they took), else facing them."""
    dx, dy = px - rx, py - ry
    dist = math.hypot(dx, dy)
    if dist > follow_distance:
        ux, uy = dx / dist, dy / dist
        gx, gy = px - follow_distance * ux, py - follow_distance * uy
    else:
        gx, gy = rx, ry
    if math.hypot(vx, vy) > moving_speed:
        yaw = math.atan2(vy, vx)
    else:
        yaw = math.atan2(py - gy, px - gx) if dist > 1e-3 else 0.0
    return gx, gy, yaw


def update_velocity(vx, vy, prev, cur, dt, max_speed, gain=0.4):
    """EMA person velocity from two smoothed positions. A stale gap resets it
    (a finite difference across it would spike the lead)."""
    if dt >= 0.5:
        return 0.0, 0.0
    if dt <= 0.02:
        return vx, vy
    nvx = (cur[0] - prev[0]) / dt
    nvy = (cur[1] - prev[1]) / dt
    speed = math.hypot(nvx, nvy)
    if speed > max_speed:
        nvx *= max_speed / speed
        nvy *= max_speed / speed
    return gain * nvx + (1.0 - gain) * vx, gain * nvy + (1.0 - gain) * vy


__all__ = [
    "LETHAL", "INSCRIBED", "MAX_GOAL_COST", "FOLLOWING", "LOST", "SEARCHING",
    "GAVE_UP", "IDLE", "lead_target", "follow_goal", "cell_cost",
    "snap_along_line", "snap_spiral", "search_yaws", "wrap_angle", "lost_goal",
    "update_velocity",
]

"""Base placement planning: where should the base stand to work at a surface/target?

Pure Python + numpy (no ROS), so nav_central and table_docker share it and it is
unit-testable. Instead of a hardcoded stand-off per location, the goal is a REGION
of candidate base poses (x, y, yaw) that are then scored (lowest cost wins):

  * reach      — the gap between the robot's front and the surface must sit inside
                 the arm's reach band [gap_min, gap_max] (a 1-D inverse
                 reachability map, after Vahrenkamp et al. ICRA 2013), preferring gap.
  * collision  — the FULL footprint is checked on the costmap, not just the centre
                 cell; poses whose footprint touches a lethal cell are rejected.
  * alignment  — stand in front of the object to manipulate (lateral error).
  * travel     — prefer poses close to where the robot already is.

The ranked list keeps the alternatives, so a caller can fall back to the next pose
when the first one fails (least-commitment, Stulp et al. "Action-Related Places").
Scoring navigation + manipulation costs together follows Reister et al. RA-L 2022.
"""

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

# nav2_costmap_2d cost values
LETHAL = 254
INSCRIBED = 253
NO_INFORMATION = 255  # costmap_2d raw; OccupancyGrid publishes it as -1


@dataclass
class ReachProfile:
    """Arm reach band for one surface type: gap = robot front -> surface (m)."""

    gap: float
    gap_min: float
    gap_max: float
    shape: str = "line"  # 'line' | 'circle' | 'auto'

    def clamp(self, gap):
        return min(max(gap, self.gap_min), self.gap_max)


@dataclass
class Weights:
    cost: float = 1.0  # footprint cost / 252
    reach: float = 1.0  # |gap - preferred| / band width
    lateral: float = 3.0  # m from the target along the surface
    travel: float = 0.3  # m from the current robot pose


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    gap: float = 0.0
    lateral: float = 0.0
    cost: int = 0
    travel: float = 0.0
    score: float = math.inf
    valid: bool = True
    reason: str = ""
    extra: dict = field(default_factory=dict)


class Grid:
    """Axis-aligned 2D cost grid (an OccupancyGrid / nav2 costmap in its own frame)."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        self.cells = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

    @classmethod
    def from_occupancy_grid(cls, msg):
        """nav_msgs/OccupancyGrid (as nav2 publishes costmaps) -> costmap units."""
        info = msg.info
        return cls(occupancy_to_cost(msg.data), info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y)

    def lookup(self, xs, ys):
        """Cell values at world points; out-of-grid -> -1 (unknown)."""
        mx = np.floor((np.asarray(xs) - self.origin_x) / self.resolution).astype(int)
        my = np.floor((np.asarray(ys) - self.origin_y) / self.resolution).astype(int)
        inside = (mx >= 0) & (mx < self.width) & (my >= 0) & (my < self.height)
        out = np.full(mx.shape, -1, dtype=np.int16)
        out[inside] = self.cells[my[inside], mx[inside]]
        return out


def occupancy_to_cost(values):
    """Invert nav2_costmap_2d's OccupancyGrid translation table (0 -> 0,
    1..252 -> 1..98, 253 -> 99, 254 -> 100, 255 -> -1) back to costmap units."""
    v = np.asarray(values, dtype=np.int32)
    cost = 1 + np.rint((v - 1) * 251.0 / 97.0).astype(np.int32)
    cost = np.where(v <= 0, 0, cost)
    cost = np.where(v == 99, INSCRIBED, cost)
    cost = np.where(v >= 100, LETHAL, cost)
    return np.where(v < 0, NO_INFORMATION, cost)


def footprint_extent(footprint):
    """(front, back, half_width) of a base_link footprint polygon."""
    fp = np.asarray(footprint, dtype=float)
    return float(fp[:, 0].max()), float(-fp[:, 0].min()), float(np.abs(fp[:, 1]).max())


def pad_footprint(footprint, padding):
    """Grow a (convex, base-centred) footprint outward by `padding` metres."""
    fp = np.asarray(footprint, dtype=float)
    if padding <= 0.0:
        return fp
    c = fp.mean(axis=0)
    return fp + np.sign(fp - c) * padding


def footprint_samples(footprint, step, padding=0.0):
    """Points covering the footprint polygon (base_link frame), boundary included.
    `padding` grows it first: a clearance margin from obstacles."""
    fp = pad_footprint(footprint, padding)
    xmin, ymin = fp.min(axis=0)
    xmax, ymax = fp.max(axis=0)
    xs = np.arange(xmin, xmax + step * 0.5, step)
    ys = np.arange(ymin, ymax + step * 0.5, step)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
    inside = points_in_polygon(pts, fp)
    # Always include the exact edges so thin obstacles on the boundary are caught.
    edge = []
    for i in range(len(fp)):
        a, b = fp[i], fp[(i + 1) % len(fp)]
        n = max(2, int(math.ceil(np.linalg.norm(b - a) / step)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            edge.append(a + t * (b - a))
    return np.vstack([pts[inside], np.asarray(edge)])


def points_in_polygon(pts, poly):
    """Even-odd rule; pts Nx2, poly Mx2."""
    x, y = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(pts), dtype=bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside


def transform_points(pts, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1],
                     y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def footprint_cost(grid, x, y, yaw, samples, ignore=None, unknown_cost=0):
    """Max cost under the footprint placed at (x, y, yaw). `samples` comes from
    footprint_samples(). `ignore(world_pts) -> bool mask` drops cells that belong to
    the surface being docked at (its own obstacle cells are expected to be close).
    Grid values must already be in costmap units (see occupancy_to_cost)."""
    world = transform_points(samples, x, y, yaw)
    if ignore is not None:
        keep = ~ignore(world)
        world = world[keep]
        if len(world) == 0:
            return 0
    vals = grid.lookup(world[:, 0], world[:, 1]).astype(np.int32)
    vals = np.where((vals < 0) | (vals == NO_INFORMATION), unknown_cost, vals)
    return int(vals.max()) if len(vals) else 0


def footprint_clearance(points, footprint):
    """Signed distance from each point (base_link frame) to the footprint polygon:
    > 0 outside, <= 0 inside. Used as the live safety distance while docking."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return np.empty(0)
    fp = np.asarray(footprint, dtype=float)
    best = np.full(len(pts), np.inf)
    for i in range(len(fp)):
        a, b = fp[i], fp[(i + 1) % len(fp)]
        ab = b - a
        t = np.clip(((pts - a) @ ab) / (ab @ ab + 1e-12), 0.0, 1.0)
        proj = a + t[:, None] * ab
        best = np.minimum(best, np.hypot(*(pts - proj).T))
    inside = points_in_polygon(pts, fp)
    return np.where(inside, -best, best)


# ---------------------------------------------------------------- candidates
def _gap_values(profile, step):
    """Preferred gap first, then the rest of the reach band outward from it."""
    vals = [profile.gap]
    k = 1
    while True:
        added = False
        for g in (profile.gap - k * step, profile.gap + k * step):
            if profile.gap_min - 1e-9 <= g <= profile.gap_max + 1e-9:
                vals.append(round(g, 4))
                added = True
        if not added:
            break
        k += 1
    return vals


def line_face_candidates(p1, p2, normal, front, half_width, profile, target=None,
                         lateral_step=0.05, gap_step=0.04, edge_margin=None):
    """Base poses in front of a flat face (segment p1-p2, `normal` pointing from the
    robot side INTO the surface). The base faces the surface (yaw along normal) and
    its front edge sits `gap` from the face. Lateral samples cover the face; the
    lateral error is measured to the target's projection (or the face centre)."""
    p1, p2, n = (np.asarray(v, dtype=float) for v in (p1, p2, normal))
    n = n / (np.linalg.norm(n) + 1e-12)
    d = p2 - p1
    length = float(np.linalg.norm(d))
    u = d / length if length > 1e-6 else np.array([-n[1], n[0]])
    yaw = math.atan2(n[1], n[0])
    # Keep the base mostly in front of the face: centre can go up to
    # `edge_margin` past each end (default half the base width).
    margin = half_width if edge_margin is None else edge_margin
    t_lo, t_hi = -margin, length + margin
    if target is not None:
        t_target = float((np.asarray(target, dtype=float) - p1) @ u)
    else:
        t_target = length / 2.0
    t_pref = min(max(t_target, 0.0), length)
    ts = np.arange(t_lo, t_hi + 1e-9, lateral_step)
    ts = np.unique(np.concatenate([ts, [t_pref]]))
    out = []
    for gap in _gap_values(profile, gap_step):
        back = front + gap
        for t in ts:
            q = p1 + u * t
            c = q - n * back
            out.append(Candidate(float(c[0]), float(c[1]), yaw, gap=gap,
                                 lateral=abs(float(t) - t_target)))
    return out


def circle_face_candidates(center, radius, front, profile, ref_angle, target=None,
                           angle_step_deg=10.0, max_angle_deg=180.0, gap_step=0.04):
    """Base poses around a round table: on a radius line, facing the centre.
    `ref_angle` is the bearing centre->robot. With a target on the table, the
    preferred bearing is centre->target (stand behind the object)."""
    c = np.asarray(center, dtype=float)
    if target is not None:
        tv = np.asarray(target, dtype=float) - c
        pref = math.atan2(tv[1], tv[0]) if np.linalg.norm(tv) > 0.05 else ref_angle
    else:
        pref = ref_angle
    steps = int(max_angle_deg // angle_step_deg)
    out = []
    for gap in _gap_values(profile, gap_step):
        rr = radius + gap + front
        for k in range(-steps, steps + 1):
            a = ref_angle + math.radians(angle_step_deg * k)
            x, y = c[0] + rr * math.cos(a), c[1] + rr * math.sin(a)
            yaw = math.atan2(c[1] - y, c[0] - x)
            dang = abs((a - pref + math.pi) % (2 * math.pi) - math.pi)
            out.append(Candidate(float(x), float(y), yaw, gap=gap,
                                 lateral=dang * (radius + 1e-6),
                                 extra={"angle": a}))
    return out


def ring_candidates(target, robot, d_min, d_pref, d_max, d_step=0.15,
                    angle_step_deg=10.0):
    """Base poses on rings around a point target (person/object), facing it."""
    tx, ty = target
    base = math.atan2(robot[1] - ty, robot[0] - tx)
    dists = [d_pref] + [d for d in np.arange(d_min, d_max + 1e-9, d_step)
                        if abs(d - d_pref) > 1e-6]
    out = []
    for d in dists:
        for k in range(0, int(180 // angle_step_deg) + 1):
            for sign in ((1,) if k in (0, int(180 // angle_step_deg)) else (1, -1)):
                a = base + sign * math.radians(angle_step_deg * k)
                x, y = tx + d * math.cos(a), ty + d * math.sin(a)
                out.append(Candidate(float(x), float(y), math.atan2(ty - y, tx - x),
                                     gap=float(d), lateral=0.0, extra={"angle": a}))
    return out


# ------------------------------------------------------------------ ranking
def rank(candidates, profile, weights=None, grid=None, samples=None, robot=None,
         ignore=None, lethal=LETHAL, unknown_cost=0):
    """Score every candidate (lower is better). A footprint cell >= `lethal` (an
    actual obstacle, not inflation: the footprint itself is checked) rejects.
    Returns (valid sorted best-first, all candidates)."""
    w = weights or Weights()
    band = max(profile.gap_max - profile.gap_min, 1e-3)
    for c in candidates:
        if grid is not None and samples is not None:
            c.cost = footprint_cost(grid, c.x, c.y, c.yaw, samples, ignore, unknown_cost)
            if c.cost >= lethal:
                c.valid = False
                c.reason = f"footprint cost {c.cost}"
        if robot is not None:
            c.travel = math.hypot(c.x - robot[0], c.y - robot[1])
        c.score = (w.cost * min(c.cost, 252) / 252.0
                   + w.reach * abs(c.gap - profile.gap) / band
                   + w.lateral * c.lateral
                   + w.travel * c.travel)
    valid = sorted((c for c in candidates if c.valid),
                   key=lambda c: (round(c.score, 6), c.x, c.y))
    return valid, candidates


def footprint_astar(grid, start, goal, yaw, samples, ignore=None, step=None,
                    cost_weight=1.0, max_expansions=20000):
    """Holonomic A* over (x, y) at fixed heading `yaw` on `grid`: every node keeps
    the whole footprint (`samples`) off lethal cells. For short approach legs on
    the local costmap, where the obstacles nav2's planner lost (thin chair legs)
    are known. Returns [(x, y), ...] from start to goal, or None."""
    step = step or grid.resolution
    sx, sy = start
    gx, gy = goal

    def key(x, y):
        return (int(round((x - sx) / step)), int(round((y - sy) / step)))

    def world(k):
        return sx + k[0] * step, sy + k[1] * step

    free_cache = {}

    def cost(k):
        if k not in free_cache:
            x, y = world(k)
            free_cache[k] = footprint_cost(grid, x, y, yaw, samples, ignore)
        return free_cache[k]

    goal_k = key(gx, gy)
    if cost(goal_k) >= LETHAL:
        return None
    start_k = (0, 0)
    moves = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    h = lambda k: math.hypot(k[0] - goal_k[0], k[1] - goal_k[1])  # noqa: E731
    openq = [(h(start_k), 0.0, start_k)]
    came = {start_k: None}
    g = {start_k: 0.0}
    expansions = 0
    while openq and expansions < max_expansions:
        _, gk, k = heapq.heappop(openq)
        if k == goal_k:
            path = []
            while k is not None:
                path.append(world(k))
                k = came[k]
            path.reverse()
            path[-1] = (gx, gy)
            return path
        if gk > g.get(k, math.inf):
            continue
        expansions += 1
        for dx, dy, d in moves:
            n = (k[0] + dx, k[1] + dy)
            c = cost(n)
            if c >= LETHAL and n != start_k:
                continue
            ng = gk + d * (1.0 + cost_weight * min(c, 252) / 252.0)
            if ng < g.get(n, math.inf):
                g[n] = ng
                came[n] = k
                heapq.heappush(openq, (ng + h(n), ng, n))
    return None


def halfplane_ignore(point_on_face, normal, margin):
    """Ignore cells on/behind a flat face: (p - face) . n > -margin."""
    p0 = np.asarray(point_on_face, dtype=float)
    n = np.asarray(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-12)

    def fn(world):
        return (world - p0) @ n > -margin
    return fn


def disc_ignore(center, radius, margin):
    """Ignore cells inside a round table (+ margin)."""
    c = np.asarray(center, dtype=float)

    def fn(world):
        return np.hypot(world[:, 0] - c[0], world[:, 1] - c[1]) < radius + margin
    return fn


def load_weights(data, key):
    """`weights.<key>` of approach_profiles.yaml (dock | point) as Weights."""
    return Weights(**((data or {}).get(key) or {}))


def load_profiles(data):
    """Parse the `profiles` mapping of approach_profiles.yaml."""
    out = {}
    for name, p in (data or {}).items():
        out[name] = ReachProfile(float(p["gap"]), float(p["gap_min"]),
                                 float(p["gap_max"]), str(p.get("shape", "line")))
    return out

"""Ground-truth geometry of the navigation arena, for checking approaches.

The arena world is built from the occupied cells of the navigation map
(scripts/world_from_map.py), so the true obstacle set is exactly those cells plus
any furniture the test spawns. Distances here use the robot's Gazebo pose, never
what the navigation stack reports.
"""

import math

import numpy as np

# Keep in sync with nav2_omni*.yaml `footprint`
FOOTPRINT = np.array([[0.325, 0.25], [0.325, -0.25], [-0.325, -0.25], [-0.325, 0.25]])


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = (int(v) for v in line.split())
        f.readline()
        return np.frombuffer(f.read(width * height), dtype=np.uint8).reshape(
            height, width
        )


def map_obstacle_points(pgm_path, resolution, origin, occupied_thresh=0.65, step=0.01):
    """Dense points covering every occupied map cell (the arena walls)."""
    grid = read_pgm(pgm_path)
    height = grid.shape[0]
    rows, cols = np.nonzero(grid < int((1.0 - occupied_thresh) * 255))
    offs = np.arange(0.0, resolution + 1e-9, step)
    ox, oy = np.meshgrid(offs, offs)
    ox, oy = ox.ravel(), oy.ravel()
    x0 = origin[0] + cols * resolution
    y0 = origin[1] + (height - 1 - rows) * resolution
    return np.stack([(x0[:, None] + ox).ravel(), (y0[:, None] + oy).ravel()], axis=1)


def box_points(cx, cy, yaw, sx, sy, step=0.01):
    """Points covering a box footprint (sx x sy) at (cx, cy, yaw)."""
    xs = np.arange(-sx / 2, sx / 2 + 1e-9, step)
    ys = np.arange(-sy / 2, sy / 2 + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack(
        [cx + c * gx.ravel() - s * gy.ravel(), cy + s * gx.ravel() + c * gy.ravel()],
        axis=1,
    )


def disc_points(cx, cy, r, step=0.01):
    xs = np.arange(-r, r + 1e-9, step)
    gx, gy = np.meshgrid(xs, xs)
    m = np.hypot(gx, gy) <= r
    return np.stack([cx + gx[m], cy + gy[m]], axis=1)


def to_base(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    d = points - np.array([x, y])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def footprint_clearance(points_base, footprint=FOOTPRINT):
    """Min signed distance from points (base frame) to the footprint polygon
    (negative = inside, i.e. the body overlaps the obstacle)."""
    fp = np.asarray(footprint, dtype=float)
    best = np.full(len(points_base), np.inf)
    for i in range(len(fp)):
        a, b = fp[i], fp[(i + 1) % len(fp)]
        ab = b - a
        t = np.clip(((points_base - a) @ ab) / (ab @ ab), 0.0, 1.0)
        proj = a + t[:, None] * ab
        best = np.minimum(best, np.hypot(*(points_base - proj).T))
    x, y = points_base[:, 0], points_base[:, 1]
    inside = (
        (x <= fp[:, 0].max())
        & (x >= fp[:, 0].min())
        & (y <= fp[:, 1].max())
        & (y >= fp[:, 1].min())
    )
    return np.where(inside, -best, best)


def clearance(points_world, pose, radius=2.0):
    """(min clearance, nearest point world) of the footprint at pose=(x, y, yaw)."""
    x, y, yaw = pose
    near = points_world[
        np.hypot(points_world[:, 0] - x, points_world[:, 1] - y) < radius
    ]
    if len(near) == 0:
        return math.inf, None
    d = footprint_clearance(to_base(near, x, y, yaw))
    i = int(np.argmin(d))
    return float(d[i]), near[i]

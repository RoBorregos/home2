"""Costmap validation and relocation of tagged viewpoints.

The sublocation poses in ``areas_<MAP_NAME>.json`` were placed by hand with
``map_area_tagger``, so they are usually good. "Usually" is not enough for a
patrol: furniture moves, maps get re-recorded, and a viewpoint that now sits
inside the inflation makes Nav2 reject the goal and stalls the whole route. This
module checks each pose against an occupancy grid and, when one is blocked,
walks a ring around the furniture to find the nearest free replacement that still
faces it — the same idea as ``nav_central._find_free_approach``
(`nav_central.py:1015`), but ROS-free so it can be exercised offline against the
saved ``.pgm`` of the arena.

Pure stdlib: the grid is read as the flat row-major sequence that
``nav_msgs/OccupancyGrid`` already provides (-1 unknown, 0-100 cost), so nothing
here needs numpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence

from .surfaces import Viewpoint

UNKNOWN = -1

# Matches nav_central._grid_cell_free: cost >= 50 is "too close to an obstacle".
# The costmap's own inflation_radius (0.45 m, nav2_omni_limp.yaml:199) already
# bakes the clearance into the cost, so a single-cell test is meaningful.
DEFAULT_MAX_COST = 50

# Half the footprint's short side (footprint 0.65 x 0.50 m,
# nav2_omni_limp.yaml:192) — the inscribed radius of the base.
DEFAULT_CHECK_RADIUS = 0.25

# How far in front of a tagged pose the furniture is assumed to be when the
# areas JSON carries no centroid. The operator tags the robot standing at
# grasping distance, so this is about one standoff.
DEFAULT_PROBE_DISTANCE = 0.8


@dataclass(frozen=True)
class Grid:
    """An occupancy grid in the map frame, in OccupancyGrid conventions."""

    data: Sequence[int]
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def cell_of(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def cost_at(self, x: float, y: float) -> int | None:
        """Cost at a world point; ``None`` outside the grid, ``-1`` unknown."""
        cell = self.cell_of(x, y)
        if cell is None:
            return None
        col, row = cell
        return int(self.data[row * self.width + col])


def is_free(
    grid: Grid,
    x: float,
    y: float,
    max_cost: int = DEFAULT_MAX_COST,
    unknown_is_free: bool = False,
) -> bool:
    """Whether a single world point is on a known, cheap-enough cell.

    Out of bounds is never free. Unknown follows `unknown_is_free`: strict for
    picking a *new* pose, lenient when judging one a human already chose.
    """
    cost = grid.cost_at(x, y)
    if cost is None:
        return False
    if cost == UNKNOWN:
        return unknown_is_free
    return cost < max_cost


def is_area_free(
    grid: Grid,
    x: float,
    y: float,
    radius: float = DEFAULT_CHECK_RADIUS,
    max_cost: int = DEFAULT_MAX_COST,
    unknown_is_free: bool = False,
) -> bool:
    """`is_free` over the base's inscribed disk, sampled on the grid pitch."""
    if radius <= 0.0:
        return is_free(grid, x, y, max_cost, unknown_is_free)
    step = max(grid.resolution, 0.01)
    steps = int(radius / step)
    for dx in range(-steps, steps + 1):
        for dy in range(-steps, steps + 1):
            ox, oy = dx * step, dy * step
            if math.hypot(ox, oy) > radius:
                continue
            if not is_free(grid, x + ox, y + oy, max_cost, unknown_is_free):
                return False
    return True


def target_of(vp: Viewpoint, probe_distance: float = DEFAULT_PROBE_DISTANCE) -> tuple[float, float]:
    """Where the furniture is, as far as we can tell.

    Prefers an explicit centroid in the sublocation's ``_meta`` (``center``);
    otherwise projects `probe_distance` along the tagged heading, since the
    operator tags the robot *facing* the furniture.
    """
    center = vp.meta.get("center") if isinstance(vp.meta, dict) else None
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        try:
            return float(center[0]), float(center[1])
        except (TypeError, ValueError):
            pass
    yaw = vp.yaw
    return vp.x + probe_distance * math.cos(yaw), vp.y + probe_distance * math.sin(yaw)


def facing(x: float, y: float, tx: float, ty: float) -> tuple[float, float]:
    """(qz, qw) for a pose at (x, y) looking at (tx, ty)."""
    yaw = math.atan2(ty - y, tx - x)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def relocate(
    vp: Viewpoint,
    grid: Grid,
    max_shift: float = 0.6,
    radius: float = DEFAULT_CHECK_RADIUS,
    max_cost: int = DEFAULT_MAX_COST,
    probe_distance: float = DEFAULT_PROBE_DISTANCE,
) -> Viewpoint | None:
    """Nearest free pose that still faces the same furniture, or ``None``.

    Walks outward from the tagged standoff in 10 cm shells and fans ±180° around
    the furniture, starting from the original direction, so the first hit is the
    smallest change from what the operator intended.
    """
    tx, ty = target_of(vp, probe_distance)
    standoff = math.hypot(vp.x - tx, vp.y - ty)
    if standoff < 1e-3:
        return None
    base = math.atan2(vp.y - ty, vp.x - tx)  # furniture -> tagged pose

    shells = [standoff]
    shell = standoff + 0.15
    while shell <= standoff + max_shift + 1e-6:
        shells.append(shell)
        shell += 0.15

    for shell in shells:
        for step in range(0, 19):  # 0°, then ±10° ... ±180°
            for sign in (1,) if step == 0 else (1, -1):
                angle = base + sign * math.radians(10 * step)
                gx = tx + shell * math.cos(angle)
                gy = ty + shell * math.sin(angle)
                if not is_area_free(grid, gx, gy, radius, max_cost):
                    continue
                qz, qw = facing(gx, gy, tx, ty)
                return replace(vp, x=gx, y=gy, qz=qz, qw=qw)
    return None


@dataclass
class ValidationReport:
    """What happened to each viewpoint, for logging and for the patrol response."""

    kept: list[Viewpoint]
    relocated: list[tuple[str, float]]  # (key, meters moved)
    dropped: list[str]

    @property
    def summary(self) -> str:
        parts = [f"{len(self.kept)} ok"]
        if self.relocated:
            worst = max(shift for _, shift in self.relocated)
            parts.append(f"{len(self.relocated)} relocated (max {worst:.2f} m)")
        if self.dropped:
            parts.append(f"{len(self.dropped)} dropped: {', '.join(self.dropped)}")
        return ", ".join(parts)


def validate_viewpoints(
    viewpoints: list[Viewpoint],
    grid: Grid | None,
    radius: float = DEFAULT_CHECK_RADIUS,
    max_cost: int = DEFAULT_MAX_COST,
    max_shift: float = 0.6,
    probe_distance: float = DEFAULT_PROBE_DISTANCE,
    unknown_is_free: bool = True,
) -> ValidationReport:
    """Keep the poses the robot can actually stand on, relocating what it can.

    With no grid, every pose is kept: a missing costmap must degrade to today's
    behaviour, never to an empty patrol.

    `unknown_is_free` defaults to True *for the incoming check*: a hand-tagged
    pose over an unmapped cell is normal near the walls of a partially explored
    arena, and dropping it would be worse than letting Nav2 judge. Replacement
    poses are always chosen on known cells.
    """
    if grid is None:
        return ValidationReport(kept=list(viewpoints), relocated=[], dropped=[])

    kept: list[Viewpoint] = []
    relocated: list[tuple[str, float]] = []
    dropped: list[str] = []

    for vp in viewpoints:
        if is_area_free(grid, vp.x, vp.y, radius, max_cost, unknown_is_free):
            kept.append(vp)
            continue
        moved = relocate(vp, grid, max_shift, radius, max_cost, probe_distance)
        if moved is None:
            dropped.append(vp.key)
            continue
        kept.append(moved)
        relocated.append((vp.key, math.hypot(moved.x - vp.x, moved.y - vp.y)))

    return ValidationReport(kept=kept, relocated=relocated, dropped=dropped)


# ----------------------------------------------------------------- offline only


def grid_from_pgm(pgm_path: str, map_yaml: dict) -> Grid:
    """Read a saved map (``.pgm`` + its YAML) into a :class:`Grid`.

    For offline testing against the real arena map; the node builds its Grid from
    a live ``OccupancyGrid`` instead. Applies the nav2 trinary conversion using
    the YAML's ``occupied_thresh`` / ``free_thresh`` / ``negate``.
    """
    with open(pgm_path, "rb") as handle:
        raw = handle.read()

    # P5 header: magic, then width height maxval, skipping '#' comments.
    fields: list[int] = []
    index = 2  # past "P5"
    while len(fields) < 3:
        while index < len(raw) and raw[index : index + 1].isspace():
            index += 1
        if raw[index : index + 1] == b"#":
            while index < len(raw) and raw[index : index + 1] not in (b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(raw) and not raw[index : index + 1].isspace():
            index += 1
        fields.append(int(raw[start:index]))
    index += 1  # single whitespace byte after maxval
    width, height, _maxval = fields
    pixels = raw[index : index + width * height]

    occupied = float(map_yaml.get("occupied_thresh", 0.65))
    free = float(map_yaml.get("free_thresh", 0.196))
    negate = bool(map_yaml.get("negate", 0))
    origin = map_yaml.get("origin", [0.0, 0.0, 0.0])

    data = [0] * (width * height)
    for row in range(height):
        # PGM rows run top-down; OccupancyGrid rows run bottom-up.
        src = (height - 1 - row) * width
        dst = row * width
        for col in range(width):
            value = pixels[src + col]
            ratio = value / 255.0 if negate else (255 - value) / 255.0
            if ratio > occupied:
                data[dst + col] = 100
            elif ratio < free:
                data[dst + col] = 0
            else:
                data[dst + col] = UNKNOWN
    return Grid(
        data=data,
        width=width,
        height=height,
        resolution=float(map_yaml.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )

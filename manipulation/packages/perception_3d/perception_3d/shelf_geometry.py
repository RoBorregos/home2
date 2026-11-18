"""Model-free shelf detection from z-up depth points (no ROS, tested with synthetic clouds).

Board faces are peaks in the Z histogram of horizontal points; walls are the vertical
planes at the extremes. See manipulation/SHELF.md.
"""

from collections.abc import Sequence
from dataclasses import dataclass, field, replace

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ShelfParams:
    """Tunables. Only physical tolerances: nothing here describes one shelf."""

    # Footprint in the shelf frame, m. x forward from the robot, y left, z up.
    x_min: float = 0.15  # drop the robot's own body
    x_max: float = 1.20
    y_half: float = 0.80
    z_min: float = 0.04  # drop the floor
    z_max: float = 1.80

    voxel: float = 0.01  # downsample grid, m
    normal_neighbours: int = 12
    horizontal_cos: float = 0.90  # |nz| above -> board face
    vertical_cos: float = 0.30  # |nz| below -> wall candidate

    z_bin: float = 0.01
    min_level_points: int = 60  # smoothed histogram peak height
    max_board_thickness: float = 0.06
    min_level_spacing: float = 0.12  # two surfaces closer than this are one board
    min_width_fraction: float = 0.60  # of the reference width (walls or widest board)
    min_fill_fraction: float = 0.60  # occupied y-bins across the surface width
    extent_bin: float = 0.02
    default_thickness: float = 0.018

    wall_bin: float = 0.02
    min_wall_points: int = 40
    wall_min_height: float = 0.25
    wall_search: float = 0.20  # a wall must sit within this of the boards' extent
    wall_thickness: float = 0.02

    board_sink: float = 0.01  # collision box top sits this far below the surface
    padding: float = 0.02  # every other box face grown this far into the free space


@dataclass(frozen=True)
class ShelfLevel:
    height: float
    thickness: float
    front_x: float
    back_x: float
    y_min: float
    y_max: float
    clearance: float = 0.0
    num_points: int = 0
    # Only the underside was in view: height = underside + default thickness.
    seen_from_below: bool = False


@dataclass(frozen=True)
class Box:
    """Axis-aligned collision box: center and full size, in the shelf frame."""

    id: str
    center: tuple
    size: tuple


@dataclass(frozen=True)
class ShelfGeometry:
    levels: tuple  # ShelfLevel, ascending height
    y_min: float
    y_max: float
    front_x: float
    back_x: float
    top_z: float = 0.0  # underside of the top board when only seen from below, else 0
    has_left_wall: bool = False
    has_right_wall: bool = False
    has_back_wall: bool = False
    boxes: tuple = field(default_factory=tuple)

    @property
    def heights(self) -> list:
        return [level.height for level in self.levels]


@dataclass(frozen=True)
class Frame:
    """Points of one depth frame in the shelf frame, plus where the camera was."""

    points: np.ndarray  # (N, 3)
    viewpoint: np.ndarray  # (3,)


class ShelfNotFound(Exception):
    pass


# ======================================================================
# Point cloud helpers
# ======================================================================


def crop(points: np.ndarray, p: ShelfParams) -> np.ndarray:
    keep = (
        (points[:, 0] > p.x_min)
        & (points[:, 0] < p.x_max)
        & (np.abs(points[:, 1]) < p.y_half)
        & (points[:, 2] > p.z_min)
        & (points[:, 2] < p.z_max)
    )
    return points[keep]


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """One point (the centroid) per occupied voxel."""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    counts = np.bincount(inverse).astype(float)
    sums = np.zeros((len(counts), 3))
    for axis in range(3):
        sums[:, axis] = np.bincount(inverse, weights=points[:, axis])
    return sums / counts[:, None]


def estimate_normals(points: np.ndarray, viewpoint: np.ndarray, k: int) -> np.ndarray:
    """PCA normals oriented toward the camera, so nz > 0 means a face seen from above."""
    n = len(points)
    if n == 0:
        return np.zeros((0, 3))
    k = int(min(k, n))
    if k < 3:
        return np.tile([0.0, 0.0, 1.0], (n, 1))
    _, idx = cKDTree(points).query(points, k=k)
    neighbours = points[idx]  # (n, k, 3)
    centred = neighbours - neighbours.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centred, centred) / k
    _, vectors = np.linalg.eigh(cov)  # ascending eigenvalues
    normals = vectors[:, :, 0]
    to_camera = viewpoint[None, :] - points
    flip = np.einsum("ij,ij->i", normals, to_camera) < 0
    normals[flip] *= -1.0
    # Up/down from the camera height: the view-vector test fails at grazing angles.
    horizontal = np.abs(normals[:, 2]) > 0.8
    want_up = viewpoint[2] > points[horizontal, 2]
    sign = np.where(want_up, 1.0, -1.0)
    normals[horizontal] *= (sign * np.sign(normals[horizontal, 2]))[:, None]
    return normals


def _histogram_peaks(
    values: np.ndarray, lo: float, hi: float, width: float, min_count: float
):
    """Smoothed 1D histogram peaks -> list of (refined_position, count)."""
    if len(values) == 0 or hi <= lo:
        return []
    edges = np.arange(lo, hi + width, width)
    counts, _ = np.histogram(values, bins=edges)
    smooth = np.convolve(counts, [0.25, 0.5, 0.25], mode="same")
    peaks = []
    for i in range(len(smooth)):
        left = smooth[i - 1] if i > 0 else -1.0
        right = smooth[i + 1] if i + 1 < len(smooth) else -1.0
        if smooth[i] < min_count or smooth[i] < left or smooth[i] <= right:
            continue
        centre = edges[i] + width / 2
        window = values[np.abs(values - centre) <= 1.5 * width]
        peaks.append(
            (float(window.mean()) if len(window) else centre, float(smooth[i]))
        )
    return peaks


def _extent(values: np.ndarray, lo_pct: float = 2.0, hi_pct: float = 98.0):
    return float(np.percentile(values, lo_pct)), float(np.percentile(values, hi_pct))


def _fill_fraction(values: np.ndarray, lo: float, hi: float, width: float) -> float:
    bins = max(1, int(np.ceil((hi - lo) / width)))
    occupied = np.unique(np.clip(((values - lo) / width).astype(int), 0, bins - 1))
    return len(occupied) / bins


# ======================================================================
# Detection
# ======================================================================


def classify(frames: Sequence[Frame], p: ShelfParams):
    """Crop, downsample and label every frame. Returns (points, normals)."""
    all_points, all_normals = [], []
    for frame in frames:
        points = voxel_downsample(
            crop(np.asarray(frame.points, dtype=float), p), p.voxel
        )
        if len(points) == 0:
            continue
        all_points.append(points)
        all_normals.append(
            estimate_normals(
                points, np.asarray(frame.viewpoint, dtype=float), p.normal_neighbours
            )
        )
    if not all_points:
        raise ShelfNotFound("no points inside the shelf footprint")
    return np.vstack(all_points), np.vstack(all_normals)


def _surface_candidates(up: np.ndarray, p: ShelfParams):
    """Up-facing Z peaks with their lateral/depth extents and fill."""
    peaks = _histogram_peaks(up[:, 2], p.z_min, p.z_max, p.z_bin, p.min_level_points)
    candidates = []
    for z, _ in peaks:
        slab = up[np.abs(up[:, 2] - z) <= 1.5 * p.z_bin]
        if len(slab) < p.min_level_points:
            continue
        y_lo, y_hi = _extent(slab[:, 1])
        x_lo, x_hi = _extent(slab[:, 0])
        candidates.append(
            {
                "z": z,
                "n": len(slab),
                "y": (y_lo, y_hi),
                "x": (x_lo, x_hi),
                "fill": _fill_fraction(slab[:, 1], y_lo, y_hi, p.extent_bin),
            }
        )
    return candidates


def _walls(
    vertical: np.ndarray, normals: np.ndarray, boards_y, boards_back_x, p: ShelfParams
):
    """Side walls near the boards' lateral extent, back wall near their far edge."""
    found = {"left": None, "right": None, "back": None}
    if len(vertical) == 0:
        return found
    side = vertical[np.abs(normals[:, 1]) > 0.8]
    for y_peak, _ in _histogram_peaks(
        side[:, 1], -p.y_half, p.y_half, p.wall_bin, p.min_wall_points
    ):
        strip = side[np.abs(side[:, 1] - y_peak) <= 1.5 * p.wall_bin]
        z_lo, z_hi = _extent(strip[:, 2])
        if z_hi - z_lo < p.wall_min_height:
            continue
        for name, edge in (("left", boards_y[0]), ("right", boards_y[1])):
            gap = abs(y_peak - edge)
            if gap <= p.wall_search and (
                found[name] is None or gap < abs(found[name] - edge)
            ):
                found[name] = y_peak  # the plane nearest the boards' edge
    # Back wall: a plane behind the boards that is tall or spans most of the width.
    width = boards_y[1] - boards_y[0]
    back = vertical[np.abs(normals[:, 0]) > 0.8]
    for x_peak, _ in _histogram_peaks(
        back[:, 0], p.x_min, p.x_max, p.wall_bin, p.min_wall_points
    ):
        if x_peak < boards_back_x - p.wall_search:
            continue
        strip = back[np.abs(back[:, 0] - x_peak) <= 1.5 * p.wall_bin]
        z_lo, z_hi = _extent(strip[:, 2])
        y_lo, y_hi = _extent(strip[:, 1])
        tall = z_hi - z_lo >= p.wall_min_height
        wide = y_hi - y_lo >= p.min_width_fraction * width
        if not (tall or wide):
            continue
        if found["back"] is None or x_peak < found["back"]:
            found["back"] = x_peak  # the nearest tall plane behind the boards
    return found


def _boards_seen_from_below(
    down: np.ndarray, undersides, levels: list, reference: float, p: ShelfParams
) -> list:
    """Boards seen only from below: wide down-facing planes above the lowest surface."""
    found = []
    lowest = levels[0].height
    for z, _ in undersides:
        if z < lowest:
            continue
        known = [lv.height for lv in levels] + [lv.height for lv in found]
        if any(
            -p.min_level_spacing / 2 < h - z <= p.max_board_thickness for h in known
        ):
            continue  # the underside of a board already seen, or noise next to one
        slab = down[np.abs(down[:, 2] - z) <= 1.5 * p.z_bin]
        y_lo, y_hi = _extent(slab[:, 1])
        if (y_hi - y_lo) < p.min_width_fraction * reference:
            continue
        if _fill_fraction(slab[:, 1], y_lo, y_hi, p.extent_bin) < p.min_fill_fraction:
            continue
        x_lo, x_hi = _extent(slab[:, 0])
        found.append(
            ShelfLevel(
                height=float(z + p.default_thickness),
                thickness=p.default_thickness,
                front_x=x_lo,
                back_x=x_hi,
                y_min=y_lo,
                y_max=y_hi,
                num_points=len(slab),
                seen_from_below=True,
            )
        )
    return found


def detect_shelf(
    frames: Sequence[Frame], params: ShelfParams | None = None
) -> ShelfGeometry:
    """Detect levels, walls and collision boxes. Raises ShelfNotFound when no board is seen."""
    p = params or ShelfParams()
    points, normals = classify(frames, p)
    nz = normals[:, 2]
    up = points[nz > p.horizontal_cos]
    down = points[nz < -p.horizontal_cos]
    vertical_mask = np.abs(nz) < p.vertical_cos

    candidates = _surface_candidates(up, p)
    if not candidates:
        raise ShelfNotFound("no horizontal surface in view")

    # Reference width: the side walls when both are seen, else the widest surface.
    widest = max(c["y"][1] - c["y"][0] for c in candidates)
    y_all = (min(c["y"][0] for c in candidates), max(c["y"][1] for c in candidates))
    back_all = max(c["x"][1] for c in candidates)
    walls = _walls(points[vertical_mask], normals[vertical_mask], y_all, back_all, p)
    if walls["left"] is not None and walls["right"] is not None:
        reference = walls["right"] - walls["left"]
    else:
        reference = widest

    accepted = [
        c
        for c in candidates
        if (c["y"][1] - c["y"][0]) >= p.min_width_fraction * reference
        and c["fill"] >= p.min_fill_fraction
    ]
    if not accepted:
        raise ShelfNotFound(
            "every horizontal surface looks like an object, not a board"
        )

    # Merge surfaces closer than one compartment: keep the better supported one.
    accepted.sort(key=lambda c: c["z"])
    merged = []
    for c in accepted:
        if merged and c["z"] - merged[-1]["z"] < p.min_level_spacing:
            if c["n"] > merged[-1]["n"]:
                merged[-1] = c
            continue
        merged.append(c)

    # Undersides give each board's thickness and the boards above the camera.
    undersides = _histogram_peaks(
        down[:, 2], p.z_min, p.z_max, p.z_bin, p.min_level_points
    )
    levels = []
    for c in merged:
        below = [z for z, _ in undersides if 0.0 < c["z"] - z <= p.max_board_thickness]
        thickness = c["z"] - max(below) if below else p.default_thickness
        levels.append(
            ShelfLevel(
                height=c["z"],
                thickness=float(thickness),
                front_x=c["x"][0],
                back_x=c["x"][1],
                y_min=c["y"][0],
                y_max=c["y"][1],
                num_points=int(c["n"]),
            )
        )
    levels.extend(_boards_seen_from_below(down, undersides, levels, reference, p))
    levels.sort(key=lambda level: level.height)
    # Underside of the topmost board when the camera only saw it from below.
    top_z = (
        levels[-1].height - levels[-1].thickness if levels[-1].seen_from_below else 0.0
    )

    # Clearance up to the underside of the board above; 0 on the top board.
    with_clearance = [
        replace(
            level,
            clearance=float(max(above.height - above.thickness - level.height, 0.0)),
        )
        for level, above in zip(levels, levels[1:])
    ] + [levels[-1]]

    geometry = ShelfGeometry(
        levels=tuple(with_clearance),
        y_min=float(walls["left"] if walls["left"] is not None else y_all[0]),
        y_max=float(walls["right"] if walls["right"] is not None else y_all[1]),
        front_x=float(min(level.front_x for level in with_clearance)),
        back_x=float(walls["back"] if walls["back"] is not None else back_all),
        top_z=float(top_z),
        has_left_wall=walls["left"] is not None,
        has_right_wall=walls["right"] is not None,
        has_back_wall=walls["back"] is not None,
    )
    return replace(geometry, boxes=tuple(collision_boxes(geometry, p)))


# ======================================================================
# Planning-scene boxes
# ======================================================================


def collision_boxes(
    g: ShelfGeometry, p: ShelfParams, namespace: str = "frida_shelf_"
) -> list:
    """Boards, side walls and back wall as axis-aligned boxes.

    Board tops sink ``board_sink`` so fingertips at an object's base do not collide;
    every other face grows ``padding`` into the free space for scan and hand-eye error.
    """
    boxes = []
    pad = p.padding

    def box(name, lo, hi):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        boxes.append(
            Box(
                id=f"{namespace}{name}",
                center=tuple((lo + hi) / 2),
                size=tuple(hi - lo),
            )
        )

    front = g.front_x - pad
    back = max(g.back_x, g.front_x + 0.05)
    for i, level in enumerate(g.levels):
        top = level.height - p.board_sink
        box(
            f"board_{i + 1}",
            (front, g.y_min, top - level.thickness - pad),
            (back, g.y_max, top),
        )
    # Walls run from the lowest board to the highest one.
    bottom = g.levels[0].height - g.levels[0].thickness
    top = g.levels[-1].height
    if g.has_left_wall:
        box(
            "wall_left",
            (front, g.y_min - p.wall_thickness, bottom),
            (back, g.y_min + pad, top),
        )
    if g.has_right_wall:
        box(
            "wall_right",
            (front, g.y_max - pad, bottom),
            (back, g.y_max + p.wall_thickness, top),
        )
    if g.has_back_wall:
        box(
            "wall_back",
            (g.back_x - pad, g.y_min, bottom),
            (g.back_x + p.wall_thickness, g.y_max, top),
        )
    return boxes


__all__ = [
    "Box",
    "Frame",
    "ShelfGeometry",
    "ShelfLevel",
    "ShelfNotFound",
    "ShelfParams",
    "collision_boxes",
    "detect_shelf",
]

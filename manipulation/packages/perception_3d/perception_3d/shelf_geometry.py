"""Model-free shelf detection from depth points expressed in a z-up frame.

No ROS here, so the whole thing is unit-testable with synthetic clouds.

The idea: a shelf is a stack of horizontal boards. Points whose normal is vertical
pile up at a few heights, so a histogram of their Z has one peak per board face.
Up-facing peaks are surfaces objects rest on, down-facing peaks are the undersides
(which give the board thickness and the ceiling of each compartment). Nothing is
assumed about the number of levels or their spacing beyond physics:

* a board spans (almost) the whole shelf width, an object on it does not;
* a board is thin, so its underside sits a few centimetres below its surface;
* two consecutive levels are far enough apart to put something in between.

Side and back walls are the vertical planes at the lateral/far extremes. The
result also carries axis-aligned boxes for the planning scene so MoveIt knows about
the compartment ceiling and walls the eye-in-hand octomap never sees.
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
    top_z: float = 0.0  # underside of the top board, 0 = not seen
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
    # A horizontal face is only visible from the side the camera is on, and the
    # view-vector test above is unreliable at grazing angles (a far board seen from
    # almost its own height), so settle up/down from the camera height alone.
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
        if abs(y_peak - boards_y[0]) <= p.wall_search and (
            found["left"] is None
            or abs(y_peak - boards_y[0]) < abs(found["left"] - boards_y[0])
        ):
            found["left"] = y_peak
        if abs(y_peak - boards_y[1]) <= p.wall_search and (
            found["right"] is None
            or abs(y_peak - boards_y[1]) < abs(found["right"] - boards_y[1])
        ):
            found["right"] = y_peak
    back = vertical[np.abs(normals[:, 0]) > 0.8]
    for x_peak, _ in _histogram_peaks(
        back[:, 0], p.x_min, p.x_max, p.wall_bin, p.min_wall_points
    ):
        strip = back[np.abs(back[:, 0] - x_peak) <= 1.5 * p.wall_bin]
        z_lo, z_hi = _extent(strip[:, 2])
        if z_hi - z_lo < p.wall_min_height or x_peak < boards_back_x - p.wall_search:
            continue
        if found["back"] is None or x_peak < found["back"]:
            found["back"] = x_peak  # the nearest tall plane behind the boards
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

    # Undersides: thickness of each board, and the ceiling above the top level.
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
    top_candidates = []
    for z, _ in undersides:
        if z - levels[-1].height < p.min_level_spacing:
            continue
        slab = down[np.abs(down[:, 2] - z) <= 1.5 * p.z_bin]
        y_lo, y_hi = _extent(slab[:, 1])
        if (y_hi - y_lo) >= p.min_width_fraction * reference:
            top_candidates.append(z)
    top_z = min(top_candidates) if top_candidates else 0.0

    with_clearance = []
    for i, level in enumerate(levels):
        if i + 1 < len(levels):
            above = levels[i + 1]
            clearance = above.height - above.thickness - level.height
        else:
            clearance = top_z - level.height if top_z else 0.0
        with_clearance.append(replace(level, clearance=float(max(clearance, 0.0))))

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
    """Boards, side walls, back wall and top board as axis-aligned boxes.

    Board tops are sunk ``board_sink`` below the measured surface so the fingertips
    touching an object's base do not register as a collision with the board.
    """
    boxes = []
    depth = max(g.back_x - g.front_x, 0.05)
    x_centre = (g.front_x + g.back_x) / 2
    width = max(g.y_max - g.y_min, 0.05)
    y_centre = (g.y_min + g.y_max) / 2
    for i, level in enumerate(g.levels):
        top = level.height - p.board_sink
        boxes.append(
            Box(
                id=f"{namespace}board_{i + 1}",
                center=(x_centre, y_centre, top - level.thickness / 2),
                size=(depth, width, level.thickness),
            )
        )
    bottom = g.levels[0].height - g.levels[0].thickness
    if g.top_z:
        top = g.top_z + p.default_thickness
        boxes.append(
            Box(
                id=f"{namespace}top",
                center=(x_centre, y_centre, g.top_z + p.default_thickness / 2),
                size=(depth, width, p.default_thickness),
            )
        )
    else:
        top = g.levels[-1].height + p.default_thickness
    height = top - bottom
    z_centre = (top + bottom) / 2
    if g.has_left_wall:
        boxes.append(
            Box(
                id=f"{namespace}wall_left",
                center=(x_centre, g.y_min - p.wall_thickness / 2, z_centre),
                size=(depth, p.wall_thickness, height),
            )
        )
    if g.has_right_wall:
        boxes.append(
            Box(
                id=f"{namespace}wall_right",
                center=(x_centre, g.y_max + p.wall_thickness / 2, z_centre),
                size=(depth, p.wall_thickness, height),
            )
        )
    if g.has_back_wall:
        boxes.append(
            Box(
                id=f"{namespace}wall_back",
                center=(g.back_x + p.wall_thickness / 2, y_centre, z_centre),
                size=(p.wall_thickness, width, height),
            )
        )
    return boxes


# ======================================================================
# Using the result
# ======================================================================


def level_for_height(z: float, heights: Sequence[float], down_tolerance: float = 0.05):
    """Height of the level a point at ``z`` rests on: the highest surface below it.

    ``down_tolerance`` absorbs depth noise that puts an object's point slightly under
    its own surface. Returns None below the lowest level.
    """
    resting = [h for h in sorted(heights) if h - down_tolerance <= z]
    return resting[-1] if resting else None


__all__ = [
    "Box",
    "Frame",
    "ShelfGeometry",
    "ShelfLevel",
    "ShelfNotFound",
    "ShelfParams",
    "collision_boxes",
    "detect_shelf",
    "level_for_height",
]

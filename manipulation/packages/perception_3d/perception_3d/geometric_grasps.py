from collections import Counter
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import scipy.ndimage as ndi
from scipy.spatial.transform import Rotation

from frida_constants.manipulation_constants import (
    BOX_NAMES,
    CYLINDRICAL_NAMES,
    FLAT_OBJECT_NAMES,
    GRASP_CLASS_BOX,
    GRASP_CLASS_CYLINDRICAL,
    GRASP_CLASS_FLAT,
    GRASP_CLASS_GENERIC,
    GRASP_CLASS_PEAK,
    GRASP_CLASS_RIM,
    GRASP_CLASS_ROUND,
    PEAK_NAMES,
    RIM_NAMES,
    ROUND_NAMES,
)

GRASP_CLASS_OBJECTS = {
    GRASP_CLASS_FLAT: FLAT_OBJECT_NAMES,
    GRASP_CLASS_RIM: RIM_NAMES,
    GRASP_CLASS_PEAK: PEAK_NAMES,
    GRASP_CLASS_BOX: BOX_NAMES,
    GRASP_CLASS_CYLINDRICAL: CYLINDRICAL_NAMES,
    GRASP_CLASS_ROUND: ROUND_NAMES,
}
OBJECT_GRASP_CLASS = {
    name.lower(): grasp_class
    for grasp_class, names in GRASP_CLASS_OBJECTS.items()
    for name in names
}
assert len(OBJECT_GRASP_CLASS) == sum(
    map(len, GRASP_CLASS_OBJECTS.values())
), "an object is listed under two grasp classes"
DEFORMABLE_OBJECTS = frozenset({"clothes", "bread", "chip_bag", "towel", "plush_toy"})

GRIPPER_MAX_APERTURE = 0.09
GRIPPER_FINGER_LENGTH = 0.085
GRIPPER_PALM_TO_FLANGE = 0.085
GRIPPER_HALF_THICKNESS = 0.03
GRIPPER_REACH = GRIPPER_FINGER_LENGTH + GRIPPER_PALM_TO_FLANGE

MIN_POINTS_FOR_PCA = 10
ELONGATION_MIN_RATIO = 2.0

FLAT_TABLE_PERCENTILE = 85
FLAT_HEIGHT_BAND = (0.001, 0.05)
FLAT_HANDLE_FRACTION = 0.60
FLAT_SURFACE_OFFSET = 0.003

FLOOR_PERCENTILE = 5
FLOOR_MARGIN = 0.05
SUPPORT_MARGIN = 0.005
TOP_PERCENTILE = 90

RIM_TOP_BAND = 0.03
RIM_NEAR_FRACTION = 0.05

SOLID_SUPPORT_MARGIN = 0.01
SOLID_TOP_PERCENTILE = 99
SOLID_MAX_POINTS = 1000
FIT_STEP_DEG = 5
APERTURE_TOLERANCE = 0.005
GRASP_PAST_CENTER = 0.015
TOP_VISIBLE_MARGIN = 0.03

DEPTH_JUMP = 0.01
NEIGHBOUR_PAD = 0.5
OBSTACLE_MIN_POINTS = 10
GRIPPER_FINGER_THICKNESS = 0.01
APPROACH_CLEARANCE = 0.10
ANGLE_CUTOFF_DEG = 45

TABLE_CLEARANCE = GRIPPER_FINGER_LENGTH * 0.15
CAMERA_RADIUS = 0.09
ZED_OFFSET = np.array([-0.096, 0.0, 0.041])  # copy of pick.py's, keep equal
CAMERA_CLEARANCE_SCALE = 0.03
MOVE_SCALE = 0.3

PEAK_GRID_RES = 0.05
PEAK_NBR = 3

HOLLOW_GRID_RES = 0.02
HOLLOW_MIN_DEPTH = 0.03
FLAT_MAX_HEIGHT = 0.03
AXISYMMETRIC_MAX_ELONGATION = 1.5
BALL_ASPECT = (0.7, 1.4)

TOP_DOWN = np.array([0.0, 0.0, -1.0])
DEFAULT_CLOSING_AXIS = np.array([0.0, -1.0, 0.0])

TILT_NAMES = {0: "top", 45: "diagonal", 90: "front"}


class GraspRejected(Exception):
    pass


def require(condition, reason: str) -> None:
    if not condition:
        raise GraspRejected(reason)


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Scene:
    depth: np.ndarray
    intrinsics: Intrinsics
    cam_to_base: np.ndarray
    gripper_to_base: np.ndarray
    bbox: tuple

    @property
    def roi(self) -> np.ndarray:
        xmin, ymin, xmax, ymax = self.bbox
        return self.depth[ymin:ymax, xmin:xmax]

    @property
    def valid(self) -> np.ndarray:
        roi = self.roi
        return (roi > 0) & ~np.isnan(roi)

    def deproject(self, u, v, z) -> np.ndarray:
        u, v, z = map(np.atleast_1d, (u, v, z))
        k = self.intrinsics
        cam = np.column_stack(
            ((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z, np.ones_like(z))
        )
        return (self.cam_to_base @ cam.T).T[:, :3]

    def points(self, mask: np.ndarray) -> np.ndarray:
        v, u = np.nonzero(mask)
        xmin, ymin = self.bbox[:2]
        points = self.deproject(u + xmin, v + ymin, self.roi[v, u])
        points = points[np.isfinite(points).all(axis=1)]
        require(len(points) >= MIN_POINTS_FOR_PCA, f"only {len(points)} valid points")
        return points


@dataclass(frozen=True)
class ObjectGeometry:
    support_z: float
    top_z: float
    width: float
    elongation: float
    is_hollow: bool

    @property
    def height(self) -> float:
        return self.top_z - self.support_z


@dataclass(frozen=True)
class Grasp:
    position: np.ndarray
    orientation: np.ndarray
    approach: str = "top"


def parse_bbox(xmin, ymin, xmax, ymax, shape) -> tuple:
    height, width = shape
    if xmax <= 1.0 and ymax <= 1.0:
        xmin, xmax = xmin * width, xmax * width
        ymin, ymax = ymin * height, ymax * height
    bbox = (
        int(max(0, xmin)),
        int(max(0, ymin)),
        int(min(width, xmax)),
        int(min(height, ymax)),
    )
    require(bbox[2] > bbox[0] and bbox[3] > bbox[1], f"degenerate bbox {bbox}")
    return bbox


def above_support(points: np.ndarray, margin: float) -> tuple:
    support_z = np.percentile(points[:, 2], FLOOR_PERCENTILE)
    above = points[points[:, 2] > support_z + margin]
    return support_z, above if len(above) >= MIN_POINTS_FOR_PCA else points


# defining the object is the biggest depth-continuous blob in the bbox; a
# neighbour touching it merges in, a bigger neighbour inside the bbox wins.
def segment(scene: Scene, margin: float) -> tuple:
    xmin, ymin, xmax, ymax = scene.bbox
    height, width = scene.depth.shape
    pad_x = int((xmax - xmin) * NEIGHBOUR_PAD)
    pad_y = int((ymax - ymin) * NEIGHBOUR_PAD)
    x0, y0 = max(0, xmin - pad_x), max(0, ymin - pad_y)
    x1, y1 = min(width, xmax + pad_x), min(height, ymax + pad_y)

    depth = scene.depth[y0:y1, x0:x1].astype(float)
    depth[~(depth > 0)] = np.nan
    v, u = np.nonzero(np.isfinite(depth))
    xyz = np.full(depth.shape + (3,), np.nan)
    xyz[v, u] = scene.deproject(u + x0, v + y0, depth[v, u])

    inside = np.zeros(depth.shape, dtype=bool)
    inside[ymin - y0 : ymax - y0, xmin - x0 : xmax - x0] = True
    in_bbox = xyz[inside & np.isfinite(xyz).all(axis=2)]
    require(len(in_bbox) >= MIN_POINTS_FOR_PCA, f"only {len(in_bbox)} valid points")
    support_z = np.percentile(in_bbox[:, 2], FLOOR_PERCENTILE)

    with np.errstate(invalid="ignore"):
        above = xyz[..., 2] > support_z + margin
        jump_v = np.abs(np.diff(depth, axis=0)) > DEPTH_JUMP
        jump_u = np.abs(np.diff(depth, axis=1)) > DEPTH_JUMP
    edges = np.zeros_like(above)
    edges[1:] |= jump_v
    edges[:-1] |= jump_v
    edges[:, 1:] |= jump_u
    edges[:, :-1] |= jump_u

    labels, _ = ndi.label(above & ~edges)
    counts = np.bincount(labels[inside])
    counts[0] = 0
    target = np.argmax(counts)
    if counts[target] < MIN_POINTS_FOR_PCA:
        support_z, points = above_support(in_bbox, margin)
        return support_z, points, np.empty((0, 3))
    sizes = np.bincount(labels.ravel())
    others = (labels > 0) & (labels != target) & (sizes[labels] >= OBSTACLE_MIN_POINTS)
    return support_z, xyz[labels == target], xyz[others]


def principal_axes(xy: np.ndarray) -> tuple:
    return np.linalg.eigh(np.cov((xy - xy.mean(axis=0)).T))


def elevation_grid(points: np.ndarray, resolution: float) -> tuple:
    origin = points[:, :2].min(axis=0)
    cells = ((points[:, :2] - origin) / resolution).astype(int)
    grid = np.full(cells.max(axis=0) + 1, -np.inf)
    np.maximum.at(grid, (cells[:, 0], cells[:, 1]), points[:, 2])
    return grid, origin


def spread(values: np.ndarray) -> float:
    return values.std() if len(values) > 2 else np.inf


def grasp_frame(approach: np.ndarray, closing_hint: np.ndarray) -> np.ndarray:
    z = approach / np.linalg.norm(approach)
    y = closing_hint - np.dot(closing_hint, z) * z
    require(np.linalg.norm(y) > 1e-6, "closing axis parallel to approach")
    y = y / np.linalg.norm(y)
    return Rotation.from_matrix(np.column_stack((np.cross(y, z), y, z))).as_quat()


def is_hollow(points: np.ndarray) -> bool:
    grid, _ = elevation_grid(points, HOLLOW_GRID_RES)
    if min(grid.shape) < 3:
        return False
    border = np.ones(grid.shape, dtype=bool)
    border[1:-1, 1:-1] = False
    filled = np.isfinite(grid)
    edge, inner = grid[border & filled], grid[~border & filled]
    return (
        len(edge) > 0
        and len(inner) > 0
        and np.median(edge) - np.median(inner) > HOLLOW_MIN_DEPTH
    )


def describe(scene: Scene) -> ObjectGeometry:
    support_z, points, _ = segment(scene, SUPPORT_MARGIN)
    xy = points[:, :2]
    eigenvalues, eigenvectors = principal_axes(xy)
    low, high = np.percentile((xy - xy.mean(axis=0)) @ eigenvectors[:, 0], [5, 95])
    return ObjectGeometry(
        support_z=support_z,
        top_z=np.percentile(points[:, 2], TOP_PERCENTILE),
        width=high - low,
        elongation=eigenvalues[1] / max(eigenvalues[0], 1e-9),
        is_hollow=is_hollow(points),
    )


def classify(geometry: ObjectGeometry) -> str:
    if geometry.height < FLAT_MAX_HEIGHT:
        return GRASP_CLASS_FLAT
    if geometry.is_hollow:
        return GRASP_CLASS_RIM
    if geometry.elongation < AXISYMMETRIC_MAX_ELONGATION:
        low, high = BALL_ASPECT
        if low * geometry.width <= geometry.height <= high * geometry.width:
            return GRASP_CLASS_ROUND
        return GRASP_CLASS_CYLINDRICAL
    return GRASP_CLASS_BOX


def flat(scene: Scene, grasp_class: Optional[str] = None) -> Grasp:
    roi, valid = scene.roi, scene.valid
    require(np.count_nonzero(valid) >= MIN_POINTS_FOR_PCA, "too few depth pixels")
    table_depth = np.percentile(roi[valid], FLAT_TABLE_PERCENTILE)
    nearest, farthest = FLAT_HEIGHT_BAND
    on_table = valid & (roi < table_depth - nearest) & (roi > table_depth - farthest)
    labels, count = ndi.label(on_table)
    require(count > 0, "nothing above the table")
    points = scene.points(labels == np.argmax(np.bincount(labels.ravel())[1:]) + 1)

    xy = points[:, :2]
    center = xy.mean(axis=0)
    eigenvalues, eigenvectors = principal_axes(xy)
    require(
        eigenvalues[0] > 1e-9
        and eigenvalues[1] / eigenvalues[0] >= ELONGATION_MIN_RATIO,
        "not elongated enough for a reliable long axis",
    )
    long_axis = np.append(eigenvectors[:, 1], 0.0)
    long_axis /= np.linalg.norm(long_axis)

    along = (xy - center) @ long_axis[:2]
    across = (xy - center) @ np.array([-long_axis[1], long_axis[0]])
    positive = along >= 0
    sign = 1.0 if spread(across[positive]) <= spread(across[~positive]) else -1.0
    handle = positive if sign > 0 else ~positive
    reach = np.percentile(np.abs(along[handle]), 90) if handle.any() else 0.0
    grasp_xy = center + sign * FLAT_HANDLE_FRACTION * reach * long_axis[:2]

    xmin, ymin, xmax, ymax = scene.bbox
    table_z = scene.deproject((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, table_depth)
    return Grasp(
        np.array([*grasp_xy, table_z[0, 2] + FLAT_SURFACE_OFFSET]),
        grasp_frame(TOP_DOWN, np.cross(TOP_DOWN, long_axis)),
    )


def rim(scene: Scene, grasp_class: Optional[str] = None) -> Grasp:
    _, points = above_support(scene.points(scene.valid), FLOOR_MARGIN)
    top_z = np.percentile(points[:, 2], TOP_PERCENTILE)
    ring = points[points[:, 2] > top_z - RIM_TOP_BAND]
    ring = ring if len(ring) >= MIN_POINTS_FOR_PCA else points

    nearest = np.argsort(np.hypot(ring[:, 0], ring[:, 1]))
    count = min(max(MIN_POINTS_FOR_PCA, int(RIM_NEAR_FRACTION * len(ring))), len(ring))
    near_rim = np.median(ring[nearest[:count]], axis=0)

    radial = np.array([near_rim[0], near_rim[1], 0.0])
    require(np.linalg.norm(radial) > 1e-6, "rim point at the robot origin")
    return Grasp(near_rim, grasp_frame(TOP_DOWN, radial))


def peak(scene: Scene, grasp_class: Optional[str] = None) -> Grasp:
    floor_z, points = above_support(scene.points(scene.valid), FLOOR_MARGIN)
    grid, origin = elevation_grid(points, PEAK_GRID_RES)
    require(min(grid.shape) >= PEAK_NBR, "container too small to search for peaks")
    filled = np.isfinite(grid)
    require(np.count_nonzero(filled) >= MIN_POINTS_FOR_PCA, "too few occupied cells")

    is_peak = (
        filled
        & (grid >= ndi.maximum_filter(grid, size=PEAK_NBR))
        & (grid > floor_z + FLOOR_MARGIN)
    )
    cells = np.argwhere(is_peak)
    require(len(cells) > 0, "no peak above the floor")

    peaks_xy = origin + (cells + 0.5) * PEAK_GRID_RES
    center = np.median(points[:, :2], axis=0)
    best = np.argmin(np.linalg.norm(peaks_xy - center, axis=1))

    return Grasp(
        np.array([*peaks_xy[best], grid[tuple(cells[best])]]),
        grasp_frame(TOP_DOWN, DEFAULT_CLOSING_AXIS),
    )


def fit_box(points: np.ndarray) -> tuple:
    angles = np.radians(np.arange(0, 180, FIT_STEP_DEG))
    axes = np.column_stack((np.cos(angles), np.sin(angles)))
    low, high = np.percentile(points[:, :2] @ axes.T, [1, 99], axis=0)
    k = np.argmin(high - low)
    j = (k + len(angles) // 2) % len(angles)
    center = axes[k] * (low[k] + high[k]) / 2 + axes[j] * (low[j] + high[j]) / 2
    return center, axes[k], axes[j], high[k] - low[k], high[j] - low[j]


def turn(vector: np.ndarray, degrees: float) -> np.ndarray:
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]])


# (label, tilt from vertical, horizontal approach side, closing axis) per shape.
def approaches(grasp_class, away, short_axis, long_axis) -> list:
    tangent = np.array([-away[1], away[0]])
    tops = [(f"top {t}", 0, away, turn(tangent, t)) for t in (0, 45, 90, 135)]
    if grasp_class == GRASP_CLASS_ROUND:
        return tops
    if grasp_class == GRASP_CLASS_CYLINDRICAL:
        return tops + [
            (f"{TILT_NAMES[tilt]} {az:+d}", tilt, turn(away, az), turn(tangent, az))
            for tilt in (45, 90)
            for az in (-30, 0, 30)
        ]
    tilts = (0, 45, 90) if grasp_class == GRASP_CLASS_BOX else range(0, 91, 15)
    found = []
    for name, closing, other in (
        ("short", short_axis, long_axis),
        ("long", long_axis, short_axis),
    ):
        side = other if other @ away >= 0 else -other
        found += [
            (f"{TILT_NAMES.get(t, f'tilt {t}')} {name}", t, side, closing)
            for t in tilts
        ]
    return found


def solid(scene: Scene, grasp_class: str) -> Grasp:
    support_z, points, obstacles = segment(scene, SOLID_SUPPORT_MARGIN)
    points = points[:: max(1, len(points) // SOLID_MAX_POINTS)]
    top_z = np.percentile(points[:, 2], SOLID_TOP_PERCENTILE)
    center, short_axis, long_axis, short, long = fit_box(points)
    require(np.linalg.norm(center) > 1e-6, "object at the robot origin")
    away = center / np.linalg.norm(center)
    middle = np.array([*center, (support_z + top_z) / 2])
    top_tip_z = max(top_z - 0.85 * GRIPPER_FINGER_LENGTH, support_z + TABLE_CLEARANCE)

    camera = scene.cam_to_base[:3, 3]
    view = (center - camera[:2]) / max(np.linalg.norm(center - camera[:2]), 1e-6)
    sees_top = camera[2] > top_z + TOP_VISIBLE_MARGIN and scene.bbox[1] > 0

    # neighbours as gaps to the fitted rectangle and bearings from
    # its centre, blocked within a fixed cone; MoveIt's octomap is the real check.
    rel = obstacles[:, :2] - center
    gap = np.hypot(
        np.maximum(np.abs(rel @ long_axis) - long / 2, 0),
        np.maximum(np.abs(rel @ short_axis) - short / 2, 0),
    )
    bearing = rel / np.maximum(np.linalg.norm(rel, axis=1, keepdims=True), 1e-6)
    cone = np.cos(np.radians(ANGLE_CUTOFF_DEG))

    grasps, rejected = [], Counter()
    for label, tilt, side, closing in approaches(
        grasp_class, away, short_axis, long_axis
    ):
        if not sees_top and abs(closing @ view) > np.sin(np.radians(20)):
            rejected["top hidden, closing across unseen depth"] += 1
            continue
        lean = np.radians(tilt)
        approach = np.cos(lean) * TOP_DOWN + np.sin(lean) * np.append(side, 0.0)
        across = np.append(closing, 0.0)
        rotation = np.column_stack((np.cross(across, approach), across, approach))
        tip = (
            np.array([*center, top_tip_z])
            if tilt == 0
            else middle + approach * GRASP_PAST_CENTER
        )

        local = (points - tip) @ rotation
        slab = np.abs(local[:, 0]) <= GRIPPER_HALF_THICKNESS
        between = slab & (local[:, 2] <= 0) & (local[:, 2] >= -GRIPPER_FINGER_LENGTH)
        if np.count_nonzero(between) < MIN_POINTS_FOR_PCA:
            rejected["fingers miss the object"] += 1
            continue
        low, high = np.percentile(local[between, 1], [1, 99])
        tip = tip + rotation[:, 1] * (low + high) / 2
        width = high - low
        if tilt:  # a side band misses the top face, so trust the fitted footprint
            width = max(
                width,
                abs(closing @ long_axis) * long + abs(closing @ short_axis) * short,
            )
        palm = (
            slab
            & (local[:, 2] < -GRIPPER_FINGER_LENGTH)
            & (np.abs(local[:, 1] - (low + high) / 2) <= GRIPPER_MAX_APERTURE / 2)
        )
        lowest = tip[2] - GRIPPER_HALF_THICKNESS * abs(rotation[2, 0])
        near = obstacles[:, 2] >= lowest
        finger_room = (GRIPPER_MAX_APERTURE - width) / 2 + GRIPPER_FINGER_THICKNESS

        if lowest < support_z + TABLE_CLEARANCE - 1e-3:
            rejected["gripper hits the table"] += 1
        elif width > GRIPPER_MAX_APERTURE + APERTURE_TOLERANCE:
            rejected["wider than the gripper opens"] += 1
        elif np.count_nonzero(palm) >= MIN_POINTS_FOR_PCA:
            rejected["palm hits the object"] += 1
        elif np.any(near & (gap < finger_room) & (np.abs(bearing @ closing) >= cone)):
            rejected["neighbour beside the fingers"] += 1
        elif tilt and np.any(
            near & (gap < APPROACH_CLEARANCE) & (bearing @ -side >= cone)
        ):
            rejected["neighbour in the approach path"] += 1
        else:
            for sign, suffix in ((1, ""), (-1, " flipped")):
                grasp = Grasp(
                    tip, grasp_frame(approach, sign * rotation[:, 1]), label + suffix
                )
                value = score(scene, grasp, support_z)
                if value > 0:
                    grasps.append((value, grasp))
                else:
                    rejected["camera hits the table"] += 1

    if not grasps:
        raise GraspRejected(
            "; ".join(f"{reason} x{count}" for reason, count in rejected.most_common())
        )
    return max(grasps, key=lambda scored: scored[0])[1]


def score(scene: Scene, grasp: Grasp, support_z: float) -> float:
    rotation = Rotation.from_quat(grasp.orientation).as_matrix()
    flange = grasp.position - rotation[:, 2] * GRIPPER_REACH

    camera_z = flange[2] + (rotation @ ZED_OFFSET)[2]
    clearance = max(camera_z - support_z - CAMERA_RADIUS, 0.0)
    camera = 1 - np.exp(-clearance / CAMERA_CLEARANCE_SCALE)

    current = scene.gripper_to_base
    moved = np.linalg.norm(flange - current[:3, 3])
    turned = Rotation.from_matrix(current[:3, :3].T @ rotation).magnitude()
    move = np.exp(-(moved + GRIPPER_REACH * turned) / MOVE_SCALE)

    return camera * move


RECIPES: dict[str, Callable[[Scene, str], Grasp]] = {
    GRASP_CLASS_FLAT: flat,
    GRASP_CLASS_RIM: rim,
    GRASP_CLASS_PEAK: peak,
    GRASP_CLASS_BOX: solid,
    GRASP_CLASS_CYLINDRICAL: solid,
    GRASP_CLASS_ROUND: solid,
    GRASP_CLASS_GENERIC: solid,
}


def grasp_for(scene: Scene, grasp_class: str) -> Grasp:
    recipe = RECIPES.get(grasp_class)
    require(recipe is not None, f"no grasp recipe for '{grasp_class}' yet")
    return recipe(scene, grasp_class)

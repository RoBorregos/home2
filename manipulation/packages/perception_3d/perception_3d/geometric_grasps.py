from collections import Counter
from dataclasses import dataclass, replace
from typing import Callable

import numpy as np
import scipy.ndimage as ndi
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from frida_constants.manipulation_constants import (
    FLAT_OBJECT_NAMES,
    GRASP_CLASS_BOX,
    GRASP_CLASS_CYLINDRICAL,
    GRASP_CLASS_FLAT,
    GRASP_CLASS_PEAK,
    GRASP_CLASS_RIM,
    GRASP_CLASS_ROUND,
    GRASP_CLASS_TRASH,
    GRIPPER_BODY_CORNERS,
    GRIPPER_FINGER_LENGTH,
    GRIPPER_FINGER_THICKNESS,
    GRIPPER_HALF_THICKNESS,
    GRIPPER_MAX_APERTURE,
    GRIPPER_REACH,
    PEAK_NAMES,
    PLACE_TRASH_HEIGHT_OFFSET,
    PRE_GRASP_DISTANCE,
    RIM_NAMES,
    TOP_GRASP_DEPTH,
    TRASH_BIN_NAME,
)

GRASP_CLASS_OBJECTS = {
    GRASP_CLASS_FLAT: FLAT_OBJECT_NAMES,
    GRASP_CLASS_RIM: RIM_NAMES,
    GRASP_CLASS_PEAK: PEAK_NAMES,
    GRASP_CLASS_TRASH: [TRASH_BIN_NAME],
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

MIN_POINTS_FOR_PCA = 10
ELONGATION_MIN_RATIO = 2.0

FLAT_TABLE_PERCENTILE = 85
FLAT_HEIGHT_BAND = (0.001, 0.05)
FLAT_HANDLE_FRACTION = 0.60
FLAT_SURFACE_OFFSET = 0.003

FLOOR_PERCENTILE = 5
FLOOR_MARGIN = 0.05
TOP_PERCENTILE = 90

RIM_TOP_BAND = 0.03
RIM_NEAR_FRACTION = 0.05

TRASH_TOP_BBOX_FRACTION = 0.5

SOLID_SUPPORT_MARGIN = 0.01
SOLID_TOP_PERCENTILE = 99
SOLID_MAX_POINTS = 1000
SHAPE_TOLERANCE = {"sphere": 0.003, "cylinder": 0.004, "box": 0.0045}
SHAPE_ITERATIONS = 300
SHAPE_RADIUS = (0.02, 0.08)
BOX_AXES_MIN = 0.71
SPHERE_MAX_HEIGHT = 0.09
NORMAL_NEIGHBOURS = 12
SIDE_NORMAL_Z = 0.35
AXIS_COS = np.cos(np.radians(20))
FIT_STEP_DEG = 5
APERTURE_TOLERANCE = 0.005
GRASP_PAST_CENTER = 0.015
TOP_VISIBLE_MARGIN = 0.03

DEPTH_JUMP = 0.01
NEIGHBOUR_PAD = 0.5
OBSTACLE_MIN_POINTS = 10
SELF_MARGIN = 0.01

TABLE_CLEARANCE = 0.005
CYLINDER_TOP_DEPTH = 0.09
CYLINDER_PAST_CENTER = 0.035  # the pads sit behind the tip, so reach past the axis
PALM_OFFSET = 0.073
PALM_ABOVE_TIP = GRIPPER_REACH - PALM_OFFSET
GRIPPER_BODY = np.array(GRIPPER_BODY_CORNERS)

FINGER_SWEEP = np.array(
    [
        [
            -GRIPPER_HALF_THICKNESS,
            -GRIPPER_MAX_APERTURE / 2 - GRIPPER_FINGER_THICKNESS,
            -GRIPPER_FINGER_LENGTH - PRE_GRASP_DISTANCE,
        ],
        [
            GRIPPER_HALF_THICKNESS,
            GRIPPER_MAX_APERTURE / 2 + GRIPPER_FINGER_THICKNESS,
            0.0,
        ],
    ]
)
BODY_SWEEP = GRIPPER_BODY - [[0.0, 0.0, PRE_GRASP_DISTANCE], [0.0, 0.0, 0.0]]
BODY_MIN_CLEARANCE = 0.02
BODY_CLEARANCE_SCALE = 0.01
SPARE_SCALE = 0.02
ARM_SCALE = 0.015
MOVE_SCALE = 0.55

PEAK_GRID_RES = 0.05
PEAK_NBR = 3

TOP_DOWN = np.array([0.0, 0.0, -1.0])
DEFAULT_CLOSING_AXIS = np.array([0.0, -1.0, 0.0])

TILT_NAMES = {0: "top", 45: "diagonal", 90: "front"}
SHAPE_CLASS = {
    "sphere": GRASP_CLASS_ROUND,
    "cylinder": GRASP_CLASS_CYLINDRICAL,
    "box": GRASP_CLASS_BOX,
}


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
    """One depth frame with its bbox and the transforms to work in link_base."""

    depth: np.ndarray
    intrinsics: Intrinsics
    cam_to_base: np.ndarray
    gripper_to_base: np.ndarray
    bbox: tuple
    cylinder_diagonal: bool = False

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
class Grasp:
    position: np.ndarray
    orientation: np.ndarray
    approach: str = "top"
    score: float = 0.0


@dataclass(frozen=True)
class Fit:
    kind: str
    center: np.ndarray
    yaw: float
    size: tuple


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


def segment(scene: Scene, margin: float) -> tuple:
    # Object = blob with most pixels in the bbox; everything else is an obstacle.
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
    ispart = (labels == target) & inside
    sizes = np.bincount(labels.ravel())
    others = (labels > 0) & ~ispart & (sizes[labels] >= OBSTACLE_MIN_POINTS)
    return support_z, xyz[ispart], xyz[others]


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


def flat(scene: Scene) -> tuple:
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
    return None, [
        Grasp(
            np.array([*grasp_xy, table_z[0, 2] + FLAT_SURFACE_OFFSET]),
            grasp_frame(TOP_DOWN, np.cross(TOP_DOWN, long_axis)),
        )
    ]


def rim(scene: Scene) -> tuple:
    _, points = above_support(scene.points(scene.valid), FLOOR_MARGIN)
    top_z = np.percentile(points[:, 2], TOP_PERCENTILE)
    ring = points[points[:, 2] > top_z - RIM_TOP_BAND]
    ring = ring if len(ring) >= MIN_POINTS_FOR_PCA else points

    nearest = np.argsort(np.hypot(ring[:, 0], ring[:, 1]))
    count = min(max(MIN_POINTS_FOR_PCA, int(RIM_NEAR_FRACTION * len(ring))), len(ring))
    near_rim = np.median(ring[nearest[:count]], axis=0)

    radial = np.array([near_rim[0], near_rim[1], 0.0])
    require(np.linalg.norm(radial) > 1e-6, "rim point at the robot origin")
    return None, [Grasp(near_rim, grasp_frame(TOP_DOWN, radial))]


def peak(scene: Scene) -> tuple:
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

    return None, [
        Grasp(
            np.array([*peaks_xy[best], grid[tuple(cells[best])]]),
            grasp_frame(TOP_DOWN, DEFAULT_CLOSING_AXIS),
        )
    ]


def trash_bin(scene: Scene) -> tuple:
    top_half = scene.valid
    top_half[int(len(top_half) * TRASH_TOP_BBOX_FRACTION) :] = False
    _, points = above_support(scene.points(top_half), FLOOR_MARGIN)
    center_x, center_y = np.median(points[:, :2], axis=0)
    drop_z = np.percentile(points[:, 2], TOP_PERCENTILE) + PLACE_TRASH_HEIGHT_OFFSET
    return None, [
        Grasp(
            np.array([center_x, center_y, drop_z]),
            grasp_frame(TOP_DOWN, DEFAULT_CLOSING_AXIS),
            "place",
        )
    ]


def fit_box(points: np.ndarray) -> tuple:
    angles = np.radians(np.arange(0, 180, FIT_STEP_DEG))
    axes = np.column_stack((np.cos(angles), np.sin(angles)))
    low, high = np.percentile(points[:, :2] @ axes.T, [1, 99], axis=0)
    k = np.argmin(high - low)
    j = (k + len(angles) // 2) % len(angles)
    center = axes[k] * (low[k] + high[k]) / 2 + axes[j] * (low[j] + high[j]) / 2
    return center, axes[k], axes[j], high[k] - low[k], high[j] - low[j]


def circumsphere(sample: np.ndarray) -> tuple:
    a = 2 * (sample[:, 1:] - sample[:, :1])
    b = (sample[:, 1:] ** 2).sum(axis=2) - (sample[:, :1] ** 2).sum(axis=2)
    ok = np.abs(np.linalg.det(a)) > 1e-12
    centers = np.linalg.solve(a[ok], b[ok][..., None])[..., 0]
    radii = np.linalg.norm(sample[ok, 0] - centers, axis=1)
    fits = (radii >= SHAPE_RADIUS[0]) & (radii <= SHAPE_RADIUS[1])
    return centers[fits], radii[fits]


def shape_fractions(points: np.ndarray) -> dict:
    """Share of points explained by the best sphere, cylinder and box."""
    rng = np.random.default_rng(0)
    points = points - points.mean(axis=0)
    xy, z = points[:, :2], points[:, 2]
    from_top = np.abs(z - np.percentile(z, SOLID_TOP_PERCENTILE))

    def draw(k):
        return points[:, :k][rng.integers(len(points), size=(SHAPE_ITERATIONS, k + 1))]

    centers, radii = circumsphere(draw(3))
    away = np.linalg.norm(points[None] - centers[:, None], axis=2)
    sphere = (np.abs(away - radii[:, None]) < SHAPE_TOLERANCE["sphere"]).mean(axis=1)

    centers, radii = circumsphere(draw(2))
    away = np.linalg.norm(xy[None] - centers[:, None], axis=2)
    tolerance = SHAPE_TOLERANCE["cylinder"]
    side = np.abs(away - radii[:, None]) < tolerance
    cap = (from_top < tolerance) & (away <= radii[:, None])
    cylinder = (side | cap).mean(axis=1)

    center, axis_a, axis_b, size_a, size_b = fit_box(points)
    out = np.maximum(
        np.abs((xy - center) @ axis_a) - size_a / 2,
        np.abs((xy - center) @ axis_b) - size_b / 2,
    )
    tolerance = SHAPE_TOLERANCE["box"]
    box = (np.abs(out) < tolerance) | ((from_top < tolerance) & (out <= 0))
    return {
        "sphere": max(sphere, default=0.0),
        "cylinder": max(cylinder, default=0.0),
        "box": box.mean(),
    }


def box_axes_share(points: np.ndarray) -> float:
    """Share of side-surface normals aligned with the box axes (NaN if few)."""
    points = points - points.mean(axis=0)
    if len(points) < NORMAL_NEIGHBOURS:
        return np.nan
    near = cKDTree(points).query(points, k=NORMAL_NEIGHBOURS)[1]
    patch = points[near] - points[near].mean(axis=1, keepdims=True)
    normal = np.linalg.eigh(np.einsum("nki,nkj->nij", patch, patch))[1][:, :, 0]
    side = normal[np.abs(normal[:, 2]) < SIDE_NORMAL_Z, :2]
    if len(side) < MIN_POINTS_FOR_PCA:
        return np.nan
    side = side / np.linalg.norm(side, axis=1, keepdims=True)
    _, axis_a, axis_b, _, _ = fit_box(points)
    return np.mean(
        (np.abs(side @ axis_a) >= AXIS_COS) | (np.abs(side @ axis_b) >= AXIS_COS)
    )


def estimate_shape(points: np.ndarray, support_z: float) -> str:
    """Pick sphere, cylinder or box from the fit shares, normals and height."""
    fractions = shape_fractions(points)
    share = box_axes_share(points)
    if share >= BOX_AXES_MIN:
        fractions["cylinder"] = -1.0
    elif share < BOX_AXES_MIN:
        fractions["box"] = -1.0
    if (
        np.percentile(points[:, 2], SOLID_TOP_PERCENTILE) - support_z
        > SPHERE_MAX_HEIGHT
    ):
        fractions["sphere"] = -1.0
    return max(fractions, key=fractions.get)


def turn(vector: np.ndarray, degrees: float) -> np.ndarray:
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]])


def approaches(grasp_class, away, short_axis, long_axis, diagonal=False) -> list:
    """Candidate (label, tilt, approach side, closing axis) for a grasp class."""
    tangent = np.array([-away[1], away[0]])
    tops = [(f"top {t}", 0, away, turn(tangent, t)) for t in (0, 45, 90, 135)]
    if grasp_class == GRASP_CLASS_ROUND:
        return tops
    if grasp_class == GRASP_CLASS_CYLINDRICAL:
        return [(TILT_NAMES[45], 45, away, tangent)] if diagonal else tops
    found = []
    for name, closing, other in (
        ("short", short_axis, long_axis),
        ("long", long_axis, short_axis),
    ):
        side = other if other @ away >= 0 else -other
        found += [
            (f"{tilt_name} {name}", t, side, closing)
            for t, tilt_name in TILT_NAMES.items()
        ]
    return found


def pad_contact(center, normal, extent, bottom, span, tip, rotation) -> np.ndarray:
    s, z = np.meshgrid(
        np.linspace(-extent / 2, extent / 2, 9), np.linspace(bottom, bottom + span, 9)
    )
    section = np.column_stack((center + np.outer(s.ravel(), normal), z.ravel()))
    local = (section - tip) @ rotation
    on_pads = (
        (np.abs(local[:, 0]) <= GRIPPER_HALF_THICKNESS)
        & (local[:, 2] <= 0)
        & (local[:, 2] >= -GRIPPER_FINGER_LENGTH)
    )
    return section[on_pads]


def solid(scene: Scene, grasp_class: str, segmented: tuple) -> tuple:
    """Rank grasps on a segmented solid of the given class; returns (fit, grasps)."""
    support_z, points, obstacles = segmented
    points = points[:: max(1, len(points) // SOLID_MAX_POINTS)]
    top_z = np.percentile(points[:, 2], SOLID_TOP_PERCENTILE)
    center, short_axis, long_axis, short, long = fit_box(points)
    require(np.linalg.norm(center) > 1e-6, "object at the robot origin")
    away = center / np.linalg.norm(center)
    middle = np.array([*center, (support_z + top_z) / 2])
    top_tip_z = max(top_z - TOP_GRASP_DEPTH, support_z + TABLE_CLEARANCE)
    above_tip = GRIPPER_FINGER_LENGTH

    height = top_z - support_z
    yaw = np.arctan2(long_axis[1], long_axis[0])
    if grasp_class == GRASP_CLASS_ROUND:
        fit = Fit("sphere", middle, yaw, (height / 2,) * 3)
        bottom, span = middle[2], 0.0
        top_tip_z = max(top_tip_z, middle[2] - GRASP_PAST_CENTER)
    elif grasp_class == GRASP_CLASS_CYLINDRICAL:
        fit = Fit("cylinder", middle, yaw, (long / 2, long / 2, height))
        bottom, span = support_z, height
        top_tip_z = max(top_z - CYLINDER_TOP_DEPTH, support_z + TABLE_CLEARANCE)
        above_tip = PALM_ABOVE_TIP
    else:
        fit = Fit("box", middle, yaw, (long, short, height))
        bottom, span = support_z, height

    camera = scene.cam_to_base[:3, 3]
    view = (center - camera[:2]) / max(np.linalg.norm(center - camera[:2]), 1e-6)
    sees_top = camera[2] > top_z + TOP_VISIBLE_MARGIN and scene.bbox[1] > 0

    # obstacle points this close to the fitted footprint are the object's own
    # surface cut off by a depth edge, not a neighbour
    rel = obstacles[:, :2] - center
    gap = np.hypot(
        np.maximum(np.abs(rel @ long_axis) - long / 2, 0),
        np.maximum(np.abs(rel @ short_axis) - short / 2, 0),
    )
    obstacles = obstacles[gap >= SELF_MARGIN]

    tilted_past = (
        CYLINDER_PAST_CENTER
        if grasp_class == GRASP_CLASS_CYLINDRICAL
        else GRASP_PAST_CENTER
    )
    grasps, costs, rejected = [], [], Counter()
    for label, tilt, side, closing in approaches(
        grasp_class, away, short_axis, long_axis, scene.cylinder_diagonal
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
            else middle + approach * tilted_past
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
            if fit.kind == "cylinder":
                width = max(width, long)
            else:
                width = max(
                    width,
                    abs(closing @ long_axis) * long + abs(closing @ short_axis) * short,
                )
        palm = (
            slab
            & (local[:, 2] < -above_tip)
            & (np.abs(local[:, 1] - (low + high) / 2) <= GRIPPER_MAX_APERTURE / 2)
        )
        lowest = tip[2] - GRIPPER_HALF_THICKNESS * abs(rotation[2, 0])
        in_fingers = count_inside((obstacles - tip) @ rotation, FINGER_SWEEP)
        normal = np.array([-closing[1], closing[0]])
        extent = abs(normal @ long_axis) * long + abs(normal @ short_axis) * short
        held = pad_contact(
            center,
            normal,
            extent if fit.kind == "box" else 0.0,
            bottom,
            span,
            tip,
            rotation,
        )

        if lowest < support_z + TABLE_CLEARANCE - 1e-3:
            rejected["gripper hits the table"] += 1
        elif width > GRIPPER_MAX_APERTURE + APERTURE_TOLERANCE:
            rejected["wider than the gripper opens"] += 1
        elif np.count_nonzero(palm) >= MIN_POINTS_FOR_PCA:
            rejected["palm hits the object"] += 1
        elif len(held) == 0:
            rejected["fingers miss the widest part"] += 1
        elif in_fingers >= OBSTACLE_MIN_POINTS:
            rejected["neighbour beside the fingers"] += 1
        else:
            arm = abs((held[:, :2].mean(axis=0) - center) @ normal)
            flange = tip - approach * GRIPPER_REACH
            for sign, suffix in ((1, ""), (-1, " flipped")):
                flipped = rotation * [sign, sign, 1]
                body_z = flange[2] + (flipped[2] * GRIPPER_BODY).min(axis=0).sum()
                in_body = count_inside((obstacles - flange) @ flipped, BODY_SWEEP)
                if body_z - support_z < BODY_MIN_CLEARANCE:
                    rejected["gripper body too close to the table"] += 1
                elif in_body >= OBSTACLE_MIN_POINTS:
                    rejected["neighbour hits the gripper body"] += 1
                else:
                    value = score(width, arm, body_z - support_z)
                    orientation = Rotation.from_matrix(flipped).as_quat()
                    grasps.append(Grasp(tip, orientation, label + suffix, value))
                    costs.append(move_cost(scene, flipped, flange))

    if not grasps:
        raise GraspRejected(
            "; ".join(f"{reason} x{count}" for reason, count in rejected.most_common())
        )
    cheapest = min(costs)  # the trip every candidate shares cancels out
    grasps = [
        replace(grasp, score=grasp.score * np.exp(-(cost - cheapest) / MOVE_SCALE))
        for grasp, cost in zip(grasps, costs)
    ]
    return fit, sorted(grasps, key=lambda grasp: grasp.score, reverse=True)


def measured_solid(scene: Scene) -> tuple:
    """Classify a solid by its measured shape, then grasp it."""
    segmented = segment(scene, SOLID_SUPPORT_MARGIN)
    support_z, points, _ = segmented
    shape = SHAPE_CLASS[
        estimate_shape(points[:: max(1, len(points) // SOLID_MAX_POINTS)], support_z)
    ]
    return (shape, *solid(scene, shape, segmented))


def count_inside(points: np.ndarray, box: np.ndarray) -> int:
    return np.count_nonzero(np.all((points >= box[0]) & (points <= box[1]), axis=1))


def score(width: float, arm: float, clearance: float) -> float:
    spare = GRIPPER_MAX_APERTURE + APERTURE_TOLERANCE - width
    opening = 1 - np.exp(-spare / SPARE_SCALE)
    centred = np.exp(-arm / ARM_SCALE)
    clear = 1 - np.exp(-clearance / BODY_CLEARANCE_SCALE)
    return opening * centred * clear


def move_cost(scene: Scene, rotation: np.ndarray, flange: np.ndarray) -> float:
    current = scene.gripper_to_base
    turned = Rotation.from_matrix(current[:3, :3].T @ rotation).magnitude()
    return np.linalg.norm(flange - current[:3, 3]) + GRIPPER_REACH * turned


RECIPES: dict[str, Callable[[Scene], tuple]] = {
    GRASP_CLASS_FLAT: flat,
    GRASP_CLASS_RIM: rim,
    GRASP_CLASS_PEAK: peak,
    GRASP_CLASS_TRASH: trash_bin,
}


def grasp_for(scene: Scene, grasp_class: str) -> tuple:
    recipe = RECIPES.get(grasp_class)
    require(recipe is not None, f"no grasp recipe for '{grasp_class}'")
    return recipe(scene)

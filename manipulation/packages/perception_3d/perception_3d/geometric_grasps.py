from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
import scipy.ndimage as ndi
from scipy.spatial.transform import Rotation

from frida_constants.manipulation_constants import (
    FLAT_OBJECT_NAMES,
    GRASP_CLASS_BOX,
    GRASP_CLASS_CYLINDRICAL,
    GRASP_CLASS_FLAT,
    GRASP_CLASS_HANDLE,
    GRASP_CLASS_PEAK,
    GRASP_CLASS_RIM,
    GRASP_CLASS_ROUND,
    PEAK_NAMES,
    RIM_NAMES,
)

GRASP_CLASS_OBJECTS = {
    GRASP_CLASS_FLAT: FLAT_OBJECT_NAMES,
    GRASP_CLASS_RIM: RIM_NAMES,
    GRASP_CLASS_PEAK: PEAK_NAMES,
    GRASP_CLASS_BOX: ["cornflakes", "cereal", "milk", "rubiks_cube"],
    GRASP_CLASS_CYLINDRICAL: [],
    GRASP_CLASS_ROUND: [],
    GRASP_CLASS_HANDLE: [],
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

GRIPPER_MAX_APERTURE = 0.095
GRIPPER_FINGER_LENGTH = 0.16

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

BOX_SUPPORT_MARGIN = 0.01

PEAK_GRID_RES = 0.05
PEAK_NBR = 3

HOLLOW_GRID_RES = 0.02
HOLLOW_MIN_DEPTH = 0.03
FLAT_MAX_HEIGHT = 0.03
AXISYMMETRIC_MAX_ELONGATION = 1.5
BALL_ASPECT = (0.7, 1.4)

TOP_DOWN = np.array([0.0, 0.0, -1.0])
DEFAULT_CLOSING_AXIS = np.array([0.0, -1.0, 0.0])


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
    support_z, points = above_support(scene.points(scene.valid), SUPPORT_MARGIN)
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
    require(
        geometry.width <= GRIPPER_MAX_APERTURE,
        f"{geometry.width:.3f} m wide, gripper opens {GRIPPER_MAX_APERTURE} m",
    )
    if geometry.elongation < AXISYMMETRIC_MAX_ELONGATION:
        low, high = BALL_ASPECT
        if low * geometry.width <= geometry.height <= high * geometry.width:
            return GRASP_CLASS_ROUND
        return GRASP_CLASS_CYLINDRICAL
    return GRASP_CLASS_BOX


def flat(scene: Scene) -> Grasp:
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


def rim(scene: Scene) -> Grasp:
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


def peak(scene: Scene) -> Grasp:
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


RECIPES: dict[str, Optional[Callable[[Scene], Grasp]]] = {
    GRASP_CLASS_FLAT: flat,
    GRASP_CLASS_RIM: rim,
    GRASP_CLASS_PEAK: peak,
    GRASP_CLASS_BOX: None,
    GRASP_CLASS_CYLINDRICAL: None,
    GRASP_CLASS_ROUND: None,
    GRASP_CLASS_HANDLE: None,
}


def grasp_for(scene: Scene, grasp_class: str) -> Grasp:
    recipe = RECIPES.get(grasp_class)
    require(recipe is not None, f"no grasp recipe for '{grasp_class}' yet")
    return recipe(scene)

"""Shelf support-surface detection from an organized RGB-D point cloud."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

from xarm_utils.shelf_geometry import (
    BOARD_THICKNESS,
    ShelfObservation,
    coherent_surfaces,
    describe_surfaces,
    detect_structure,
    inside_structure,
)
from xarm_utils.shelf_levels import (
    DEFAULT_MIN_LEVEL_SEPARATION,
    MAX_SHELF_LEVELS,
    MIN_SHELF_LEVELS,
    ShelfCalibrationError,
)

if TYPE_CHECKING:
    from sensor_msgs.msg import CameraInfo

DEFAULT_MAX_TILT_DEG = 15.0
NORMAL_RADIUS = 5
MIN_SURFACE_POINTS = 100
MIN_PATCH_POINTS = 24
MAX_PATCH_GAP = 0.25
MIN_SURFACE_WIDTH = 0.35
MIN_SURFACE_DEPTH = 0.08
MIN_RELATIVE_SURFACE_WIDTH = 0.65
MAX_PLANE_ERROR = 0.015
# Neighbouring image pixels must also be neighbours in 3-D. This prevents
# growth from jumping from a board to a distant, nearly coplanar surface.
MAX_GROWTH_STEP = 0.04
MAX_GROWTH_TILT_DEG = 60.0
MAX_GROWTH_PASSES = 2
CONNECTIVITY = np.ones((3, 3), dtype=bool)
# The floor is a wide horizontal plane too; anything this close to base_link is
# not a shelf level. Anything clearly below the floor means the TF is wrong.
MIN_LEVEL_HEIGHT = 0.05
INVALID_FRAME_HEIGHT = -0.05


@dataclass
class _Surface:
    mask: np.ndarray
    center: np.ndarray
    normal: np.ndarray
    point_count: int
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    # True when the measured plane is the underside of a board seen from below.
    # Its support surface sits BOARD_THICKNESS above the measured points.
    underside: bool = False

    @property
    def level(self) -> float:
        return float(self.center[2]) + (BOARD_THICKNESS if self.underside else 0.0)


def _rotation_matrix(quaternion) -> np.ndarray:
    values = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float
    )
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ShelfCalibrationError("Camera transform has an invalid quaternion")
    x, y, z, w = values / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _surface_normals(points: np.ndarray, camera_position: np.ndarray) -> np.ndarray:
    """Estimate camera-facing normals, rejecting one-sided depth discontinuities."""
    radius = NORMAL_RADIUS
    center = points[radius:-radius, radius:-radius]
    right = points[radius:-radius, 2 * radius :] - center
    left = center - points[radius:-radius, : -2 * radius]
    down = points[2 * radius :, radius:-radius] - center
    up = center - points[: -2 * radius, radius:-radius]
    normals = np.cross(right + left, down + up)
    lengths = np.linalg.norm(normals, axis=-1)
    valid = np.isfinite(center).all(axis=-1) & np.isfinite(normals).all(axis=-1)
    valid &= lengths > 1e-10
    # A large jump on only one side is a boundary, not a measured surface.
    for forward, backward in ((right, left), (down, up)):
        distances = np.stack(
            [np.linalg.norm(forward, axis=-1), np.linalg.norm(backward, axis=-1)]
        )
        valid &= distances.min(axis=0) >= 0.2 * distances.max(axis=0)
    normals = np.divide(
        normals,
        lengths[..., None],
        out=np.full_like(normals, np.nan),
        where=valid[..., None],
    )
    away = np.sum(normals * (camera_position - center), axis=-1) < 0
    normals[away] *= -1
    result = np.full_like(points, np.nan)
    result[radius:-radius, radius:-radius] = normals
    return result


def _fit_surface(points: np.ndarray, mask: np.ndarray, max_tilt_deg: float):
    cloud = points[mask]
    center = np.median(cloud, axis=0)
    centered = cloud - center
    eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered / len(cloud))
    normal = eigenvectors[:, 0]
    if normal[2] < 0:
        normal = -normal
    error = np.percentile(np.abs(centered @ normal), 90)
    if (
        eigenvalues[1] < 1e-8
        or normal[2] < np.cos(np.radians(max_tilt_deg))
        or error > MAX_PLANE_ERROR
    ):
        return None
    return _Surface(
        mask, center, normal, len(cloud), cloud.min(axis=0), cloud.max(axis=0)
    )


def _merge_surfaces(points: np.ndarray, surfaces: list[_Surface], max_tilt_deg: float):
    """Combine coplanar patches, including fragments separated by missing depth."""
    merged = []
    for surface in sorted(surfaces, key=lambda item: item.point_count, reverse=True):
        for index, other in enumerate(merged):
            offset = surface.center - other.center
            gap = np.maximum(
                np.maximum(surface.lower - other.upper, other.lower - surface.upper), 0
            )
            if (
                np.linalg.norm(gap[:2]) > MAX_PATCH_GAP
                or surface.normal @ other.normal < np.cos(np.radians(max_tilt_deg))
                or abs(offset @ surface.normal) > MAX_PLANE_ERROR
                or abs(offset @ other.normal) > MAX_PLANE_ERROR
            ):
                continue
            combined = _fit_surface(points, surface.mask | other.mask, max_tilt_deg)
            if combined is not None:
                merged[index] = combined
                break
        else:
            merged.append(surface)
    return merged


def _surface_extent(cloud: np.ndarray) -> np.ndarray:
    """Measure depth and width along the footprint's own principal axes."""
    xy = cloud[:, :2] - np.median(cloud[:, :2], axis=0)
    _, axes = np.linalg.eigh(xy.T @ xy / len(xy))
    lower, upper = np.percentile(xy @ axes, [5, 95], axis=0)
    return np.sort(upper - lower)


def _has_surface_extent(cloud: np.ndarray) -> bool:
    """Reject thin edges and small object tops, independent of shelf yaw."""
    depth, width = _surface_extent(cloud)
    return bool(depth >= MIN_SURFACE_DEPTH and width >= MIN_SURFACE_WIDTH)


def _grow_surface(
    points: np.ndarray,
    normals: np.ndarray,
    surface: _Surface,
    max_tilt_deg: float,
    neighbors: tuple[np.ndarray, np.ndarray] | None = None,
) -> _Surface:
    """Grow a measured plane through unreliable normals, without filling depth.

    Strict normals establish the seed. Growth uses point-to-plane distance;
    available normals veto vertical faces and undersides. An edge lattice
    preserves connectivity in both the image and the measured 3-D cloud.
    """
    height, width = surface.mask.shape
    normal_valid = np.isfinite(normals).all(axis=-1)
    support = ~normal_valid | (
        normals[..., 2] >= np.cos(np.radians(MAX_GROWTH_TILT_DEG))
    )
    if neighbors is None:
        neighbors = _growth_neighbors(points)
    right, down = neighbors
    for _ in range(MAX_GROWTH_PASSES):
        distance = np.abs(np.sum((points - surface.center) * surface.normal, axis=-1))
        compatible = support & np.isfinite(distance) & (distance <= MAX_PLANE_ERROR)
        compatible |= surface.mask
        lattice = np.zeros((2 * height - 1, 2 * width - 1), dtype=bool)
        lattice[::2, ::2] = compatible
        lattice[::2, 1::2] = compatible[:, :-1] & compatible[:, 1:] & right
        lattice[1::2, ::2] = compatible[:-1] & compatible[1:] & down
        seed = np.zeros_like(lattice)
        seed[::2, ::2] = surface.mask
        grown = ndimage.binary_propagation(seed, mask=lattice)[::2, ::2]
        if np.array_equal(grown, surface.mask):
            break
        fitted = _fit_surface(points, grown, max_tilt_deg)
        if fitted is None:
            break
        surface = fitted
    return surface


def _growth_neighbors(points):
    return (
        np.linalg.norm(points[:, 1:] - points[:, :-1], axis=-1) <= MAX_GROWTH_STEP,
        np.linalg.norm(points[1:] - points[:-1], axis=-1) <= MAX_GROWTH_STEP,
    )


def detect_shelf_geometry(
    depth: np.ndarray,
    camera_info: CameraInfo,
    transform,
    min_levels: int = MIN_SHELF_LEVELS,
    max_levels: int = MAX_SHELF_LEVELS,
    min_level_separation: float = DEFAULT_MIN_LEVEL_SEPARATION,
    *,
    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
) -> ShelfObservation:
    """Return measured supports, cabinet boundaries, occupancy and diagnostics.

    Normals and connected planes are identified before grouping heights. A
    surface's height is the median of its measured points. Vertical walls,
    observed undersides, narrow edges and small object tops are excluded.
    Large, flat object tops still require contextual or multi-view verification.
    """
    if depth.ndim != 2 or min(depth.shape) <= 2 * NORMAL_RADIUS:
        raise ShelfCalibrationError(f"Expected a depth image, got shape {depth.shape}")
    if not 0 <= min_levels <= max_levels or max_levels < 1:
        raise ShelfCalibrationError("Invalid minimum or maximum shelf level count")
    if not np.isfinite(min_level_separation) or min_level_separation <= 0:
        raise ShelfCalibrationError("Minimum level separation must be positive")
    if not np.isfinite(max_tilt_deg) or not 0 < max_tilt_deg < 90:
        raise ShelfCalibrationError("Surface tilt must be between 0 and 90 degrees")
    intrinsics = np.asarray(camera_info.k, dtype=float)
    if intrinsics.shape != (9,) or not np.isfinite(intrinsics).all():
        raise ShelfCalibrationError("CameraInfo has invalid intrinsics")
    fx, fy, cx, cy = intrinsics[[0, 4, 2, 5]]
    if fx <= 0 or fy <= 0:
        raise ShelfCalibrationError("CameraInfo has invalid focal lengths")

    height, width = depth.shape
    if (
        getattr(camera_info, "width", width) != width
        or getattr(camera_info, "height", height) != height
    ):
        raise ShelfCalibrationError("Depth dimensions do not match CameraInfo")
    if (
        getattr(getattr(transform, "header", None), "frame_id", "base_link")
        != "base_link"
    ):
        raise ShelfCalibrationError("Camera transform must target base_link")
    y0, y1 = int(height * 0.10), int(height * 0.95)
    x0, x1 = int(width * 0.10), int(width * 0.90)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    z = depth[y0:y1, x0:x1].astype(float)
    valid = np.isfinite(z) & (z > 0.15) & (z < 4.0)
    if int(valid.sum()) < 200:
        raise ShelfCalibrationError("Not enough valid depth points in the shelf view")
    z[~valid] = np.nan
    camera_points = np.stack(((xx - cx) * z / fx, (yy - cy) * z / fy, z), axis=-1)
    rotation = _rotation_matrix(transform.transform.rotation)
    translation = transform.transform.translation
    camera_position = np.array(
        [translation.x, translation.y, translation.z], dtype=float
    )
    if not np.isfinite(camera_position).all():
        raise ShelfCalibrationError("Camera transform has an invalid translation")
    points = camera_points @ rotation.T + camera_position

    normals = _surface_normals(points, camera_position)
    cosine = np.cos(np.radians(max_tilt_deg))
    supports = _seed_surfaces(points, normals[..., 2] >= cosine, max_tilt_deg)

    candidates = []
    neighbors = _growth_neighbors(points)
    for surface in _merge_surfaces(points, supports, max_tilt_deg):
        surface = _grow_surface(points, normals, surface, max_tilt_deg, neighbors)
        if surface.point_count >= MIN_SURFACE_POINTS and _has_surface_extent(
            points[surface.mask]
        ):
            candidates.append(surface)

    # Boards at or above the camera only show their underside. Normals face the
    # camera, so those planes point down. They are not grown: growth vetoes
    # downward normals on purpose, and a board underside is flat enough as seeded.
    undersides = _seed_surfaces(points, normals[..., 2] <= -cosine, max_tilt_deg)
    for surface in _merge_surfaces(points, undersides, max_tilt_deg):
        if surface.point_count >= MIN_SURFACE_POINTS and _has_surface_extent(
            points[surface.mask]
        ):
            surface.underside = True
            candidates.append(surface)

    # The floor band is dropped before any relative-size test, or a wide floor
    # would eliminate a lone shelf. Planes clearly below it stay selected so
    # the diagnostics flag the frame as invalid instead of hiding a TF bug.
    floor_like = [
        s for s in candidates if INVALID_FRAME_HEIGHT <= s.level < MIN_LEVEL_HEIGHT
    ]
    floor_ids = {id(s) for s in floor_like}
    candidates = [s for s in candidates if id(s) not in floor_ids]

    candidates = coherent_surfaces(
        points, candidates, camera_position, min_level_separation
    )
    if candidates:
        widths = [_surface_extent(points[s.mask])[1] for s in candidates]
        minimum = max(widths) * MIN_RELATIVE_SURFACE_WIDTH
        candidates = [s for s, width in zip(candidates, widths) if width >= minimum]

    structure = detect_structure(points, normals, candidates, camera_position)
    if candidates:
        reference = max(candidates, key=lambda s: s.point_count).center
        candidates = [
            s for s in candidates if inside_structure(points, s, structure, reference)
        ]

    # Measured supports win over undersides at the same height; larger first.
    selected = []
    for surface in sorted(
        candidates, key=lambda s: (not s.underside, s.point_count), reverse=True
    ):
        if all(
            abs(surface.level - previous.level) >= min_level_separation
            for previous in selected
        ):
            selected.append(surface)
        if len(selected) == max_levels:
            break
    if len(selected) < min_levels:
        raise ShelfCalibrationError(
            f"Detected {len(selected)} shelf levels; expected at least {min_levels}. "
            "Ensure the shelf support surfaces are visible and well illuminated."
        )
    stamp = getattr(getattr(transform, "header", None), "stamp", None)
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec if stamp is not None else None
    observation = describe_surfaces(
        points, normals, selected, camera_position, (x0, y0), stamp_ns, structure
    )
    diagnostics = observation.diagnostics
    diagnostics["excluded_floor_heights_m"] = sorted(s.level for s in floor_like)
    diagnostics["underside_levels_m"] = sorted(s.level for s in selected if s.underside)
    return observation


def _seed_surfaces(points: np.ndarray, mask: np.ndarray, max_tilt_deg: float):
    """Fit one plane per connected patch of pixels with a consistent normal."""
    labels, _ = ndimage.label(mask, structure=CONNECTIVITY)
    counts = np.bincount(labels.ravel())
    surfaces = []
    for component in np.flatnonzero(counts >= MIN_PATCH_POINTS):
        if component == 0:
            continue
        surface = _fit_surface(points, labels == component, max_tilt_deg)
        if surface is not None:
            surfaces.append(surface)
    return surfaces


def detect_shelf_levels(
    depth: np.ndarray,
    camera_info: CameraInfo,
    transform,
    min_levels: int = MIN_SHELF_LEVELS,
    max_levels: int = MAX_SHELF_LEVELS,
    min_level_separation: float = DEFAULT_MIN_LEVEL_SEPARATION,
    *,
    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
) -> list[float]:
    """Compatibility API returning ordered heights of the visible shelf stack."""
    observation = detect_shelf_geometry(
        depth,
        camera_info,
        transform,
        min_levels,
        max_levels,
        min_level_separation,
        max_tilt_deg=max_tilt_deg,
    )
    if observation.diagnostics["invalid_frame"]:
        raise ShelfCalibrationError(
            "Support heights are below base_link; verify camera TF and live joints"
        )
    return observation.levels

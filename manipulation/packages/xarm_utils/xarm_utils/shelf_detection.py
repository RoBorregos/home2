"""Shelf support-surface detection from an organized RGB-D point cloud."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

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
MIN_SURFACE_WIDTH = 0.35
MIN_SURFACE_DEPTH = 0.08
MAX_PLANE_ERROR = 0.015
CONNECTIVITY = np.ones((3, 3), dtype=bool)


@dataclass
class _Surface:
    mask: np.ndarray
    center: np.ndarray
    normal: np.ndarray
    point_count: int


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
    return _Surface(mask, center, normal, len(cloud))


def _merge_surfaces(points: np.ndarray, surfaces: list[_Surface], max_tilt_deg: float):
    """Combine coplanar patches, including fragments separated by missing depth."""
    merged = []
    for surface in sorted(surfaces, key=lambda item: item.point_count, reverse=True):
        for index, other in enumerate(merged):
            offset = surface.center - other.center
            if (
                surface.normal @ other.normal < np.cos(np.radians(max_tilt_deg))
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


def _has_surface_extent(cloud: np.ndarray) -> bool:
    """Measure the footprint along its principal axes to support rotated shelves."""
    xy = cloud[:, :2] - np.median(cloud[:, :2], axis=0)
    _, axes = np.linalg.eigh(xy.T @ xy / len(xy))
    lower, upper = np.percentile(xy @ axes, [5, 95], axis=0)
    depth, width = np.sort(upper - lower)
    return bool(depth >= MIN_SURFACE_DEPTH and width >= MIN_SURFACE_WIDTH)


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
    """Return one robust height per observed upper support surface in base_link.

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
    upward = normals[..., 2] >= np.cos(np.radians(max_tilt_deg))
    labels, _ = ndimage.label(upward, structure=CONNECTIVITY)
    counts = np.bincount(labels.ravel())
    surfaces = []
    for component in np.flatnonzero(counts >= MIN_SURFACE_POINTS):
        if component == 0:
            continue
        surface = _fit_surface(points, labels == component, max_tilt_deg)
        if surface is not None:
            surfaces.append(surface)

    candidates = []
    growth_tilt = min(2 * max_tilt_deg, 89.0)
    growth_normals = normals[..., 2] >= np.cos(np.radians(growth_tilt))
    for surface in _merge_surfaces(points, surfaces, max_tilt_deg):
        distance = np.abs(np.sum((points - surface.center) * surface.normal, axis=-1))
        compatible = growth_normals & (distance <= MAX_PLANE_ERROR)
        grown = ndimage.binary_propagation(
            surface.mask, structure=CONNECTIVITY, mask=compatible | surface.mask
        )
        surface = _fit_surface(points, grown, max_tilt_deg) or surface
        if _has_surface_extent(points[surface.mask]):
            candidates.append(surface)

    selected = []
    for surface in sorted(candidates, key=lambda item: item.point_count, reverse=True):
        level = float(surface.center[2])
        if all(abs(level - previous) >= min_level_separation for previous in selected):
            selected.append(level)
        if len(selected) == max_levels:
            break
    if len(selected) < min_levels:
        raise ShelfCalibrationError(
            f"Detected {len(selected)} shelf levels; expected at least {min_levels}. "
            "Ensure the shelf support surfaces are visible and well illuminated."
        )
    return sorted(selected)

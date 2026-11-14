"""Measured shelf geometry and conservative surface maps, independent of ROS.

Quality scores are heuristics, not calibrated probabilities. A measured support
cell is not a certificate of collision clearance. Unmeasured cells stay unknown.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import ConvexHull

from xarm_utils.shelf_levels import ShelfCalibrationError

PLANE_TOLERANCE = 0.015
STRUCTURE_ANGLE_DEG = 20.0
# Typical shelf board thickness. A board seen from below reports its support
# surface this far above the measured underside.
BOARD_THICKNESS = 0.02
GRID_RESOLUTION = 0.02
GRID_UNKNOWN, GRID_SUPPORT, GRID_OBSTACLE, GRID_OUTSIDE = range(4)


@dataclass
class BoundaryPlane:
    """A vertical boundary fitted to measured points in base_link."""

    normal: np.ndarray
    center: np.ndarray
    point_count: int
    error_m: float

    @property
    def offset(self) -> float:
        return float(-self.normal @ self.center)

    def to_dict(self) -> dict:
        return {
            "normal": self.normal.tolist(),
            "offset_m": self.offset,
            "point_count": self.point_count,
            "residual_p90_m": self.error_m,
            "source": "measured_vertical_points",
        }


@dataclass
class ShelfStructure:
    """Optional boundaries of the selected stack; missing faces remain unknown."""

    lateral_axis: np.ndarray
    depth_axis: np.ndarray
    left: BoundaryPlane | None = None
    right: BoundaryPlane | None = None
    back: BoundaryPlane | None = None

    @property
    def boundary_count(self) -> int:
        return sum(p is not None for p in (self.left, self.right, self.back))

    def to_dict(self) -> dict:
        return {
            "lateral_axis": self.lateral_axis.tolist(),
            "depth_axis": self.depth_axis.tolist(),
            **{
                name: panel.to_dict() if panel is not None else None
                for name, panel in (
                    ("left", self.left),
                    ("right", self.right),
                    ("back", self.back),
                )
            },
        }


@dataclass
class ShelfSurfaceGeometry:
    """A fitted support, its observed hull and an explicitly estimated outline."""

    height_m: float
    center: np.ndarray
    normal: np.ndarray
    observed_hull: np.ndarray
    estimated_outline: np.ndarray
    outline_source: str
    width_m: float
    depth_m: float
    point_count: int
    residual_p90_m: float
    quality_score: float
    width_coverage: float
    depth_coverage: float
    boundary_count: int
    front: BoundaryPlane | None
    grid_origin: np.ndarray
    grid_axes: np.ndarray
    grid_resolution_m: float
    grid_states: np.ndarray
    obstacle_max_height_m: float
    pixel_mask: np.ndarray = field(repr=False)
    crop_origin: tuple[int, int] = (0, 0)
    # "support": the top of the board was measured. "underside": only its
    # underside was seen and height_m assumes BOARD_THICKNESS above it.
    source: str = "support"

    @property
    def tilt_deg(self) -> float:
        return float(np.degrees(np.arccos(np.clip(self.normal[2], -1, 1))))

    def evidence(self, calibration_warning: bool = False) -> dict:
        """Return the measurements used for adaptive temporal confirmation."""
        return {
            "source": self.source,
            "quality_score": self.quality_score,
            "width_coverage": self.width_coverage,
            "depth_coverage": self.depth_coverage,
            "residual_p90_m": self.residual_p90_m,
            "boundary_count": self.boundary_count,
            "normal": self.normal.tolist(),
            "calibration_warning": calibration_warning,
        }

    def to_dict(self) -> dict:
        return {
            "height_m": self.height_m,
            "source": self.source,
            "plane": {
                "normal": self.normal.tolist(),
                "offset_m": float(-self.normal @ self.center),
            },
            "center_m": self.center.tolist(),
            "tilt_deg": self.tilt_deg,
            "width_m": self.width_m,
            "depth_m": self.depth_m,
            "point_count": self.point_count,
            "residual_p90_m": self.residual_p90_m,
            "quality_score": self.quality_score,
            "quality_is_probability": False,
            "width_coverage": self.width_coverage,
            "depth_coverage": self.depth_coverage,
            "observed_support_hull_m": self.observed_hull.tolist(),
            "estimated_outline_m": self.estimated_outline.tolist(),
            "outline_source": self.outline_source,
            "front_face": self.front.to_dict() if self.front else None,
            "placement_clearance_verified": False,
            "surface_grid": {
                "origin_m": self.grid_origin.tolist(),
                "axes": self.grid_axes.tolist(),
                "resolution_m": self.grid_resolution_m,
                "states": self.grid_states.tolist(),
                "legend": {
                    "0": "unknown",
                    "1": "observed_support",
                    "2": "obstacle",
                    "3": "outside_outline",
                },
                "obstacle_max_height_m": self.obstacle_max_height_m,
            },
        }


@dataclass
class ShelfObservation:
    """One RGB-D observation; level indices describe the visible stack only."""

    surfaces: list[ShelfSurfaceGeometry]
    structure: ShelfStructure | None
    diagnostics: dict
    stamp_ns: int | None = None

    @property
    def levels(self) -> list[float]:
        return [s.height_m for s in self.surfaces]

    @property
    def calibration_warning(self) -> bool:
        return bool(self.diagnostics.get("calibration_warning", False))

    def to_dict(self) -> dict:
        return {
            "schema_version": 1,
            "frame": "base_link",
            "stamp_ns": self.stamp_ns,
            "level_indices": "ordered_visible_surfaces_not_global_competition_labels",
            "surfaces": [s.to_dict() for s in self.surfaces],
            "structure": self.structure.to_dict() if self.structure else None,
            "diagnostics": self.diagnostics,
        }


def footprint_axes(points: np.ndarray, camera_position: np.ndarray):
    """Orient the footprint's long axis laterally and its short axis away."""
    xy = points[:, :2] - np.median(points[:, :2], axis=0)
    _, vectors = np.linalg.eigh(xy.T @ xy / len(xy))
    lateral = np.r_[vectors[:, -1], 0.0]
    forward = np.array([-lateral[1], lateral[0], 0.0])
    if forward @ (np.median(points, axis=0) - camera_position) < 0:
        forward *= -1
    return lateral, forward


def _fit_boundary(cloud, axis):
    center = np.median(cloud, axis=0)
    centered = cloud - center
    values, vectors = np.linalg.eigh(centered.T @ centered / len(cloud))
    normal = vectors[:, 0]
    if normal @ axis < 0:
        normal *= -1
    error = float(np.percentile(np.abs(centered @ normal), 90))
    if (
        values[1] < 1e-7
        or normal @ axis < np.cos(np.radians(STRUCTURE_ANGLE_DEG))
        or error > PLANE_TOLERANCE
    ):
        return None
    return BoundaryPlane(normal, center, len(cloud), error)


def find_boundary(
    points,
    normals,
    axis,
    location,
    region,
    min_span,
    span_axis,
    min_height,
    allow_missing_normals=False,
    search_distance=0.22,
):
    """Find a vertical plane near a geometrically predicted face, without ROI input."""
    vertical = np.abs(normals[..., 2]) <= np.sin(np.radians(STRUCTURE_ANGLE_DEG))
    aligned = np.abs(normals @ axis) >= np.cos(np.radians(STRUCTURE_ANGLE_DEG))
    if allow_missing_normals:
        missing = ~np.isfinite(normals).all(axis=-1)
        vertical |= missing
        aligned |= missing
    positions = points @ axis
    valid = (
        region & vertical & aligned & (np.abs(positions - location) < search_distance)
    )
    cloud = points[valid]
    if len(cloud) < 40:
        return None
    offsets = cloud @ axis
    bins = np.floor(offsets / 0.01).astype(int)
    labels, counts = np.unique(bins, return_counts=True)
    peak = (labels[np.argmax(counts)] + 0.5) * 0.01
    seed = cloud[np.abs(offsets - peak) <= 0.025]
    if len(seed) < 30:
        return None
    panel = _fit_boundary(seed, axis)
    if panel is None:
        return None
    for _ in range(2):
        inliers = cloud[
            np.abs((cloud - panel.center) @ panel.normal) <= PLANE_TOLERANCE
        ]
        if len(inliers) < 30:
            return None
        fitted = _fit_boundary(inliers, axis)
        if fitted is None:
            return None
        panel = fitted
    span = np.ptp(np.percentile(inliers @ span_axis, [5, 95]))
    height = np.ptp(np.percentile(inliers[:, 2], [5, 95]))
    return panel if span >= min_span and height >= min_height else None


def detect_structure(
    points, normals, surfaces, camera_position
) -> ShelfStructure | None:
    """Identify side and back panels around a coherent stack of support planes."""
    if not surfaces:
        return None
    reference = max(surfaces, key=lambda s: s.point_count)
    lateral, forward = footprint_axes(points[reference.mask], camera_position)
    cloud = np.vstack([points[s.mask] for s in surfaces])
    lateral_bounds = np.percentile(cloud @ lateral, [2, 98])
    depth_bounds = np.percentile(cloud @ forward, [2, 98])
    low, high = min(s.center[2] for s in surfaces), max(s.center[2] for s in surfaces)
    s, d = points @ lateral, points @ forward
    region = (
        np.isfinite(points).all(axis=-1)
        & (s > lateral_bounds[0] - 0.22)
        & (s < lateral_bounds[1] + 0.22)
        & (d > depth_bounds[0] - 0.12)
        & (d < depth_bounds[1] + 0.25)
        & (points[..., 2] > low - 0.12)
        & (points[..., 2] < high + 0.7)
    )
    height_span = max(0.2, (high - low) * 0.7)
    structure = ShelfStructure(lateral, forward)
    for name, location in [("left", lateral_bounds[0]), ("right", lateral_bounds[1])]:
        panel = find_boundary(
            points, normals, lateral, location, region, 0.10, forward, height_span
        )
        if panel is not None:
            z0 = panel.center[2]
            # A stored object's side must not replace a panel spanning the stack.
            if low - 0.15 <= z0 <= high + 0.6:
                setattr(structure, name, panel)
    structure.back = find_boundary(
        points,
        normals,
        forward,
        depth_bounds[1],
        region,
        float(np.diff(lateral_bounds)[0]) * 0.55,
        lateral,
        height_span,
    )
    if structure.left is not None and structure.right is not None:
        if (structure.right.center - structure.left.center) @ lateral < 0.25:
            structure.left = structure.right = None
    return structure


def coherent_surfaces(points, surfaces, camera_position, min_separation):
    """Prefer a stacked set of matching footprints over an unrelated wide plane."""
    if len(surfaces) < 2:
        return surfaces
    groups = []
    for ri, reference in enumerate(surfaces):
        lateral, forward = footprint_axes(points[reference.mask], camera_position)
        bounds = []
        for s in surfaces:
            cloud = points[s.mask]
            bounds.append(
                np.percentile(cloud @ np.stack((lateral, forward)).T, [5, 95], axis=0)
            )
        ref = bounds[ri]
        group = []
        for s, bound in zip(surfaces, bounds):
            widths = [ref[1, 0] - ref[0, 0], bound[1, 0] - bound[0, 0]]
            overlap = max(
                0.0, min(ref[1, 0], bound[1, 0]) - max(ref[0, 0], bound[0, 0])
            )
            depth_overlap = min(ref[1, 1], bound[1, 1]) - max(ref[0, 1], bound[0, 1])
            if (
                min(widths) / max(widths) >= 0.65
                and overlap / min(widths) >= 0.6
                and depth_overlap > 0
            ):
                group.append(s)
        unique = []
        for s in sorted(group, key=lambda item: item.point_count, reverse=True):
            if all(abs(s.center[2] - p.center[2]) >= min_separation for p in unique):
                unique.append(s)
        groups.append(unique)
    best = max(groups, key=lambda g: (len(g), sum(s.point_count for s in g)))
    return best if len(best) >= 2 else surfaces


def inside_structure(points, surface, structure, reference_center) -> bool:
    """Reject measured support extending outside observed cabinet boundaries."""
    if structure is None:
        return True
    cloud = points[surface.mask]
    for panel in (structure.left, structure.right, structure.back):
        if panel is None:
            continue
        sign = np.sign((reference_center - panel.center) @ panel.normal)
        if sign == 0:
            continue
        distances = (cloud - panel.center) @ panel.normal * sign
        if np.mean(distances < -0.03) > 0.2:
            return False
    return True


def _surface_basis(normal, lateral):
    u = lateral - normal * (lateral @ normal)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    return np.stack((u, v))


def _outline_from_panels(surface, basis, hull, structure, front):
    if structure is None or any(
        p is None for p in (structure.left, structure.right, structure.back)
    ):
        return hull, "observed_support_hull"
    center = surface.center
    lines = []
    for panel in (structure.left, structure.right, structure.back):
        lines.append((basis @ panel.normal, -(panel.normal @ center + panel.offset)))
    if front is None:
        front_line = (np.array([0.0, 1.0]), float(hull[:, 1].min()))
    else:
        front_line = (basis @ front.normal, -(front.normal @ center + front.offset))
    vertices = []
    try:
        for a, b in [
            (lines[0], front_line),
            (lines[1], front_line),
            (lines[1], lines[2]),
            (lines[0], lines[2]),
        ]:
            vertices.append(np.linalg.solve(np.stack((a[0], b[0])), [a[1], b[1]]))
    except np.linalg.LinAlgError:
        return hull, "observed_support_hull"
    polygon = np.array(vertices)
    old_extent = np.ptp(hull, axis=0)
    new_extent = np.ptp(polygon, axis=0)
    if (
        not np.isfinite(polygon).all()
        or np.any(new_extent < old_extent * 0.7)
        or np.any(new_extent > old_extent * 2.5)
    ):
        return hull, "observed_support_hull"
    return polygon, "measured_panels_and_" + (
        "measured_front" if front else "observed_front_limit"
    )


def surface_grid(cloud, support, center, normal, basis, outline, obstacle_height):
    """Classify support, obstacles and unknown cells; deliberately no 'free' state."""
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    resolution = max(GRID_RESOLUTION, float(np.max(hi - lo)) / 255)
    columns, rows = np.maximum(1, np.ceil((hi - lo) / resolution).astype(int))
    yy, xx = np.mgrid[:rows, :columns]
    centers = np.stack(
        (lo[0] + (xx + 0.5) * resolution, lo[1] + (yy + 0.5) * resolution), axis=-1
    )
    equations = ConvexHull(outline).equations
    inside = np.all(centers @ equations[:, :2].T + equations[:, 2] <= 1e-8, axis=-1)
    states = np.where(inside, GRID_UNKNOWN, GRID_OUTSIDE).astype(np.uint8)

    def mark(points, state):
        uv = (points - center) @ basis.T
        indices = np.floor((uv - lo) / resolution).astype(int)
        valid = (
            (indices[:, 0] >= 0)
            & (indices[:, 0] < columns)
            & (indices[:, 1] >= 0)
            & (indices[:, 1] < rows)
        )
        x, y = indices[valid].T
        valid = inside[y, x]
        states[y[valid], x[valid]] = state

    mark(support, GRID_SUPPORT)
    offsets = (cloud - center) @ normal
    objects = cloud[(offsets > 0.025) & (offsets <= obstacle_height)]
    mark(objects, GRID_OBSTACLE)
    width_cov = float(
        np.count_nonzero(np.any(states == GRID_SUPPORT, axis=0))
        / max(1, np.count_nonzero(np.any(inside, axis=0)))
    )
    depth_cov = float(
        np.count_nonzero(np.any(states == GRID_SUPPORT, axis=1))
        / max(1, np.count_nonzero(np.any(inside, axis=1)))
    )
    return states, center + lo @ basis, resolution, width_cov, depth_cov


def describe_surfaces(
    points,
    normals,
    surfaces,
    camera_position,
    crop_origin,
    stamp_ns=None,
    structure=None,
) -> ShelfObservation:
    """Build bounded plane models and diagnostics from measured organized points."""
    if structure is None:
        structure = detect_structure(points, normals, surfaces, camera_position)
    finite = points[np.isfinite(points).all(axis=-1)]
    results = []
    for surface in sorted(surfaces, key=lambda s: s.center[2]):
        underside = bool(getattr(surface, "underside", False))
        measured = points[surface.mask]
        lateral = (
            structure.lateral_axis
            if structure
            else footprint_axes(measured, camera_position)[0]
        )
        basis = _surface_basis(surface.normal, lateral)
        if basis[1] @ (surface.center - camera_position) < 0:
            basis[1] *= -1
        uv = (measured - surface.center) @ basis.T
        hull = uv[ConvexHull(uv).vertices]
        projected = measured @ (structure.depth_axis if structure else basis[1])
        front_location = float(np.percentile(projected, 5))
        local = (
            np.isfinite(points).all(axis=-1)
            & (points[..., 2] > surface.center[2] - 0.07)
            & (points[..., 2] < surface.center[2] + 0.01)
        )
        front_axis = structure.depth_axis if structure else basis[1]
        lateral_positions = points @ lateral
        a, b = np.percentile(measured @ lateral, [2, 98])
        local &= (lateral_positions > a - 0.03) & (lateral_positions < b + 0.03)
        front = find_boundary(
            points,
            normals,
            front_axis,
            front_location,
            local,
            (b - a) * 0.6,
            lateral,
            0.01,
            allow_missing_normals=True,
            search_distance=0.07,
        )
        if front is not None and structure.back is not None:
            if abs((front.center - structure.back.center) @ front_axis) < 0.05:
                front = None
        outline, source = _outline_from_panels(surface, basis, hull, structure, front)
        if underside:
            # The front lip of a board seen from below is its own edge; a
            # measured underside says nothing about objects on top.
            front = None
            outline, source = hull, "observed_underside_hull"
        higher = [
            s.center[2] - surface.center[2] - 0.025
            for s in surfaces
            if s.center[2] > surface.center[2] + 0.05
        ]
        obstacle_height = min([0.3] + higher)
        states, origin, resolution, wc, dc = surface_grid(
            finite,
            measured,
            surface.center,
            surface.normal,
            basis,
            outline,
            obstacle_height,
        )
        error = float(
            np.percentile(np.abs((measured - surface.center) @ surface.normal), 90)
        )
        count = structure.boundary_count if structure else 0
        score = (
            0.4 * max(0.0, 1 - error / PLANE_TOLERANCE)
            + 0.3 * min(wc, dc)
            + 0.15 * min(len(measured) / 600, 1)
            + 0.15 * count / 3
        )
        extents = np.ptp(outline, axis=0)
        results.append(
            ShelfSurfaceGeometry(
                float(surface.center[2]) + (BOARD_THICKNESS if underside else 0.0),
                surface.center,
                surface.normal,
                surface.center + hull @ basis,
                surface.center + outline @ basis,
                source,
                float(extents[0]),
                float(extents[1]),
                len(measured),
                error,
                float(np.clip(score, 0, 1)),
                wc,
                dc,
                count,
                front,
                origin,
                basis,
                resolution,
                states,
                obstacle_height,
                surface.mask,
                crop_origin,
                "underside" if underside else "support",
            )
        )
    diagnostics = calibration_diagnostics(results, structure)
    if structure is None or structure.boundary_count < 3:
        diagnostics["warnings"].append(
            "incomplete_structure: unobserved walls or edges remain unknown"
        )
    return ShelfObservation(results, structure, diagnostics, stamp_ns)


def calibration_diagnostics(surfaces, structure=None) -> dict:
    """Flag a consistent lean; never claim its physical cause or correct TF."""
    invalid = any(getattr(s, "height_m", 0) < -0.02 for s in surfaces)
    result = {
        "calibration_warning": invalid,
        "invalid_frame": invalid,
        "common_tilt_deg": None,
        "normal_spread_deg": None,
        "max_panel_orthogonality_error_deg": None,
        "warnings": [],
    }
    if invalid:
        result["warnings"].append(
            "negative_support_height: base_link is assumed gravity-aligned at the robot floor; verify live joints and camera TF"
        )
    if len(surfaces) < 2:
        return result
    normals = np.array([s.normal for s in surfaces])
    average = np.mean(normals, axis=0)
    average /= np.linalg.norm(average)
    tilt = float(np.degrees(np.arccos(np.clip(average[2], -1, 1))))
    spread = float(np.max(np.degrees(np.arccos(np.clip(normals @ average, -1, 1)))))
    result.update(common_tilt_deg=tilt, normal_spread_deg=spread)
    if tilt > 5 and spread < 3:
        result["calibration_warning"] = True
        result["warnings"].append(
            "common_support_tilt: verify camera-to-arm TF and a physically level reference; no automatic correction applied"
        )
    if spread > 5:
        result["calibration_warning"] = True
        result["warnings"].append(
            "support_normals_disagree: inspect depth quality, support tilt and camera TF before trusting an extrapolated outline"
        )
    if structure is not None:
        panels = [
            p
            for p in (structure.left, structure.right, structure.back)
            if p is not None
        ]
        if panels:
            error = max(
                float(np.degrees(np.arcsin(np.clip(abs(n @ p.normal), 0, 1))))
                for n in normals
                for p in panels
            )
            result["max_panel_orthogonality_error_deg"] = error
            if error > 5:
                result["calibration_warning"] = True
                result["warnings"].append(
                    "support_panel_nonorthogonality: a rigid TF rotation cannot fix relative plane angles; inspect depth fits, panel association or physical shelf tilt"
                )
    return result


def compare_reference_heights(estimated, measured, tolerance=0.02) -> dict:
    """Compare ordered heights with an external measurement, without calibration."""
    a, b = np.asarray(estimated, float), np.asarray(measured, float)
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ShelfCalibrationError("Reference tolerance must be positive and finite")
    if (
        a.ndim != 1
        or a.shape != b.shape
        or not len(a)
        or not np.isfinite(a).all()
        or not np.isfinite(b).all()
        or np.any(np.diff(b) <= 0)
    ):
        raise ShelfCalibrationError(
            "Reference heights must be finite and match the detected level count"
        )
    errors = a - b
    return {
        "errors_m": errors.tolist(),
        "max_absolute_error_m": float(np.max(np.abs(errors))),
        "within_tolerance": bool(np.all(np.abs(errors) <= tolerance)),
        "tolerance_m": tolerance,
        "automatic_correction_applied": False,
    }


def confirmed_geometry_report(levels, observations, max_spread, mode) -> dict:
    """Match stable heights to the best measured geometry in the same window.

    Geometry remains from its reported source frame. Only the height is a
    temporal median; footprints and unknown cells are not silently fused.
    """
    frames = [o for o in observations if o is not None]
    if not frames:
        raise ShelfCalibrationError("No measured geometry for confirmed levels")
    surfaces, selected = [], []
    for index, level in enumerate(levels, 1):
        matches = [
            (o, s)
            for o in frames
            for s in o.surfaces
            if abs(s.height_m - level) <= max_spread
        ]
        if not matches:
            raise ShelfCalibrationError(
                "Confirmed height has no matching measured surface"
            )
        source, surface = max(
            matches, key=lambda pair: (pair[1].quality_score, pair[0].stamp_ns or 0)
        )
        item = surface.to_dict()
        item.update(
            visible_level_index=index,
            confirmed_height_m=level,
            source_stamp_ns=source.stamp_ns,
            supporting_frames=len(
                {o.stamp_ns if o.stamp_ns is not None else id(o) for o, _ in matches}
            ),
        )
        surfaces.append(item)
        selected.append(surface)
    source = max(
        frames,
        key=lambda o: (
            o.structure.boundary_count if o.structure else 0,
            o.stamp_ns or 0,
        ),
    )
    diagnostics = calibration_diagnostics(selected, source.structure)
    for o in frames:
        for warning in o.diagnostics.get("warnings", []):
            if warning not in diagnostics["warnings"]:
                diagnostics["warnings"].append(warning)
    return {
        "schema_version": 1,
        "frame": "base_link",
        "confirmation_mode": mode,
        "processed_frames": len(observations),
        "level_indices": "ordered_visible_surfaces_not_global_competition_labels",
        "geometry_fusion": "best_observed_frame_per_level; heights_only_are_temporal_medians",
        "surfaces": surfaces,
        "structure": source.structure.to_dict() if source.structure else None,
        "diagnostics": diagnostics,
    }

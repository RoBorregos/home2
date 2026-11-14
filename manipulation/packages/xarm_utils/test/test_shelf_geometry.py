"""Cabinet boundaries, measured occupancy and calibration diagnostics."""

from dataclasses import replace
import json
from types import SimpleNamespace as N

import numpy as np
import pytest

from xarm_utils.shelf_detection import detect_shelf_geometry, detect_shelf_levels
from xarm_utils.shelf_geometry import (
    GRID_UNKNOWN,
    GRID_SUPPORT,
    GRID_OBSTACLE,
    calibration_diagnostics,
    compare_reference_heights,
    confirmed_geometry_report,
)
from xarm_utils.shelf_levels import ShelfCalibrationError
from xarm_utils.shelf_observations import ShelfLevelConsensus


@pytest.fixture
def cabinet():
    rows, columns = np.mgrid[:360, :480]
    nx, ny = (columns - 240) / 300, (rows - 30) / 250
    depth = np.full(nx.shape, np.inf)

    def insert(distance, valid):
        nonlocal depth
        depth = np.where(
            valid & (distance > 0.15) & (distance < depth), distance, depth
        )

    def x_face(x, zlo, zhi):
        d = np.full(nx.shape, x)
        z = 1.6 - ny * d
        insert(d, (abs(nx * d) <= 0.5) & (z >= zlo) & (z <= zhi))

    x_face(1.7, 0.45, 1.5)
    for side in [-0.5, 0.5]:
        d = np.divide(-side, nx, out=np.full_like(nx, np.inf), where=nx != 0)
        with np.errstate(invalid="ignore"):
            z = 1.6 - ny * d
        insert(d, (d >= 1.1) & (d <= 1.7) & (z >= 0.45) & (z <= 1.5))
    for level in [0.7, 1.1]:
        d = np.divide(1.6 - level, ny, out=np.full_like(ny, np.inf), where=ny > 0)
        with np.errstate(invalid="ignore"):
            inside = (d >= 1.1) & (d <= 1.7) & (abs(nx * d) <= 0.5)
        insert(d, inside)
        x_face(1.1, level - 0.04, level)
    depth[~np.isfinite(depth)] = np.nan
    info = N(k=[300, 0, 240, 0, 250, 30, 0, 0, 1], width=480, height=360)
    tf = N(
        transform=N(
            rotation=N(x=-0.5, y=0.5, z=-0.5, w=0.5), translation=N(x=0.0, y=0.0, z=1.6)
        )
    )
    return N(depth=depth, info=info, tf=tf, nx=nx, ny=ny)


def test_measured_side_and_back_panels_bound_the_shelf(cabinet):
    view = detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)
    assert view.levels == pytest.approx([0.7, 1.1], abs=0.005)
    assert view.structure.boundary_count == 3
    assert view.structure.back.center[0] == pytest.approx(1.7, abs=0.02)
    assert sorted(
        [view.structure.left.center[1], view.structure.right.center[1]]
    ) == pytest.approx([-0.5, 0.5], abs=0.02)
    assert all(s.front is not None for s in view.surfaces)
    assert all(s.width_m == pytest.approx(1.0, abs=0.04) for s in view.surfaces)


def test_geometry_is_serializable_and_does_not_certify_placement(cabinet):
    view = detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)
    payload = json.loads(json.dumps(view.to_dict(), allow_nan=False))
    assert payload["frame"] == "base_link"
    for s in payload["surfaces"]:
        assert s["placement_clearance_verified"] is False
        assert s["quality_is_probability"] is False
        assert "free" not in s["surface_grid"]["legend"].values()
        assert len(s["observed_support_hull_m"]) >= 3


def test_depth_hole_stays_unknown_inside_estimated_plane(cabinet):
    d = cabinet.depth.copy()
    z = 1.6 - cabinet.ny * d
    hole = (abs(z - 1.1) < 0.002) & (d > 1.3) & (d < 1.5) & (abs(cabinet.nx * d) < 0.12)
    assert hole.any()
    d[hole] = np.nan
    view = detect_shelf_geometry(d, cabinet.info, cabinet.tf)
    top = view.surfaces[-1]
    assert np.count_nonzero(top.grid_states == GRID_UNKNOWN) > 10
    assert np.count_nonzero(top.grid_states == GRID_SUPPORT) > 10


def test_object_above_support_is_marked_as_obstacle(cabinet):
    d = cabinet.depth.copy()
    box = np.divide(
        1.6 - 0.9, cabinet.ny, out=np.full_like(d, np.inf), where=cabinet.ny > 0
    )
    with np.errstate(invalid="ignore"):
        inside = (box > 1.25) & (box < 1.45) & (abs(cabinet.nx * box) < 0.1)
    nearer = inside & (~np.isfinite(d) | (box < d))
    d[nearer] = box[nearer]
    view = detect_shelf_geometry(d, cabinet.info, cabinet.tf)
    assert view.levels == pytest.approx([0.7, 1.1], abs=0.01)
    assert np.count_nonzero(view.surfaces[0].grid_states == GRID_OBSTACLE) > 5


def test_missing_panels_remain_optional(cabinet):
    d = cabinet.depth.copy()
    z = 1.6 - cabinet.ny * d
    d[(abs(z - 0.7) > 0.002) & (abs(z - 1.1) > 0.002)] = np.nan
    view = detect_shelf_geometry(d, cabinet.info, cabinet.tf)
    assert view.structure.boundary_count == 0
    assert all(s.outline_source == "observed_support_hull" for s in view.surfaces)
    assert any("incomplete_structure" in w for w in view.diagnostics["warnings"])


def test_back_panel_is_not_reported_as_front_when_front_depth_is_missing(cabinet):
    d = cabinet.depth.copy()
    z = 1.6 - cabinet.ny * 1.28
    back = (abs(cabinet.nx * 1.28) <= 0.5) & (z >= 0.45) & (z <= 1.5)
    d[back & (~np.isfinite(d) | (d > 1.28))] = 1.28
    d[abs(d - 1.1) < 1e-6] = np.nan
    view = detect_shelf_geometry(d, cabinet.info, cabinet.tf)
    assert all(s.front is None for s in view.surfaces)


def test_height_compatibility_api_matches_geometry(cabinet):
    assert detect_shelf_levels(
        cabinet.depth, cabinet.info, cabinet.tf
    ) == pytest.approx(
        detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf).levels
    )


def test_common_tilt_is_diagnostic_not_an_automatic_tf_correction():
    angle = np.radians(8)
    normal = np.array([np.sin(angle), 0, np.cos(angle)])
    result = calibration_diagnostics([N(normal=normal.copy()), N(normal=normal.copy())])
    assert result["calibration_warning"]
    assert result["common_tilt_deg"] == pytest.approx(8)
    assert np.array_equal(normal, np.array([np.sin(angle), 0, np.cos(angle)]))


def test_rigid_rotation_preserves_right_angle_between_support_and_panel():
    angle = np.radians(8)
    normal = np.array([np.sin(angle), 0, np.cos(angle)])
    panel = N(normal=np.array([np.cos(angle), 0, -np.sin(angle)]))
    result = calibration_diagnostics(
        [N(normal=normal), N(normal=normal)], N(left=None, right=None, back=panel)
    )
    assert result["max_panel_orthogonality_error_deg"] == pytest.approx(0, abs=1e-6)


def test_nonorthogonal_fit_is_distinguished_from_global_tf_rotation():
    angle = np.radians(8)
    normal = np.array([np.sin(angle), 0, np.cos(angle)])
    panel = N(normal=np.array([1.0, 0.0, 0.0]))
    result = calibration_diagnostics(
        [N(normal=normal), N(normal=normal)], N(left=None, right=None, back=panel)
    )
    assert result["max_panel_orthogonality_error_deg"] == pytest.approx(8)
    assert any("nonorthogonality" in warning for warning in result["warnings"])


def test_reference_measurement_detects_height_offset():
    check = compare_reference_heights([0.75, 1.15], [0.7, 1.1])
    assert not check["within_tolerance"]
    assert check["max_absolute_error_m"] == pytest.approx(0.05)
    assert check["automatic_correction_applied"] is False


def test_reference_count_must_match():
    with pytest.raises(ShelfCalibrationError, match="match"):
        compare_reference_heights([0.7, 1.1], [0.7])


def test_stable_report_keeps_source_frame_and_median_separate(cabinet):
    view = detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)
    history = [replace(view, stamp_ns=i) for i in range(1, 7)]
    levels = [0.701, 1.101]
    report = confirmed_geometry_report(levels, history, 0.015, "quality")
    assert report["processed_frames"] == 6
    for level, surface in zip(levels, report["surfaces"]):
        assert surface["confirmed_height_m"] == level
        assert surface["supporting_frames"] == 6
        assert surface["source_stamp_ns"] == 6


def test_real_geometry_quality_enables_early_confirmation(cabinet):
    view = detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)
    consensus = ShelfLevelConsensus()
    evidence = [s.evidence(view.calibration_warning) for s in view.surfaces]
    for i in range(6):
        result = consensus.add(view.levels, evidence)
        if i < 5:
            assert result is None
    assert result == pytest.approx(view.levels)
    assert consensus.confirmation_mode == "quality"


def test_wide_floor_does_not_replace_the_coherent_shelf_stack(cabinet):
    d = cabinet.depth.copy()
    floor = np.divide(
        1.6, cabinet.ny, out=np.full_like(d, np.inf), where=cabinet.ny > 0
    )
    nearer = (floor > 0.15) & (floor < 4) & (~np.isfinite(d) | (floor < d))
    d[nearer] = floor[nearer]
    assert detect_shelf_levels(d, cabinet.info, cabinet.tf) == pytest.approx(
        [0.7, 1.1], abs=0.01
    )


def test_negative_base_link_support_heights_are_not_accepted(cabinet):
    cabinet.tf.transform.translation.z = 0.0
    view = detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)
    assert view.diagnostics["invalid_frame"]
    with pytest.raises(ShelfCalibrationError, match="below base_link"):
        detect_shelf_levels(cabinet.depth, cabinet.info, cabinet.tf)


def test_camera_info_resolution_mismatch_is_rejected(cabinet):
    cabinet.info.width += 1
    with pytest.raises(ShelfCalibrationError, match="CameraInfo"):
        detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)


def test_non_base_link_transform_is_rejected(cabinet):
    cabinet.tf.header = N(frame_id="map")
    with pytest.raises(ShelfCalibrationError, match="target base_link"):
        detect_shelf_geometry(cabinet.depth, cabinet.info, cabinet.tf)

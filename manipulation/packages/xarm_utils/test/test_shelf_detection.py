"""Offline regressions for support surfaces, discontinuities and invalid depth."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from xarm_utils.shelf_detection import _grow_surface, _Surface, detect_shelf_levels
from xarm_utils.shelf_levels import ShelfCalibrationError


@pytest.fixture
def scene():
    rows, columns = np.mgrid[:480, :640]
    fx, fy, cx, cy = 400.0, 150.0, 320.0, 80.0
    nx, ny = (columns - cx) / fx, (rows - cy) / fy
    info = NS(k=[fx, 0, cx, 0, fy, cy, 0, 0, 1])
    transform = NS(
        transform=NS(
            rotation=NS(x=-0.5, y=0.5, z=-0.5, w=0.5),
            translation=NS(x=0.0, y=0.0, z=1.2),
        )
    )
    levels = [0.38, 0.54, 0.72, 0.90]
    depth = np.full(nx.shape, np.nan)
    for height in levels:
        distance = np.divide(
            1.2 - height, ny, out=np.full_like(ny, np.nan), where=ny > 0
        )
        inside = (distance >= 1.1) & (distance <= 2.2) & (abs(nx * distance) < 0.5)
        nearer = inside & (~np.isfinite(depth) | (distance < depth))
        depth[nearer] = distance[nearer]
    return NS(depth=depth, info=info, transform=transform, levels=levels, nx=nx, ny=ny)


def test_four_surfaces_are_detected_without_assuming_their_count(scene):
    assert detect_shelf_levels(
        scene.depth, scene.info, scene.transform
    ) == pytest.approx(scene.levels, abs=0.01)


def test_vertical_wall_does_not_create_horizontal_levels(scene):
    wall = 1.2 / (1 - scene.nx)
    with pytest.raises(ShelfCalibrationError, match="Detected 0 shelf levels"):
        detect_shelf_levels(wall, scene.info, scene.transform)


def test_depth_step_between_two_walls_does_not_create_a_support(scene):
    wall = np.full_like(scene.depth, 1.5)
    wall[240:] = 2.2
    with pytest.raises(ShelfCalibrationError, match="Detected 0 shelf levels"):
        detect_shelf_levels(wall, scene.info, scene.transform)


def test_noise_and_missing_depth_preserve_the_surfaces(scene):
    rng = np.random.default_rng(20261111)
    noisy = scene.depth + rng.normal(0, 0.002, scene.depth.shape)
    noisy[rng.random(noisy.shape) < 0.02] = np.nan
    assert detect_shelf_levels(noisy, scene.info, scene.transform) == pytest.approx(
        scene.levels, abs=0.01
    )


def test_fragments_on_both_sides_are_combined_into_one_surface(scene):
    # Each side is narrower than MIN_SURFACE_WIDTH; the combined shelf is wide enough.
    lateral = -scene.nx * scene.depth
    split = scene.depth.copy()
    split[(abs(lateral) > 0.3) | (abs(lateral) < 0.06)] = np.nan
    assert detect_shelf_levels(split, scene.info, scene.transform) == pytest.approx(
        scene.levels, abs=0.01
    )


def test_small_box_top_is_not_an_additional_shelf(scene):
    depth = scene.depth.copy()
    box = np.divide(
        1.2 - 0.82, scene.ny, out=np.full_like(scene.ny, np.nan), where=scene.ny > 0
    )
    inside = (box >= 1.3) & (box <= 1.5) & (abs(scene.nx * box) < 0.11)
    nearer = inside & (~np.isfinite(depth) | (box < depth))
    depth[nearer] = box[nearer]
    assert detect_shelf_levels(depth, scene.info, scene.transform) == pytest.approx(
        scene.levels, abs=0.01
    )


def test_rotated_shelf_uses_its_own_footprint_axes(scene):
    half_yaw = np.radians(45) / 2
    c, s = np.cos(half_yaw), np.sin(half_yaw)
    scene.transform.transform.rotation = NS(
        x=-0.5 * (c + s), y=0.5 * (c - s), z=0.5 * (s - c), w=0.5 * (c + s)
    )
    assert detect_shelf_levels(
        scene.depth, scene.info, scene.transform
    ) == pytest.approx(scene.levels, abs=0.01)


def test_medium_object_top_is_narrower_than_shelf_structure(scene):
    depth = scene.depth.copy()
    box = np.divide(
        1.2 - 0.82, scene.ny, out=np.full_like(scene.ny, np.nan), where=scene.ny > 0
    )
    inside = (box >= 1.3) & (box <= 1.7) & (abs(scene.nx * box) < 0.29)
    nearer = inside & (~np.isfinite(depth) | (box < depth))
    depth[nearer] = box[nearer]
    assert detect_shelf_levels(depth, scene.info, scene.transform) == pytest.approx(
        scene.levels, abs=0.01
    )


def test_moderately_narrower_shelf_level_is_preserved(scene):
    depth = scene.depth.copy()
    z = 1.2 - scene.ny * depth
    depth[(abs(z - 0.90) < 0.01) & (abs(scene.nx * depth) > 0.36)] = np.nan
    assert detect_shelf_levels(depth, scene.info, scene.transform) == pytest.approx(
        scene.levels, abs=0.01
    )


def test_observed_undersides_are_not_reported_as_supports(scene):
    # Reflect the view vertically and place the camera below the same boards.
    scene.info.k[5] = 480 - scene.info.k[5] - 1
    scene.transform.transform.translation.z = 0.0
    with pytest.raises(ShelfCalibrationError, match="Detected 0 shelf levels"):
        detect_shelf_levels(scene.depth[::-1], scene.info, scene.transform)


@pytest.mark.parametrize("invalid_depth", [0.0, np.nan, np.inf])
def test_missing_or_invalid_depth_is_rejected(scene, invalid_depth):
    with pytest.raises(ShelfCalibrationError, match="Not enough valid depth"):
        detect_shelf_levels(
            np.full_like(scene.depth, invalid_depth), scene.info, scene.transform
        )


@pytest.mark.parametrize("focal", [0.0, -1.0, np.nan, np.inf])
def test_invalid_camera_intrinsics_are_rejected(scene, focal):
    scene.info.k[0] = focal
    with pytest.raises(ShelfCalibrationError, match="CameraInfo"):
        detect_shelf_levels(scene.depth, scene.info, scene.transform)


def test_invalid_transform_is_rejected(scene):
    scene.transform.transform.rotation = NS(x=0.0, y=0.0, z=0.0, w=0.0)
    with pytest.raises(ShelfCalibrationError, match="invalid quaternion"):
        detect_shelf_levels(scene.depth, scene.info, scene.transform)


@pytest.mark.parametrize("separation", [0.0, -0.1, np.nan])
def test_invalid_level_separation_is_rejected(scene, separation):
    with pytest.raises(ShelfCalibrationError, match="Minimum level separation"):
        detect_shelf_levels(
            scene.depth, scene.info, scene.transform, min_level_separation=separation
        )


@pytest.fixture
def growing_board():
    rows, columns = np.mgrid[:40, :120]
    points = np.stack((columns * 0.01, rows * 0.01, np.ones(rows.shape)), axis=-1)
    normals = np.full_like(points, np.nan)
    seed = np.zeros(rows.shape, dtype=bool)
    seed[10:25, 10:30] = True
    normals[seed] = [0, 0, 1]
    surface = _Surface(seed, np.median(points[seed], axis=0), np.array([0, 0, 1]), 300)
    return NS(points=points, normals=normals, surface=surface)


def test_growth_recovers_measured_board_with_missing_local_normals(growing_board):
    board = growing_board
    result = _grow_surface(board.points, board.normals, board.surface, 15)
    assert result.mask.all()
    assert result.center[2] == pytest.approx(1.0)


def test_growth_does_not_fill_unmeasured_depth(growing_board):
    board = growing_board
    board.points[28:33, 45:60] = np.nan
    result = _grow_surface(board.points, board.normals, board.surface, 15)
    assert not result.mask[28:33, 45:60].any()
    assert result.mask[:, 80:].any()


def test_growth_stops_at_vertical_wall_intersecting_board(growing_board):
    board = growing_board
    board.points[:, 80:, 0] = 0.8
    board.points[:, 80:, 2] += np.arange(40)[None, :] * 0.01
    board.normals[:, 80:] = [-1, 0, 0]
    result = _grow_surface(board.points, board.normals, board.surface, 15)
    assert result.mask[:, :80].all()
    assert not result.mask[:, 80:].any()


def test_growth_does_not_jump_to_distant_coplanar_object(growing_board):
    board = growing_board
    board.points[:, 60:, 0] += 1.0
    result = _grow_surface(board.points, board.normals, board.surface, 15)
    assert result.mask[:, :60].all()
    assert not result.mask[:, 60:].any()


def test_growth_excludes_observed_undersides(growing_board):
    board = growing_board
    board.normals[:, 80:] = [0, 0, -1]
    result = _grow_surface(board.points, board.normals, board.surface, 15)
    assert not result.mask[:, 80:].any()

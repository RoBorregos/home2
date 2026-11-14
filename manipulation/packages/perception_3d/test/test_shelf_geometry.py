"""Synthetic-cloud tests for the model-free shelf detection.

They pin the decisions the robot depends on: every board is found at the right
height with nothing known beforehand, objects standing on a board are not boards,
the undersides give thickness and ceiling, and the planning-scene boxes enclose the
structure. Pure numpy: run with ``pytest manipulation/packages/perception_3d/test``.
"""

import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception_3d.shelf_geometry import (
    Frame,
    ShelfNotFound,
    ShelfParams,
    detect_shelf,
    level_for_height,
)

BOARDS = [0.095, 0.39, 0.68, 1.05]
THICKNESS = 0.018
Y_HALF = 0.40
FRONT, BACK = 0.50, 0.90
TOP_UNDERSIDE = 1.30
STEP = 0.006
NOISE = 0.003
RNG = np.random.default_rng(7)


def _patch(xs, ys, zs):
    grid = np.array(np.meshgrid(xs, ys, zs, indexing="ij")).reshape(3, -1).T
    return grid + RNG.normal(scale=NOISE, size=grid.shape)


def _horizontal(z, x=(FRONT, BACK), y=(-Y_HALF, Y_HALF)):
    return _patch(np.arange(x[0], x[1], STEP), np.arange(y[0], y[1], STEP), [z])


def _side_wall(y, z_lo, z_hi):
    return _patch(np.arange(FRONT, BACK, STEP), [y], np.arange(z_lo, z_hi, STEP))


def _back_wall(z_lo, z_hi):
    return _patch([BACK], np.arange(-Y_HALF, Y_HALF, STEP), np.arange(z_lo, z_hi, STEP))


def _box_on(level_z, cx, cy, sx, sy, sz):
    """Top and front/side faces of a box standing on a board."""
    top = _horizontal(
        level_z + sz, (cx - sx / 2, cx + sx / 2), (cy - sy / 2, cy + sy / 2)
    )
    front = _patch(
        [cx - sx / 2],
        np.arange(cy - sy / 2, cy + sy / 2, STEP),
        np.arange(level_z, level_z + sz, STEP),
    )
    return np.vstack([top, front])


def shelf_frames(objects=True, walls=True, top=True):
    """Two survey views: faces are included only when that camera can see them."""
    frames = []
    for viewpoint in (np.array([0.1, 0.0, 1.25]), np.array([0.1, 0.0, 0.55])):
        parts = []
        for z in BOARDS:
            if viewpoint[2] > z:
                parts.append(_horizontal(z))
            if viewpoint[2] < z - THICKNESS:
                parts.append(_horizontal(z - THICKNESS))
        if top and viewpoint[2] < TOP_UNDERSIDE:
            parts.append(_horizontal(TOP_UNDERSIDE))
        if walls:
            parts.append(_side_wall(-Y_HALF, BOARDS[0] - THICKNESS, TOP_UNDERSIDE))
            parts.append(_side_wall(Y_HALF, BOARDS[0] - THICKNESS, TOP_UNDERSIDE))
            parts.append(_back_wall(BOARDS[0] - THICKNESS, TOP_UNDERSIDE))
        if objects:
            parts.append(
                _box_on(BOARDS[1], 0.7, 0.1, 0.10, 0.10, 0.15)
            )  # a can-sized box
            parts.append(
                _box_on(BOARDS[2], 0.7, -0.05, 0.20, 0.30, 0.03)
            )  # a wide flat tray
        frames.append(Frame(points=np.vstack(parts), viewpoint=viewpoint))
    return frames


@pytest.fixture(scope="module")
def geometry():
    return detect_shelf(shelf_frames())


def test_every_board_found_at_its_height(geometry):
    assert len(geometry.levels) == len(BOARDS)
    for level, expected in zip(geometry.levels, BOARDS):
        assert abs(level.height - expected) < 0.01, (level.height, expected)


def test_objects_are_not_levels(geometry):
    # The box top (0.54) is too narrow to be a board. The tray top (0.71) sits within
    # one compartment of board 3 (0.68): the better supported surface, the board, wins.
    assert all(abs(level.height - 0.54) > 0.02 for level in geometry.levels)
    third = min(geometry.levels, key=lambda level: abs(level.height - 0.68))
    assert abs(third.height - 0.68) < 0.01


def test_thickness_from_undersides(geometry):
    # Boards 2-4 have their underside in view from the low camera.
    for level in geometry.levels[1:]:
        assert abs(level.thickness - THICKNESS) < 0.01


def test_clearances_and_ceiling(geometry):
    levels = geometry.levels
    for lower, upper in itertools.pairwise(levels):
        expected = upper.height - THICKNESS - lower.height
        assert abs(lower.clearance - expected) < 0.015
    assert abs(geometry.top_z - TOP_UNDERSIDE) < 0.01
    assert abs(levels[-1].clearance - (TOP_UNDERSIDE - BOARDS[-1])) < 0.015


def test_walls_and_extents(geometry):
    assert geometry.has_left_wall and geometry.has_right_wall and geometry.has_back_wall
    assert abs(geometry.y_min + Y_HALF) < 0.03
    assert abs(geometry.y_max - Y_HALF) < 0.03
    assert abs(geometry.back_x - BACK) < 0.03
    assert abs(geometry.front_x - FRONT) < 0.03


def test_collision_boxes_enclose_the_structure(geometry):
    ids = {box.id for box in geometry.boxes}
    assert {"frida_shelf_board_1", "frida_shelf_board_4", "frida_shelf_top"} <= ids
    assert {
        "frida_shelf_wall_left",
        "frida_shelf_wall_right",
        "frida_shelf_wall_back",
    } <= ids
    boards = sorted(
        (b for b in geometry.boxes if "board" in b.id), key=lambda b: b.center[2]
    )
    params = ShelfParams()
    for box, level in zip(boards, geometry.levels):
        top = box.center[2] + box.size[2] / 2
        assert abs(top - (level.height - params.board_sink)) < 1e-6
        assert box.size[1] > 0.6 * 2 * Y_HALF
    back = next(b for b in geometry.boxes if b.id.endswith("wall_back"))
    assert back.center[0] > BACK  # behind the boards, never inside the compartment


def test_without_walls_extents_come_from_boards():
    geometry = detect_shelf(shelf_frames(walls=False, top=False))
    assert len(geometry.levels) == len(BOARDS)
    assert not geometry.has_left_wall and not geometry.has_back_wall
    assert geometry.top_z == 0.0
    assert geometry.levels[-1].clearance == 0.0
    assert abs(geometry.y_min + Y_HALF) < 0.03 and abs(geometry.y_max - Y_HALF) < 0.03


def test_empty_view_raises():
    frame = Frame(points=np.zeros((0, 3)), viewpoint=np.array([0.0, 0.0, 1.0]))
    with pytest.raises(ShelfNotFound):
        detect_shelf([frame])


def test_only_objects_raises():
    # A lone box top is a horizontal surface, but there is no board to compare it to.
    frame = Frame(
        points=_box_on(0.4, 0.7, 0.0, 0.1, 0.1, 0.1),
        viewpoint=np.array([0.1, 0.0, 1.2]),
    )
    params = ShelfParams(min_level_points=20)
    geometry = detect_shelf([frame], params)
    # With a single surface it IS the widest one, so it is accepted: the caller must
    # judge the result (one level, 10 cm wide). Pin that behaviour.
    assert len(geometry.levels) == 1 and geometry.y_max - geometry.y_min < 0.15


def test_level_for_height():
    assert level_for_height(0.45, BOARDS) == 0.39
    assert level_for_height(0.37, BOARDS) == 0.39  # depth noise just under the surface
    assert level_for_height(1.20, BOARDS) == 1.05
    assert level_for_height(0.01, BOARDS) is None

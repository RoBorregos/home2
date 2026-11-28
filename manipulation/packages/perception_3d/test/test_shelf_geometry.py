"""Synthetic-cloud tests for the model-free shelf detection (pure numpy)."""

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
    # Four boards from above plus the closed top, seen only from below.
    assert len(geometry.levels) == len(BOARDS) + 1
    for level, expected in zip(geometry.levels, BOARDS):
        assert abs(level.height - expected) < 0.01, (level.height, expected)
        assert not level.seen_from_below
    top = geometry.levels[-1]
    assert top.seen_from_below
    assert abs(top.height - (TOP_UNDERSIDE + ShelfParams().default_thickness)) < 0.01


def test_objects_are_not_levels(geometry):
    # The box top (0.54) is too narrow; the tray top (0.71) loses to board 3 (0.68).
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
    # The compartment under the closed top is bounded by it; nothing above the top.
    assert abs(levels[-2].clearance - (TOP_UNDERSIDE - BOARDS[-1])) < 0.015
    assert levels[-1].clearance == 0.0


def test_walls_and_extents(geometry):
    assert geometry.has_left_wall and geometry.has_right_wall and geometry.has_back_wall
    assert abs(geometry.y_min + Y_HALF) < 0.03
    assert abs(geometry.y_max - Y_HALF) < 0.03
    assert abs(geometry.back_x - BACK) < 0.03
    assert abs(geometry.front_x - FRONT) < 0.03


def test_collision_boxes_enclose_the_structure(geometry):
    ids = {box.id for box in geometry.boxes}
    assert {"frida_shelf_board_1", "frida_shelf_board_4", "frida_shelf_board_5"} <= ids
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
    # A lone surface is the widest one, so it is accepted; the caller must judge it.
    assert len(geometry.levels) == 1 and geometry.y_max - geometry.y_min < 0.15


def test_back_wall_seen_only_as_a_strip():
    # In the sim only a ~9 cm strip of the back panel was in view.
    frames = shelf_frames(walls=False)
    strip = _patch(
        [BACK], np.arange(-Y_HALF, Y_HALF, STEP), np.arange(0.69, 0.78, STEP)
    )
    frames[0] = Frame(
        points=np.vstack([frames[0].points, strip]), viewpoint=frames[0].viewpoint
    )
    geometry = detect_shelf(frames)
    assert geometry.has_back_wall
    assert abs(geometry.back_x - BACK) < 0.03


def test_narrow_face_behind_the_boards_is_not_a_back_wall():
    # A box face near the back is robot-facing and behind the boards, but narrow.
    frames = shelf_frames(walls=False)
    face = _patch(
        [BACK - 0.02], np.arange(-0.05, 0.05, STEP), np.arange(0.40, 0.50, STEP)
    )
    frames[1] = Frame(
        points=np.vstack([frames[1].points, face]), viewpoint=frames[1].viewpoint
    )
    assert not detect_shelf(frames).has_back_wall


def test_board_above_the_cameras_bounds_the_compartment():
    # A board above every camera is seen only from below and still bounds the compartment.
    frames = shelf_frames(objects=False, top=False)
    ceiling = 1.40
    frames = [
        Frame(points=np.vstack([f.points, _horizontal(ceiling)]), viewpoint=f.viewpoint)
        for f in frames
    ]
    geometry = detect_shelf(frames)
    assert geometry.levels[-1].seen_from_below
    assert abs(geometry.levels[-2].height - BOARDS[-1]) < 0.01
    assert abs(geometry.levels[-2].clearance - (ceiling - BOARDS[-1])) < 0.015


def test_boxes_keep_a_safety_margin_except_on_board_tops(geometry):
    # Flush boxes let MoveIt run the gripper into the cabinet front in the sim.
    p = ShelfParams()
    boxes = {box.id: box for box in geometry.boxes}
    board = boxes["frida_shelf_board_2"]
    level = geometry.levels[1]
    front = board.center[0] - board.size[0] / 2
    under = board.center[2] - board.size[2] / 2
    top = board.center[2] + board.size[2] / 2
    assert front == pytest.approx(geometry.front_x - p.padding)
    assert under == pytest.approx(
        level.height - p.board_sink - level.thickness - p.padding
    )
    assert top == pytest.approx(level.height - p.board_sink)  # grasps reach the base
    left = boxes["frida_shelf_wall_left"]
    assert left.center[1] + left.size[1] / 2 == pytest.approx(
        geometry.y_min + p.padding
    )
    right = boxes["frida_shelf_wall_right"]
    assert right.center[1] - right.size[1] / 2 == pytest.approx(
        geometry.y_max - p.padding
    )


def _with(frames, index, *parts):
    """``frames`` with ``parts`` added to the points of frame ``index``."""
    frames = list(frames)
    frame = frames[index]
    frames[index] = Frame(
        points=np.vstack([frame.points, *parts]), viewpoint=frame.viewpoint
    )
    return frames


def _wall_boxes(geometry):
    return {box.id.removeprefix("frida_shelf_"): box for box in geometry.boxes}


def test_clutter_in_front_is_not_a_side_wall():
    # On the robot a plinth in front of the shelf beat a side wall seen only in one
    # compartment: the wall box landed 12 cm outside the real wall.
    clutter = _patch(
        np.arange(FRONT - 0.25, FRONT - 0.12, STEP),
        [-Y_HALF - 0.10],
        np.arange(0.10, 0.40, STEP),
    )
    partial_wall = _patch(
        np.arange(FRONT, BACK, STEP), [-Y_HALF], np.arange(0.45, 0.63, STEP)
    )
    frames = _with(shelf_frames(walls=False), 0, clutter, partial_wall)
    geometry = detect_shelf(frames)
    assert geometry.has_left_wall
    assert abs(geometry.y_min + Y_HALF) < 0.02, geometry.y_min


def test_walls_not_seen_are_still_boxed():
    # A missing wall box let MoveIt plan through a real wall.
    geometry = detect_shelf(shelf_frames(walls=False, top=False))
    assert not (geometry.has_left_wall or geometry.has_right_wall)
    boxes = _wall_boxes(geometry)
    assert {"wall_left", "wall_right", "wall_back"} <= set(boxes)
    left = boxes["wall_left"]
    assert left.center[1] + left.size[1] / 2 == pytest.approx(
        geometry.y_min + ShelfParams().padding
    )


def test_walls_not_seen_can_be_left_open():
    params = ShelfParams(assume_walls=False)
    geometry = detect_shelf(shelf_frames(walls=False, top=False), params)
    assert not any("wall" in box.id for box in geometry.boxes)


def test_walls_rise_past_the_top_board_of_an_open_shelf():
    # The robot's shelf has 23 cm of wall above its top board and no lid.
    geometry = detect_shelf(shelf_frames(top=False))
    assert geometry.wall_top > BOARDS[-1] + 0.2
    for name in ("wall_left", "wall_right", "wall_back"):
        box = _wall_boxes(geometry)[name]
        assert box.center[2] + box.size[2] / 2 == pytest.approx(geometry.wall_top)


def test_walls_reach_the_floor():
    p = ShelfParams()
    geometry = detect_shelf(shelf_frames())
    for name in ("wall_left", "wall_right", "wall_back"):
        box = _wall_boxes(geometry)[name]
        assert box.center[2] - box.size[2] / 2 == pytest.approx(p.z_min)


def test_room_wall_far_behind_is_not_the_back_wall():
    room = _patch(
        [BACK + 0.25], np.arange(-Y_HALF, Y_HALF, STEP), np.arange(0.2, 1.2, STEP)
    )
    geometry = detect_shelf(_with(shelf_frames(walls=False), 0, room))
    assert not geometry.has_back_wall
    assert geometry.back_x < BACK + 0.05


def test_assumed_walls_are_as_tall_as_the_shelf():
    # No side wall in view, but the back panel shows the shelf rises to 1.30 m: the
    # assumed side walls must not stop at the top board (1.05 m).
    back_panel = _patch(
        [BACK], np.arange(-Y_HALF, Y_HALF, STEP), np.arange(0.6, 1.30, STEP)
    )
    frames = _with(shelf_frames(walls=False, top=False), 0, back_panel)
    geometry = detect_shelf(frames)
    assert not (geometry.has_left_wall or geometry.has_right_wall)
    for name in ("wall_left", "wall_right"):
        box = _wall_boxes(geometry)[name]
        assert box.center[2] + box.size[2] / 2 > 1.30


def test_object_side_inside_the_boards_is_not_a_wall():
    # With its wall out of view, a tall box 10 cm in from the edge became the wall.
    side = _patch(
        np.arange(0.60, 0.80, STEP), [Y_HALF - 0.10], np.arange(0.68, 1.00, STEP)
    )
    geometry = detect_shelf(_with(shelf_frames(walls=False, top=False), 0, side))
    assert not geometry.has_right_wall
    assert geometry.y_max > Y_HALF - 0.04


def test_assumed_walls_grow_outward_not_inward():
    p = ShelfParams()
    geometry = detect_shelf(shelf_frames(walls=False, top=False))
    boxes = _wall_boxes(geometry)
    left, right = boxes["wall_left"], boxes["wall_right"]
    assert left.center[1] - left.size[1] / 2 == pytest.approx(
        geometry.y_min - p.assumed_wall_thickness
    )
    assert right.center[1] + right.size[1] / 2 == pytest.approx(
        geometry.y_max + p.assumed_wall_thickness
    )
    # A seen wall keeps the thin box.
    seen = _wall_boxes(detect_shelf(shelf_frames()))["wall_left"]
    assert seen.size[1] == pytest.approx(p.wall_thickness + p.padding)


def test_sparse_floor_edge_does_not_stretch_the_shelf():
    # On the robot the floor's edge (z=0.05) showed up 1.3 m wide but 24 % filled: it
    # set the boards' extent and the real right wall, 29 cm inside it, was dropped.
    floor = np.vstack(
        [
            _patch(np.arange(0.35, 0.50, STEP), np.arange(y0, y0 + 0.2, STEP), [0.05])
            for y0 in (-0.75, 0.45)
        ]
    )
    frames = _with(shelf_frames(objects=False, top=False), 1, floor)
    geometry = detect_shelf(frames, ShelfParams(z_min=0.02))
    assert geometry.has_left_wall and geometry.has_right_wall
    assert abs(geometry.y_min + Y_HALF) < 0.02 and abs(geometry.y_max - Y_HALF) < 0.02


def test_tall_object_face_near_the_back_is_not_the_back_wall():
    # On the robot a 14 cm wide, 30 cm tall box face beat the real back panel.
    face = _patch(
        [BACK - 0.07], np.arange(-0.35, -0.21, STEP), np.arange(0.40, 0.70, STEP)
    )
    geometry = detect_shelf(_with(shelf_frames(), 1, face))
    assert geometry.has_back_wall
    assert abs(geometry.back_x - BACK) < 0.02, geometry.back_x

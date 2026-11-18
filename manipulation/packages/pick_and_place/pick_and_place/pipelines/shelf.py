"""The shelf in front of the robot: survey it once, then face one level at a time.

SCAN_SHELF runs ``scan``. The core keeps the ShelfScan while the base stays put, owns
its planning scene boxes, and hands it to the pick and place pipelines, which read the
boards and the extent from it. See docs/ai/shelf_detection.md.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from frida_constants.manipulation_constants import (
    ARM_HIGHEST_0_0_HEIGHT,
    ARM_LOWEST_0_0_HEIGHT,
    FAST_VELOCITY,
    SHELF_LEVEL_SETTLE,
    SHELF_SURVEY_POSES,
    SHELF_SURVEY_SETTLE,
)
from frida_motion_planning.utils.tf_utils import look_at, transform_point
from frida_pymoveit2.robots import xarm6
from geometry_msgs.msg import PointStamped

MAX_BASE_SHIFT = 0.03  # m the base may move before the scan is stale
MAX_BASE_TURN = math.radians(3.0)
LEVEL_BELOW = 0.05  # m an object's detected point may sit below its board
# Facing a level for a pick: camera this far past the board front and above the board,
# looking at its centre; VIEW_MAX_REACH from the centre when that is farther from the base.
VIEW_DISTANCE = 0.5
VIEW_HEIGHT = 0.4
VIEW_MAX_REACH = 0.5
# Facing a level for a place: camera near the base, this high above the board.
PLACE_VIEW_HEIGHT = 0.2


@dataclass
class ShelfScan:
    """The last shelf survey, in base_link."""

    levels: list  # ShelfLevel messages, ascending; objects filled in
    boxes: list  # CollisionObject messages for the planning scene
    front_x: float
    back_x: float
    y_min: float
    y_max: float
    base: Optional[np.ndarray] = None  # (x, y, yaw) of base_link in odom at the scan

    @property
    def board_levels(self) -> List[Tuple[float, float]]:
        """(height, thickness) of each board, ascending."""
        return [(lv.height, lv.thickness) for lv in self.levels]

    @property
    def extent(self) -> Tuple[float, float, float, float]:
        return self.front_x, self.back_x, self.y_min, self.y_max

    def level_of(self, z: float) -> int:
        """The level (1 = lowest) whose compartment holds height ``z``, 0 for none."""
        heights = [lv.height for lv in self.levels]
        for i, height in enumerate(heights):
            top = heights[i + 1] - LEVEL_BELOW if i + 1 < len(heights) else math.inf
            if height - LEVEL_BELOW <= z < top:
                return i + 1
        return 0

    def level_with(self, label: str) -> int:
        """The first level the scan saw ``label`` on, 0 when it did not."""
        for i, level in enumerate(self.levels):
            if label in level.objects:
                return i + 1
        return 0

    def stale(self, base: Optional[np.ndarray]) -> str:
        """Why the scan no longer matches the base pose ``base``, or ''."""
        if self.base is None:
            return ""  # no odometry at the scan: a static base (simulation)
        if base is None:
            return "odometry lost since the shelf scan"
        shift = float(np.linalg.norm(base[:2] - self.base[:2]))
        turn = abs(math.remainder(float(base[2] - self.base[2]), math.tau))
        if shift > MAX_BASE_SHIFT or turn > MAX_BASE_TURN:
            return f"base moved {shift * 100:.1f} cm / {math.degrees(turn):.1f} deg"
        return ""


def base_pose(arm) -> Optional[np.ndarray]:
    """(x, y, yaw) of base_link in odom, or None without odometry."""
    pose = arm.link_pose("base_link", "odom")
    if pose is None:
        return None
    q = pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return np.array([pose.pose.position.x, pose.pose.position.y, yaw])


# ======================================================================
# Survey
# ======================================================================


def scan(arm, perception) -> Optional[ShelfScan]:
    """Visit the survey poses, capture each view, detect the shelf. None on failure."""
    log = arm.logger
    arm.set_context("scan_shelf")
    perception.detect_shelf(reset=True)
    captured, seen = 0, []
    with arm.phase("survey"):
        for pose in SHELF_SURVEY_POSES:
            if not arm.move_to_named_position(pose, velocity=FAST_VELOCITY):
                log.warn(f"Could not reach {pose}; skipping this view")
                continue
            time.sleep(SHELF_SURVEY_SETTLE)
            captured += perception.detect_shelf(capture=True) is not None
            seen += _seen_in_base(arm, perception.detections())
    if not captured:
        log.error("No survey view captured")
        return None
    with arm.phase("detect"):
        result = perception.detect_shelf(detect=True)
    if result is None or not result.levels:
        log.error("Shelf detection failed")
        return None

    shelf = ShelfScan(
        levels=list(result.levels),
        boxes=list(result.collision_objects),
        front_x=result.front_x,
        back_x=result.back_x,
        y_min=result.y_min,
        y_max=result.y_max,
        base=base_pose(arm),
    )
    for label, z in seen:
        level = shelf.level_of(z)
        if level and label not in shelf.levels[level - 1].objects:
            shelf.levels[level - 1].objects.append(label)
    for i, level in enumerate(shelf.levels, start=1):
        log.info(f"Shelf level {i} at {level.height:.3f} m: {list(level.objects)}")
    return shelf


def _seen_in_base(arm, detections) -> List[Tuple[str, float]]:
    """(label, base_link z) of each detection with a usable 3D point."""
    seen = []
    for detection in detections:
        point = _in_base(arm, detection.point3d)
        if point is not None:
            seen.append((detection.label_text, point.point.z))
    return seen


def _in_base(arm, point: PointStamped) -> Optional[PointStamped]:
    """``point`` in base_link, or None when it is empty or has no TF."""
    if not point.header.frame_id:
        return None
    if point.header.frame_id == "base_link":
        return point
    ok, in_base = transform_point(point, "base_link", arm.tf_buffer)
    return in_base if ok else None


# ======================================================================
# Find an object and face a level
# ======================================================================


def find_object(arm, perception, shelf: ShelfScan, label: str, hint: int = 0) -> int:
    """Face the level that holds ``label`` and return it (1 = lowest), 0 if not seen.

    Tries ``hint``, then where the scan saw the label, then every other level.
    """
    order = [hint, shelf.level_with(label)] + list(range(1, len(shelf.levels) + 1))
    tried = []
    for level in order:
        if not 1 <= level <= len(shelf.levels) or level in tried:
            continue
        tried.append(level)
        if not face_level(arm, shelf, level):
            continue
        arm.clear_octomap()
        time.sleep(SHELF_LEVEL_SETTLE)  # the octomap and the detections catch up
        point = perception.locate_object(label)
        if not point.header.frame_id:
            arm.logger.info(f"No {label} seen from level {level}")
            continue
        in_base = _in_base(arm, point)
        seen_on = shelf.level_of(in_base.point.z) if in_base else 0
        if seen_on in (0, level):
            return level
        arm.logger.info(f"{label} framed from level {level} sits on level {seen_on}")
    return 0


def face_level(arm, shelf: ShelfScan, level: int, for_place: bool = False) -> bool:
    """Point the camera at ``level`` (1 = lowest): from its front edge for a pick,
    from near the base for a place."""
    board = shelf.levels[level - 1]
    with arm.phase(f"face_level_{level}"):
        if for_place:
            # From a pick's viewing pose MoveIt finds no IK for the pre-place.
            arm.move_to_named_position("front_stare", velocity=FAST_VELOCITY)
            camera = _point(0.1, 0.0, _camera_z(board.height + PLACE_VIEW_HEIGHT))
            target = _point(1.0, 0.0, board.height)
        else:
            centre = _point(
                (board.front_x + board.back_x) / 2,
                (board.y_min + board.y_max) / 2,
                board.height,
            )
            camera = _edge_view(board, centre.point)
            target = centre
        arm.logger.info(f"Facing shelf level {level} at {board.height:.3f} m")
        return arm.move_to_pose(
            look_at(camera, target),
            velocity=FAST_VELOCITY,
            target_link=xarm6.camera_frame_name(),
        )


def _edge_view(board, centre) -> PointStamped:
    """VIEW_DISTANCE past the board edge nearest the robot, on the line from the centre."""
    corners = [
        (board.front_x, board.y_min),
        (board.back_x, board.y_min),
        (board.back_x, board.y_max),
        (board.front_x, board.y_max),
    ]
    best = None
    for a, b in zip(corners, corners[1:] + corners[:1]):
        out = np.array([(a[0] + b[0]) / 2 - centre.x, (a[1] + b[1]) / 2 - centre.y])
        norm = float(np.linalg.norm(out))
        if norm < 1e-6:
            continue
        view = np.array([centre.x, centre.y]) + out / norm * (norm + VIEW_DISTANCE)
        if float(np.linalg.norm(view)) > VIEW_MAX_REACH:
            view = np.array([centre.x, centre.y]) + out / norm * VIEW_MAX_REACH
        if best is None or np.linalg.norm(view) < np.linalg.norm(best):
            best = view
    return _point(best[0], best[1], _camera_z(centre.z + VIEW_HEIGHT))


def _camera_z(z: float) -> float:
    return min(max(z, ARM_LOWEST_0_0_HEIGHT), ARM_HIGHEST_0_0_HEIGHT)


def _point(x: float, y: float, z: float) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = "base_link"
    point.point.x, point.point.y, point.point.z = float(x), float(y), float(z)
    return point


__all__ = ["ShelfScan", "base_pose", "face_level", "find_object", "scan"]

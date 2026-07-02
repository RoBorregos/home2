#!/usr/bin/env python3

"""
Task Manager for Pick and Place Challenge - RoboCup @Home 2026

Strategy: maximize pick+place score in 7 minutes.
- Cleanup phase: up to max_cleanup_objects from dining table, sorted by pick reliability.
- Breakfast phase: bowl, cereal, milk, spoon placed on dining table with pour for cereal/milk.
- Plates are skipped (GPD cannot grasp flat objects).
- Cabinet shelf placement uses an HRI categorize_objects call to match categories.
"""

import json
import os
import time
from collections import defaultdict
from datetime import datetime
from enum import Enum
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers transform type)
from std_msgs.msg import String
from frida_interfaces.msg import GripperGraspState
from frida_constants.manipulation_constants import GRIPPER_GRASP_STATE_TOPIC
from std_srvs.srv import Empty
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.shelf_pick_logic import (
    find_target_on_level,
    height_matches_level,
    levels_from_sorted_heights,
)
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 2

# Stacking detection for pick ordering: object A sits on object B when A's
# base_link Z is at least STACK_Z_MIN above B's and their XY are within
# STACK_XY_MAX. Tunable; validate on the robot.
STACK_Z_MIN = 0.03
STACK_XY_MAX = 0.12

SHELF_LEVEL_NAMES = {1: "bottom", 2: "second", 3: "third", 4: "top"}

# The arm cannot pick or place below this base_link Z. The competition shelf's REAL
# level 1 (0.095 m) is announce-only: categorization names it (that scores the points),
# but places fall back to the lowest reachable level and pick sweeps skip it.
MIN_REACHABLE_SHELF_Z = 0.20

# Stand-off (m) when docking at the dishwasher and cooking_table (dishwasher-tab zone). Their front
# sits closer than a normal table, so docking flush bumps the base into them; a small offset stops
# the base short. Larger = shorter approach (stops further back). Tune on the robot.
DISHWASHER_DOCK_OFFSET = 0.32


class ObjectCategory(Enum):
    """Object categories for placement decisions"""

    CUTLERY = "cutlery"
    TABLEWARE = "tableware"
    TRASH = "trash"
    OTHER = "other"
    COMMON = "common"


class Location(Enum):
    """Known furniture locations in the Incheon 2026 kitchen (PPC test location)."""

    DINING_TABLE = "dining_table"  # dinner table (16): clean up + set breakfast
    EXTRA_SURFACE = "extra_surface"  # counter (11): 2 common objects (source, -20/obj)
    DISHWASHER = (
        "dishwasher"  # (14): cutlery/plate/cup placed INSIDE; bowl+spoon picked from ON TOP
    )
    DISHWASHER_TAB = "dishwasher_tab"  # nav: cooking_table (13): dishwasher tab source
    CABINET = "cabinet"  # (9, left wall): place "other"; milk + cereal source
    TRASH_BIN = "trash_bin"  # nav: trash (15): fruits (trash) go here
    BREAKFAST_SURFACE = "breakfast_surface"
    BREAKFAST_ITEMS = "breakfast_items"  # nav: dishwasher top — bowl + spoon picked from on top
    KITCHEN = "kitchen"


class ObjectInfo:
    """Detected object with pick/place tracking"""

    def __init__(
        self,
        name: str = None,
        category: ObjectCategory = None,
        bbox=None,
    ):
        self.name = name
        self.category = category
        self.bbox = bbox
        self.is_picked = False
        self.skipped = False  # gave up before grasping (distinct from is_picked)
        self.is_placed = False
        self.placement_location: Location = None


class PickAndPlaceTM(Node):
    """Task Manager for Pick and Place Challenge"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"

        # Cleanup Phase
        PERCEIVE_TABLE = "PERCEIVE_TABLE"
        ANNOUNCE_OBJECTS = "ANNOUNCE_OBJECTS"
        SCAN_CABINET_SHELVES = "SCAN_CABINET_SHELVES"
        SORT_OBJECTS = "SORT_OBJECTS"
        CLEANUP_LOOP = "CLEANUP_LOOP"
        PICK_OBJECT = "PICK_OBJECT"
        DETERMINE_PLACEMENT = "DETERMINE_PLACEMENT"
        CHECK_DISHWASHER = "CHECK_DISHWASHER"
        REQUEST_DISHWASHER_HELP = "REQUEST_DISHWASHER_HELP"
        NAVIGATE_TO_PLACEMENT = "NAVIGATE_TO_PLACEMENT"
        PLACE_OBJECT = "PLACE_OBJECT"

        # Breakfast Phase
        START_BREAKFAST_PREP = "START_BREAKFAST_PREP"
        GET_BREAKFAST_ITEMS = "GET_BREAKFAST_ITEMS"
        NAVIGATE_TO_ITEM_SOURCE = "NAVIGATE_TO_ITEM_SOURCE"
        PICK_BREAKFAST_ITEM = "PICK_BREAKFAST_ITEM"
        NAVIGATE_TO_DINING = "NAVIGATE_TO_DINING"
        POUR_INTO_BOWL = "POUR_INTO_BOWL"
        PLACE_BREAKFAST_ITEM = "PLACE_BREAKFAST_ITEM"

        END = "END"
        DEBUG = "DEBUG"

    def __init__(self):
        """Initialize the node"""
        super().__init__("pickandplace_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.PICK_AND_PLACE, mock_areas=[])

        # ==========================================================
        # COMPETITION CONFIG — adjust before each run
        # ==========================================================
        # Trash rule: category from objects.json or a specific object name.
        # trash_exceptions excludes logical names from the match.
        # Incheon 2026: FRUITS are the announced trash category (P&P map).
        self.trash_category = "fruit"
        self.trash_exceptions: list[str] = []  # milk is a drink, never trash; no exceptions needed
        # use_dishwasher gates the OPEN-the-dishwasher flow. False -> place cutlery/tableware on TOP
        # of the dishwasher (banks base place pts). True -> ask for help to open + place inside.
        self.use_dishwasher = False
        # TEMP: no inside-the-dishwasher place primitive yet. While True, cutlery/plate/cup that are
        # designated for the dishwasher are instead placed (normal top place) at the dishwasher-tab
        # zone (cooking_table). Flip to False once the inside-dishwasher place exists.
        self.place_dishwasher_at_tab = True
        self.use_extra_surface = False  # pick the 2 common objects from the counter (−20 pts/obj)
        self.use_grasp_detector = False  # gate picks on the grasp bit; False ignores it
        self.use_vision_confirmation = True  # vision re-look to confirm the pick
        # Octomap clearing before shelf grasps. OFF by design: a cleared octomap lets the
        # planner route through shelf structure it can no longer see — a failed pick is
        # recoverable, a collision is not. (Stability decision 2026-07-02.)
        self.use_octomap_clearing = False
        self.max_cleanup_objects = 3  # how many to clean before breakfast

        # YOLO name mapping: logical → detection class (the model's actual labels).
        # Incheon 2026: the cereal box is detected as blue_cereal_box (the model also has
        # brown_cereal_box — TODO confirm WHICH at Setup Days); breakfast milk is chocomilk_box.
        # NOTE: dishes/cutlery (bowl, spoon, cup, red_plate, knife) come from the OLD model in the
        # combined detector (object_detector_node runs multiple YOLO models). No `fork` label exists
        # in either model. See docs/task_manager/ppc/time_strategy_2026.md.
        self.yolo_names = {
            "cereal": "cornflakes",
            "milk": "milk",
        }

        # Shelf heights in base_link Z: the REAL competition shelf levels (1 = lowest),
        # read live from the arena calibration JSON in xarm_utils (recalibrate on site
        # with `ros2 run xarm_utils shelf_height_calibrator.py --arena N`). Select the
        # arena with FRIDA_ARENA=1|2|3. Level 1 (0.095) is below MIN_REACHABLE_SHELF_Z:
        # announce-only, never placed on or picked from.
        self.arena = int(os.environ.get("FRIDA_ARENA", "1"))
        try:
            from xarm_utils.shelf_levels import get_shelf_levels

            self.shelf_level_heights = levels_from_sorted_heights(get_shelf_levels(self.arena))
        except Exception as e:
            self.get_logger().warn(f"Arena shelf JSON unavailable ({e}); using fallback heights.")
            self.shelf_level_heights = {1: 0.095, 2: 0.39, 3: 0.68, 4: 1.05}
        reachable = [h for h in self.shelf_level_heights.values() if h >= MIN_REACHABLE_SHELF_Z]
        self.default_shelf_height = min(reachable or self.shelf_level_heights.values())

        # ==========================================================
        # END COMPETITION CONFIG
        # ==========================================================

        self.yolo_to_logical = {v: k for k, v in self.yolo_names.items()}

        # Object → category mapping from objects.json
        try:
            objects_path = (
                Path(get_package_share_directory("frida_constants")) / "data" / "objects.json"
            )
            with open(objects_path) as f:
                self._object_to_category = json.load(f).get("object_to_category", {})
        except Exception:
            self._object_to_category = {}

        # Navigation mapping: logical role -> raw kitchen waypoint as named in the arena map
        # (areas.json), i.e. the LocationsNames furniture. Nav records the literal furniture
        # names; WE map our roles onto them here. Confirm these strings match areas.json exactly.
        #   counter -> extra_surface (2 common objects)   cooking_table -> dishwasher tab source
        #   dinner_table -> dining table                  trashbin -> kitchen trash bin (fruits)
        #   dishwasher  -> place cutlery/plate/cup INSIDE; bowl+spoon are picked from ON TOP of it
        self.nav_locations = {
            Location.KITCHEN: ("kitchen", "safe_place"),
            Location.DINING_TABLE: ("kitchen", "dinner_table"),
            Location.EXTRA_SURFACE: ("kitchen", "counter"),
            Location.DISHWASHER: ("kitchen", "dishwasher"),
            Location.DISHWASHER_TAB: ("kitchen", "cooking_table"),
            Location.CABINET: ("kitchen", "cabinet"),
            Location.TRASH_BIN: ("kitchen", "trashbin"),
            Location.BREAKFAST_SURFACE: ("kitchen", "dinner_table"),
            # bowl + spoon are picked from ON TOP of the dishwasher (normal top pick).
            Location.BREAKFAST_ITEMS: ("kitchen", "dishwasher"),
        }

        # Object tracking
        self.detected_objects: list[ObjectInfo] = []
        self.current_object_index = 0
        self.grasped_object: ObjectInfo = None
        self.first_pick = True

        # Breakfast items: bowl first, cereal+milk from cabinet, spoon last
        self.breakfast_items = [
            {
                "name": "bowl",
                "location": Location.BREAKFAST_ITEMS,
                "picked": False,
                "placed": False,
                "close_to": "",
            },
            {
                "name": "cereal",
                "location": Location.CABINET,
                "picked": False,
                "placed": False,
                "close_to": "bowl",
            },
            {
                "name": "milk",
                "location": Location.CABINET,
                "picked": False,
                "placed": False,
                "close_to": "cereal",
            },
            {
                "name": "spoon",
                "location": Location.BREAKFAST_ITEMS,
                "picked": False,
                "placed": False,
                "close_to": "bowl",
            },
        ]
        self.current_breakfast_item: dict = None
        self.bowl_placed = False
        self.dishwasher_open = False
        # What the gripper physically holds (ObjectInfo or breakfast dict); anti-drop guard.
        self.carrying = None

        # Shelf scanning state
        self.shelves: dict[int, list[str]] = {}
        self.object_to_placing_shelf: dict[str, list[int]] = defaultdict(list)
        self.shelf_scanned = False
        self._cabinet_scan_fresh = False
        # Level cache learned during per-level scans (classname -> level height) so later
        # cabinet picks jump straight to the right level instead of re-sweeping every level.
        self.use_shelf_cache = True
        self._shelf_level_cache: dict[str, float] = {}
        self._shelf_fallback_heights: list[float] = []
        self._shelf_fallback_idx: int = 0
        self.shelf_level_threshold = 0.20
        self.shelf_level_down_threshold = 0.05
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Runtime state
        self.current_attempts = 0
        self._docked_at_table = False  # one dock per table visit
        self.current_location: Location = None
        self.running_task = True
        self.state_start_time = None
        self.state_times: dict = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        # Octomap clearing: stale accumulation over-constrains tight shelf grasps
        self._clear_octomap_client = self.create_client(Empty, "/clear_octomap")

        # Gripper grasp detection
        self._gripper_has_object = False
        self.create_subscription(
            GripperGraspState,
            GRIPPER_GRASP_STATE_TOPIC,
            self._gripper_grasp_cb,
            10,
        )

        self.display_pub = self.create_publisher(String, "/pickandplace/display/task_step", 10)

        self.current_state = PickAndPlaceTM.TaskStates.WAIT_FOR_BUTTON
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        CLog.fsm(self, "STATE", "PickAndPlaceTaskManager has started.")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _gripper_grasp_cb(self, msg: GripperGraspState):
        self._gripper_has_object = msg.object_detected

    def _track_state_change(self, new_state: str):
        """Track time spent in each state and reset attempt counter"""
        current_time = datetime.now()

        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent
            CLog.fsm(self, "TIMER", f"State '{self.previous_state}' took {time_spent:.2f}s")

        if self.previous_state != new_state:
            self.current_attempts = 0

        self.previous_state = new_state
        self.state_start_time = current_time

        if self.state_times:
            total_time = sum(self.state_times.values())
            CLog.fsm(self, "TIMER", f"Total time elapsed: {total_time:.2f}s")

        self.display_pub.publish(String(data=new_state.lower()))

        CLog.fsm(
            self,
            "STATE",
            f"{self.previous_state} → {self.current_state}"
            if self.previous_state != self.current_state
            else self.current_state,
        )

    def navigate_to_location(self, location: Location, say: bool = True):
        """Navigate to a PPC location using the nav_locations mapping"""
        if self.current_location == location:
            CLog.nav(self, "SKIP", f"Already at {location.value}, skipping navigation.")
            return Status.EXECUTION_SUCCESS
        # A real location change is about to happen. The octomap we may have
        # had for the cabinet is no longer fresh (navigation can clear it,
        # the camera is looking elsewhere during transit, and picks between
        # visits call clear_octomap). Invalidate the cabinet-freshness flag
        # so the next cabinet place will do exactly one scan at arrival.
        self._cabinet_scan_fresh = False
        room, sublocation = self.nav_locations[location]
        result = self.navigate_to(room, sublocation, say=say)
        if result == Status.EXECUTION_SUCCESS:
            self.current_location = location
        else:
            # Location unknown after a failed nav; do not skip the next request.
            self.current_location = None
        return result

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to a location with retry logic"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")

        if say:
            # Prefer the specific sublocation for user-facing messages so the
            # robot announces e.g. "the dining table" instead of the generic
            # "kitchen" room it happens to be in.
            target = sublocation if sublocation else location
            pretty_target = target.replace("_", " ")
            CLog.nav(self, "MOVE", f"Moving to {target}")
            self.subtask_manager.hri.say(f"Now I will go to the {pretty_target}.", wait=False)

        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)

        return result

    def timeout(self, duration: float = 2.0):
        """Non-blocking wait that keeps spinning ROS"""
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _detection_point(self, detection):
        """base_link (x, y, z) of a detection's 3D point, or None on TF failure.

        Uses tf2 directly with a bounded timeout so the task manager never hangs
        if a transform is temporarily unavailable.
        """
        try:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = "zed_left_camera_optical_frame"
            stamped_point.header.stamp = rclpy.time.Time().to_msg()  # latest available
            stamped_point.point.x = float(detection.px)
            stamped_point.point.y = float(detection.py)
            stamped_point.point.z = float(detection.pz)

            transformed = self.tf_buffer.transform(
                stamped_point,
                "base_link",
                timeout=Duration(seconds=1.0),
            )
            return (transformed.point.x, transformed.point.y, transformed.point.z)
        except TransformException as e:
            CLog.vision(self, "TF", f"Transform failed: {e}", level="warn")
            return None
        except Exception as e:
            CLog.vision(self, "TF", f"Error converting point: {e}", level="error")
            return None

    def convert_to_height(self, detection) -> float | None:
        """base_link Z of a detection's 3D point, or None on TF failure."""
        point = self._detection_point(detection)
        return point[2] if point is not None else None

    def _table_counts(self):
        """Detect at the current pose and count detections per class."""
        from task_manager.utils.grasp_confirmation import count_by_class

        _, dets = self.subtask_manager.vision.detect_objects()
        return count_by_class([d.classname for d in (dets or [])])

    def _confirm_pick_by_vision(self, before_counts):
        """Re-look at the table; fail only if the grasped object is clearly still there."""
        from task_manager.utils.grasp_confirmation import picked_ok

        name = self.grasped_object.name
        target = self._to_yolo_name(name).lower()
        if not before_counts or before_counts.get(target, 0) == 0:
            return Status.EXECUTION_SUCCESS
        self.subtask_manager.manipulation.move_to_position("table_stare")
        after = self._table_counts()
        if picked_ok(before_counts, after, target):
            CLog.manip(self, "PICK", f"Vision confirmed {name} removed.", level="success")
            return Status.EXECUTION_SUCCESS
        CLog.manip(self, "PICK", f"Vision: {name} still on the table.", level="warn")
        return Status.EXECUTION_ERROR

    def _shelf_counts(self, detections, level):
        """Count detections at a shelf level (filtered by height) per class."""
        from task_manager.utils.grasp_confirmation import count_by_class

        dets = self._filter_detections_by_height(detections or [], level)
        return count_by_class([d.classname for d in dets])

    def _confirm_pick_shelf(self, before_counts, target_name, level):
        """Re-look at the shelf level; fail only if the object is clearly still there."""
        from task_manager.utils.grasp_confirmation import count_by_class, picked_ok

        target = self._to_yolo_name(target_name).lower()
        if not before_counts or before_counts.get(target, 0) == 0:
            return Status.EXECUTION_SUCCESS
        self.subtask_manager.manipulation.get_optimal_position_for_plane(
            level, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.timeout(3.0)
        _, dets = self.subtask_manager.vision.detect_objects()
        dets = self._filter_detections_by_height(dets or [], level)
        after = count_by_class([d.classname for d in dets])
        if picked_ok(before_counts, after, target):
            CLog.manip(
                self,
                "PICK",
                f"Vision confirmed {target_name} removed.",
                level="success",
            )
            return Status.EXECUTION_SUCCESS
        CLog.manip(
            self,
            "PICK",
            f"Vision: {target_name} still on the shelf.",
            level="warn",
        )
        return Status.EXECUTION_ERROR

    def _filter_detections_by_height(self, detections, target_height: float) -> list:
        """Filter detections to only include objects near the target shelf height."""
        filtered = []
        for det in detections:
            height = self.convert_to_height(det)
            if height is None:
                continue
            distance = height - target_height
            if (distance < 0 and abs(distance) < self.shelf_level_down_threshold) or (
                distance >= 0 and distance < self.shelf_level_threshold
            ):
                filtered.append(det)
                CLog.vision(
                    self,
                    "SHELF",
                    f"{det.classname} at height {height:.2f}m matches shelf at {target_height:.2f}m",
                )
            else:
                CLog.vision(
                    self,
                    "SHELF",
                    f"{det.classname} at height {height:.2f}m skipped (target {target_height:.2f}m)",
                )
        return filtered

    def _scan_shelf_level(self, height: float) -> None:
        """Scan a single shelf level: move arm, wait for octomap, detect objects."""
        self.subtask_manager.manipulation.get_optimal_position_for_plane(
            height, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.timeout(3.0)

    def _clear_octomap(self):
        """Clear the MoveIt octomap so points accumulated from prior picks and
        level scans do not over-constrain the next grasp."""
        try:
            if self._clear_octomap_client.wait_for_service(timeout_sec=2.0):
                self._clear_octomap_client.call_async(Empty.Request())
                CLog.manip(self, "PICK", "Cleared octomap before shelf grasp.")
            else:
                CLog.manip(self, "PICK", "clear_octomap unavailable.", level="warn")
        except Exception as e:
            CLog.manip(self, "PICK", f"clear_octomap failed: {e}", level="warn")

    def _visit_shelf_level(self, height: float, object_name: str):
        """Move to a shelf level, settle, detect, and learn what sits there.

        Returns (found, before_counts): whether object_name is framed at this level and,
        if so, the per-class counts for the vision confirmation. Only detections whose
        height actually matches this level are cached (classname -> height): the camera
        also frames adjacent levels, and caching those would send later picks to the
        wrong level first.
        """
        self.subtask_manager.manipulation.get_optimal_position_for_plane(
            height, tolerance=0.1, table_or_shelf=False, approach_plane=True
        )
        self.timeout(3.0)  # let the octomap settle at this level
        status, detections = self.subtask_manager.vision.detect_objects()
        retry = 0
        while status != Status.EXECUTION_SUCCESS and retry < 3:
            self.timeout(1.0)
            status, detections = self.subtask_manager.vision.detect_objects()
            retry += 1
        if status != Status.EXECUTION_SUCCESS or not detections:
            return (False, None)
        candidates = [
            (det.classname, h)
            for det in detections
            if (h := self.convert_to_height(det)) is not None
        ]
        for name, h in candidates:
            if name and height_matches_level(h, height):
                self._shelf_level_cache[name.lower()] = height
        if find_target_on_level(candidates, object_name, height) is not None:
            return (True, self._shelf_counts(detections, height))
        return (False, None)

    def _pick_from_shelf(self, object_name: str, level_heights: dict, say_name: str = None) -> int:
        """Find the target by detecting at each shelf level, then pick from that level.

        table_stare frames only the lower levels, so an object on a high level (cereal
        on L3) was missed. Here each level's own viewing pose detects + builds its
        octomap; the pick keeps that pose via in_configuration. See
        docs/ai/shelf_pick_plan.md.

        Fast path: if an earlier level scan already located this object, jump straight
        to that level (one confirm detect) instead of re-sweeping every level. A miss at
        the cached level drops the entry and falls back to the full sweep.
        """
        found_level = None
        before_counts = None

        cached = self._shelf_level_cache.get(object_name.lower()) if self.use_shelf_cache else None
        if cached is not None:
            CLog.manip(
                self, "PICK", f"Cached shelf level {cached:.3f} for {object_name} — skipping sweep."
            )
            found, before_counts = self._visit_shelf_level(cached, object_name)
            if found:
                found_level = cached
            else:
                CLog.manip(
                    self,
                    "PICK",
                    f"{object_name} not at cached level; falling back to a full sweep.",
                    level="warn",
                )
                self._shelf_level_cache.pop(object_name.lower(), None)

        if found_level is None:
            for height in sorted(level_heights.values()):
                if height < MIN_REACHABLE_SHELF_Z:
                    continue  # real level 1: too low to grasp from, announce-only
                found, before_counts = self._visit_shelf_level(height, object_name)
                if found:
                    found_level = height
                    break

        if found_level is None:
            CLog.manip(
                self,
                "PICK",
                f"{object_name} not found on any shelf level.",
                level="error",
            )
            return Status.EXECUTION_ERROR

        CLog.manip(self, "PICK", f"Found {object_name} at shelf height {found_level:.3f}.")
        self.announce_objects([object_name])
        if say_name:
            self.subtask_manager.hri.say(f"I will pick the {say_name}.", wait=False)

        if self.use_octomap_clearing:
            # Clear the octomap so points accumulated from prior picks/levels do not
            # over-constrain the grasp; let it rebuild fresh at this level.
            self._clear_octomap()
            self.timeout(3.0)
        # Arm is at the found level's pose; keep it (in_configuration).
        status = self.subtask_manager.manipulation.pick_object(
            object_name, in_configuration=True, scan_environment=True
        )
        if status == Status.EXECUTION_SUCCESS and self.use_vision_confirmation:
            status = self._confirm_pick_shelf(before_counts, object_name, found_level)
        return status

    def categorize_object(self, obj_name: str) -> ObjectCategory:
        """Assign an object to a category based on its name"""
        cutlery = ["fork", "knife", "spoon"]
        tableware = ["red_plate", "cup", "mug", "bowl"]

        name = obj_name.lower()
        if name in cutlery:
            return ObjectCategory.CUTLERY
        if name in tableware:
            return ObjectCategory.TABLEWARE
        if self._is_trash(name):
            return ObjectCategory.TRASH
        return ObjectCategory.OTHER

    def _is_trash(self, logical_name: str) -> bool:
        """True if object matches trash_category (as object name or as objects.json category)."""
        if not self.trash_category:
            return False
        trash_key = self.trash_category.lower()
        exceptions = {x.lower() for x in self.trash_exceptions}
        if logical_name in exceptions:
            return False
        if logical_name == trash_key:
            return True
        yolo_name = self._to_yolo_name(logical_name).lower()
        return self._object_to_category.get(yolo_name, "").lower() == trash_key

    def _to_yolo_name(self, logical_name: str) -> str:
        """Translate a logical object name to the YOLO class name."""
        return self.yolo_names.get(logical_name.lower(), logical_name)

    def determine_placement_location(self, obj: ObjectInfo) -> Location:
        """Return the target furniture for an object."""
        if obj.category == ObjectCategory.TRASH:
            return Location.TRASH_BIN
        elif obj.category in (ObjectCategory.CUTLERY, ObjectCategory.TABLEWARE):
            # Designated location is INSIDE the dishwasher. Until that place primitive exists,
            # place_dishwasher_at_tab routes these to the dishwasher-tab zone (cooking_table) for a
            # normal top place so the run is testable. Flip the flag off to use the real dishwasher.
            return Location.DISHWASHER_TAB if self.place_dishwasher_at_tab else Location.DISHWASHER
        else:
            # "other" objects -> cabinet, grouped with similar items.
            return Location.CABINET

    def _get_shelf_height_for_object(self, obj: ObjectInfo) -> float:
        """Get the shelf height for an object using categorize_objects mapping."""
        obj_name = obj.name.lower()
        shelf_levels = sorted(self.shelf_level_heights.keys())

        if obj_name in self.object_to_placing_shelf and self.object_to_placing_shelf[obj_name]:
            shelf_idx = self.object_to_placing_shelf[obj_name][0]
            if shelf_idx < len(shelf_levels):
                level = shelf_levels[shelf_idx]
                height = self.shelf_level_heights.get(level, self.default_shelf_height)
                CLog.vision(
                    self,
                    "SHELF",
                    f"Categorized shelf for '{obj_name}' → index {shelf_idx}, level {level} ({height}m)",
                )
                return height

        CLog.vision(
            self,
            "SHELF",
            f"No shelf match for '{obj_name}', using default {self.default_shelf_height}m",
        )
        return self.default_shelf_height

    def _level_number(self, height: float) -> int:
        """Real shelf level number (1 = lowest) for a configured height."""
        for lvl, h in sorted(self.shelf_level_heights.items()):
            if abs(h - height) < 1e-6:
                return lvl
        return 1

    def _build_shelf_fallback_list(self, obj: ObjectInfo) -> list[float]:
        """Build the ordered list of shelf heights to try for this object.

        - First entry: the shelf picked by categorize_objects for this object.
        - Remaining entries: the other REACHABLE shelves in ascending height
          order (lowest first) because lower shelves are easier for the xArm6
          to reach from the Dashgo base.

        A primary below MIN_REACHABLE_SHELF_Z (the real level 1) is announce-only:
        the categorization already named it for the points, so say why and place on
        the lowest reachable level instead.

        Returns an empty list if no shelves are configured.
        """
        if not self.shelf_level_heights:
            return []
        primary = self._get_shelf_height_for_object(obj)
        reachable = sorted(
            h for h in self.shelf_level_heights.values() if h >= MIN_REACHABLE_SHELF_Z
        )
        if primary < MIN_REACHABLE_SHELF_Z:
            if reachable:
                self.subtask_manager.hri.say(
                    f"Shelf {self._level_number(primary)} is too low for my arm, so I "
                    f"will place the {obj.name} on shelf {self._level_number(reachable[0])}.",
                    wait=False,
                )
            return reachable
        # Keep primary at the front, drop it from the tail fallbacks
        fallbacks = [h for h in reachable if abs(h - primary) > 1e-6]
        return [primary] + fallbacks

    def announce_objects(self, names):
        """Name each object for its recognize point. Recognize scores per object
        instance, so a cleanup spoon and a separate breakfast spoon each count."""
        for name in names:
            if name and name != "unknown":
                self.subtask_manager.hri.say(f"I see a {name}.")
                self.timeout(0.3)

    def _carrying_name(self) -> str:
        """Human name of whatever is currently held (ObjectInfo or breakfast dict)."""
        c = self.carrying
        if c is None:
            return ""
        if isinstance(c, dict):
            return c.get("name", "object")
        return getattr(c, "name", "object")

    def _ensure_gripper_empty(self) -> bool:
        """Anti-drop guard. Call right before any pick, once the robot is at the pick surface.

        If the gripper still holds an object (a previous place failed), place it (controlled)
        at the CURRENT location to free the gripper — never open the gripper for a new pick
        while holding something (that drops the held object: -40 penalty). Returns True if
        the gripper is (now) empty, False if it could not be freed.
        """
        if self.carrying is None:
            return True
        if self.use_grasp_detector:
            rclpy.spin_once(self, timeout_sec=0.5)
            if not self._gripper_has_object:
                # Stale bookkeeping (e.g. a place that timed out but physically finished):
                # the gripper is really empty, so don't waste place attempts.
                CLog.manip(self, "PICK", "Grasp bit reports empty gripper; clearing carry state.")
                self.carrying = None
                return True
        name = self._carrying_name()
        CLog.manip(
            self,
            "PICK",
            f"Still holding {name}; placing it here before the next pick.",
            level="warn",
        )
        self.subtask_manager.hri.say("First I will put down the object I am holding.", wait=False)
        for _ in range(ATTEMPT_LIMIT):
            if self.subtask_manager.manipulation.place() == Status.EXECUTION_SUCCESS:
                self.carrying = None
                CLog.manip(self, "PICK", f"Freed the gripper (placed {name}).", level="success")
                return True
            self.timeout(1.0)
        CLog.manip(
            self,
            "PICK",
            f"Could not put down {name}; will not open the gripper (skipping this pick).",
            level="error",
        )
        return False

    # ------------------------------------------------------------------
    # Finite State Machine
    # ------------------------------------------------------------------

    def run(self):
        """Finite State Machine"""

        # ==================== WAIT FOR START ====================
        if self.current_state == PickAndPlaceTM.TaskStates.WAIT_FOR_BUTTON:
            CLog.fsm(self, "STATE", "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)

            CLog.fsm(self, "STATE", "Start button pressed. Waiting for door to open...")
            self.subtask_manager.hri.say("Waiting for the door to open.", wait=False)
            while True:
                status, _ = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

            CLog.fsm(
                self,
                "STATE",
                "Door open. Pick and Place Challenge begins.",
                level="success",
            )
            # The scored 420s window starts NOW (door open), not at node launch —
            # without this reset the final report includes the button-wait time.
            self.total_start_time = datetime.now()
            self.current_state = PickAndPlaceTM.TaskStates.START

        # ==================== START ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.START:
            self._track_state_change(PickAndPlaceTM.TaskStates.START)
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.navigate_to_location(Location.KITCHEN, say=False)
            self.subtask_manager.hri.say(
                "Please remove the two chairs closest to me from the dining table.",
                wait=True,
            )
            self.timeout(5.0)

            if self.use_extra_surface:
                CLog.fsm(self, "STATE", "Using extra surface (common objects).")
                self.subtask_manager.hri.say("I will clean the extra surface.", wait=False)
            else:
                self.subtask_manager.hri.say("I will clean the dining table.", wait=False)

            self.current_state = PickAndPlaceTM.TaskStates.PERCEIVE_TABLE

        # ==================== PERCEIVE TABLE ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PERCEIVE_TABLE:
            self._track_state_change(PickAndPlaceTM.TaskStates.PERCEIVE_TABLE)
            table_location = (
                Location.EXTRA_SURFACE if self.use_extra_surface else Location.DINING_TABLE
            )
            self.navigate_to_location(table_location, say=False)
            self.subtask_manager.nav.dock_table()
            self._docked_at_table = True
            self.subtask_manager.manipulation.move_to_position("table_stare")

            status, detections = self.subtask_manager.vision.detect_objects(timeout=5)

            if status == Status.EXECUTION_SUCCESS and detections:
                self.detected_objects = []
                for bbox in detections:
                    raw_name = bbox.classname if bbox.classname else "unknown"
                    # Vision may publish trash-category detections as "trash/<name>"
                    if raw_name.lower().startswith("trash/"):
                        raw_name = raw_name.split("/", 1)[1]
                    obj_name = self.yolo_to_logical.get(raw_name, raw_name)
                    category = (
                        ObjectCategory.COMMON
                        if self.use_extra_surface
                        else self.categorize_object(obj_name)
                    )
                    self.detected_objects.append(
                        ObjectInfo(
                            name=obj_name,
                            category=category,
                            bbox=bbox,
                        )
                    )
                CLog.vision(
                    self,
                    "DETECT",
                    f"Detected {len(self.detected_objects)} objects on the table.",
                )
                self.current_state = PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS
            else:
                CLog.vision(
                    self,
                    "DETECT",
                    "No objects detected, skipping to breakfast phase.",
                    level="warn",
                )
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== ANNOUNCE OBJECTS ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS:
            self._track_state_change(PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS)
            self.subtask_manager.hri.say(
                f"I have detected {len(self.detected_objects)} objects on the table."
            )

            for obj in self.detected_objects:
                CLog.vision(
                    self,
                    "DETECT",
                    f"Object: {obj.name}, Category: {obj.category.value}",
                )
            self.announce_objects([obj.name for obj in self.detected_objects])

            self.current_object_index = 0
            self.current_state = PickAndPlaceTM.TaskStates.SORT_OBJECTS

        # ==================== SORT OBJECTS BY PRIORITY ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.SORT_OBJECTS:
            self._track_state_change(PickAndPlaceTM.TaskStates.SORT_OBJECTS)

            # Priority by expected points: cutlery pick is worth 100 pts
            # (50 base + 50 bonus) vs 50 for normal objects, and the first
            # pick awards an extra 100 pts.  OTHER goes to cabinet where
            # placing next to similar objects earns +20 bonus.
            # Reliability-first: a box/can (OTHER, GPD ~90%) is a far safer first
            # pick than cutlery (~30%), and the first successful pick earns the
            # +100 bonus, so OTHER goes before CUTLERY.
            PRIORITY = {
                ObjectCategory.OTHER: 0,
                ObjectCategory.CUTLERY: 1,
                ObjectCategory.TABLEWARE: 2,
                ObjectCategory.TRASH: 3,
                ObjectCategory.COMMON: 4,
            }

            skip_names = ["red_plate", "plate", "dish"]
            before = len(self.detected_objects)
            self.detected_objects = [
                obj for obj in self.detected_objects if obj.name.lower() not in skip_names
            ]
            skipped = before - len(self.detected_objects)
            if skipped > 0:
                CLog.fsm(self, "SORT", f"Skipping {skipped} plate(s) — not graspable.")

            # Stacking: never lift a base object (e.g. a bowl) while something
            # (e.g. cutlery) still rests on it. Detect "A on B" by base_link Z and
            # XY proximity, then sort so objects with nothing on top come first,
            # ranked by reliability within that group.
            points = {id(o): self._detection_point(o.bbox) for o in self.detected_objects}

            def _has_object_on_top(obj):
                po = points.get(id(obj))
                if po is None:
                    return False
                for other in self.detected_objects:
                    if other is obj:
                        continue
                    pa = points.get(id(other))
                    if pa is None:
                        continue
                    dx, dy = pa[0] - po[0], pa[1] - po[1]
                    planar = (dx * dx + dy * dy) ** 0.5
                    if pa[2] - po[2] > STACK_Z_MIN and planar < STACK_XY_MAX:
                        CLog.fsm(
                            self,
                            "SORT",
                            f"{other.name} sits on {obj.name}; picking {other.name} first.",
                        )
                        return True
                return False

            on_top = {id(o): _has_object_on_top(o) for o in self.detected_objects}
            self.detected_objects.sort(
                key=lambda o: (1 if on_top[id(o)] else 0, PRIORITY.get(o.category, 99))
            )

            for i, obj in enumerate(self.detected_objects):
                CLog.fsm(self, "SORT", f"Priority {i}: {obj.name} ({obj.category.value})")

            self.current_object_index = 0
            self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== CLEANUP LOOP ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.CLEANUP_LOOP:
            self._track_state_change(PickAndPlaceTM.TaskStates.CLEANUP_LOOP)

            while self.current_object_index < len(self.detected_objects) and (
                self.detected_objects[self.current_object_index].is_picked
                or self.detected_objects[self.current_object_index].skipped
            ):
                self.current_object_index += 1

            placed_count = sum(1 for o in self.detected_objects if o.is_placed)

            if (
                self.current_object_index < len(self.detected_objects)
                and placed_count < self.max_cleanup_objects
            ):
                self.grasped_object = self.detected_objects[self.current_object_index]
                self.current_state = PickAndPlaceTM.TaskStates.PICK_OBJECT
            else:
                CLog.fsm(
                    self,
                    "STATE",
                    f"Cleanup done ({placed_count} placed, switching to breakfast).",
                    level="success",
                )
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== PICK OBJECT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PICK_OBJECT:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_OBJECT)

            table_location = (
                Location.EXTRA_SURFACE if self.use_extra_surface else Location.DINING_TABLE
            )
            # Dock once per table visit: skip if perceive or a prior object already
            # docked here; retries reuse it.
            if self.current_attempts == 0 and not self._docked_at_table:
                self.navigate_to_location(table_location, say=False)
                self.subtask_manager.nav.dock_table()
                self._docked_at_table = True

            if not self._ensure_gripper_empty():
                self.grasped_object.skipped = True
                self.current_object_index += 1
                self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP
                return

            self.subtask_manager.manipulation.move_to_position("table_stare")

            before_counts = None
            if self.use_vision_confirmation:
                before_counts = self._table_counts()

            self.subtask_manager.hri.say(f"I will pick the {self.grasped_object.name}.", wait=False)

            status = self.subtask_manager.manipulation.pick_object(
                self._to_yolo_name(self.grasped_object.name)
            )

            # Verify gripper actually has the object
            if status == Status.EXECUTION_SUCCESS:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.use_grasp_detector and not self._gripper_has_object:
                    CLog.manip(
                        self,
                        "PICK",
                        f"Gripper reports no object after picking {self.grasped_object.name}.",
                        level="warn",
                    )
                    self.subtask_manager.hri.say("I did not grasp the object.", wait=False)
                    status = Status.EXECUTION_ERROR

            if status == Status.EXECUTION_SUCCESS and self.use_vision_confirmation:
                status = self._confirm_pick_by_vision(before_counts)

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_picked = True
                self.carrying = self.grasped_object  # gripper now physically holds it
                if self.first_pick:
                    CLog.manip(self, "PICK", "FIRST PICK BONUS achieved!", level="success")
                    self.first_pick = False
                self.current_attempts = 0
                self.current_state = PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT
            else:
                self.current_attempts += 1
                CLog.manip(
                    self,
                    "PICK",
                    f"Failed to pick {self.grasped_object.name} — attempt {self.current_attempts}/{ATTEMPT_LIMIT}",
                    level="error",
                )
                if self.current_attempts >= ATTEMPT_LIMIT:
                    CLog.manip(
                        self,
                        "PICK",
                        f"Skipping {self.grasped_object.name} after {ATTEMPT_LIMIT} attempts.",
                        level="warn",
                    )
                    self.current_attempts = 0
                    self.grasped_object.skipped = True
                    self.current_object_index += 1
                    self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== DETERMINE PLACEMENT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT)
            self.grasped_object.placement_location = self.determine_placement_location(
                self.grasped_object
            )
            CLog.fsm(
                self,
                "SORT",
                f"{self.grasped_object.name} → {self.grasped_object.placement_location.value}",
            )

            if (
                self.grasped_object.placement_location == Location.DISHWASHER
                and self.use_dishwasher
                and not self.dishwasher_open
            ):
                # Only run the open-the-dishwasher flow when explicitly enabled; otherwise
                # place on top of the dishwasher (default).
                self.current_state = PickAndPlaceTM.TaskStates.CHECK_DISHWASHER
            else:
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        # ==================== CHECK DISHWASHER ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.CHECK_DISHWASHER:
            self._track_state_change(PickAndPlaceTM.TaskStates.CHECK_DISHWASHER)
            self.navigate_to_location(Location.DISHWASHER, say=False)
            status, answer = self.subtask_manager.vision.moondream_query(
                "Is the dishwasher door open? Reply only with yes or no."
            )
            if status == Status.EXECUTION_SUCCESS and "yes" in answer.lower():
                self.dishwasher_open = True
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT
            else:
                self.current_state = PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP

        # ==================== REQUEST DISHWASHER HELP ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP:
            self._track_state_change(PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP)
            self.subtask_manager.hri.say(
                "Could you please open the dishwasher door for me?",
                wait=True,
            )
            _, answer = self.subtask_manager.hri.confirm(
                "Please say yes once the dishwasher door is open.",
                use_keyword=True,
                retries=5,
                wait_between_retries=10.0,
            )
            if answer == "yes":
                CLog.hri(
                    self,
                    "CONFIRM",
                    "Referee confirmed dishwasher is open.",
                    level="success",
                )
            else:
                CLog.hri(
                    self,
                    "CONFIRM",
                    "No confirmation, assuming dishwasher is open.",
                    level="warn",
                )
            self.dishwasher_open = True
            self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        # ==================== NAVIGATE TO PLACEMENT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT)
            self._docked_at_table = False  # left the table to place
            placement_loc = self.grasped_object.placement_location
            result = self.navigate_to_location(placement_loc)
            # dishwasher / cooking_table -> short approach (they bump if docked flush);
            # extra surface -> default dock; cabinet -> stand off ~30 cm.
            # Trash bin uses its own detect-and-drop flow.
            if placement_loc in (Location.DISHWASHER, Location.DISHWASHER_TAB):
                self.subtask_manager.nav.dock_table(offset=DISHWASHER_DOCK_OFFSET)
            elif placement_loc == Location.EXTRA_SURFACE:
                self.subtask_manager.nav.dock_table()
            elif placement_loc == Location.CABINET:
                self.subtask_manager.nav.dock_table(offset=0.30)

            if result == Status.EXECUTION_SUCCESS:
                if placement_loc == Location.CABINET and not self.shelf_scanned:
                    self.current_state = PickAndPlaceTM.TaskStates.SCAN_CABINET_SHELVES
                else:
                    self.current_state = PickAndPlaceTM.TaskStates.PLACE_OBJECT
            else:
                CLog.nav(
                    self,
                    "FAIL",
                    f"Navigation to {placement_loc.value} failed, skipping object.",
                    level="error",
                )
                self.grasped_object.is_picked = True
                self.current_object_index += 1
                self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== SCAN CABINET SHELVES ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.SCAN_CABINET_SHELVES:
            self._track_state_change(PickAndPlaceTM.TaskStates.SCAN_CABINET_SHELVES)
            self.subtask_manager.hri.say("Scanning cabinet shelves.", wait=False)

            # Scan each shelf level (move arm, build octomap, detect + filter by height)
            shelf_levels = sorted(self.shelf_level_heights.keys())
            self.shelves = {i: [] for i in range(len(shelf_levels))}

            for idx, level in enumerate(shelf_levels):
                height = self.shelf_level_heights[level]
                CLog.vision(self, "SHELF", f"Scanning shelf level {level} at {height}m")
                self.subtask_manager.hri.say(f"Scanning shelf number {idx + 1}.", wait=False)

                # An unreachable level (real level 1) is viewed from the lowest reachable
                # level's pose — the camera can frame it even though the arm cannot operate
                # there; the height filter below still attributes detections to the real level.
                self._scan_shelf_level(
                    height if height >= MIN_REACHABLE_SHELF_Z else self.default_shelf_height
                )

                status, detections = self.subtask_manager.vision.detect_objects()
                retry = 0
                while status != Status.EXECUTION_SUCCESS and retry < 3:
                    CLog.vision(
                        self,
                        "SHELF",
                        f"Retry {retry + 1} detecting at shelf {level}",
                        level="warn",
                    )
                    self.timeout(1.0)
                    status, detections = self.subtask_manager.vision.detect_objects()
                    retry += 1

                if status == Status.EXECUTION_SUCCESS and detections:
                    filtered = self._filter_detections_by_height(detections, height)
                    for det in filtered:
                        obj_name = det.classname if det.classname else "unknown"
                        self.shelves[idx].append(obj_name)
                    CLog.vision(self, "SHELF", f"Shelf {level}: {self.shelves[idx]}")
                else:
                    CLog.vision(
                        self,
                        "SHELF",
                        f"No objects detected at shelf level {level}.",
                        level="warn",
                    )

            self.shelf_scanned = True
            # All 3 levels just got scanned, so the octomap is fresh enough
            # that the upcoming place does not need its own per-level scan.
            self._cabinet_scan_fresh = True
            CLog.vision(self, "SHELF", f"All shelves scanned: {self.shelves}")

            # Categorize table objects against shelf contents
            # Only the shelf-bound "other" objects need shelf categorization;
            # cutlery/tableware go to the dishwasher, trash to the bin.
            object_names_on_table = [
                obj.name for obj in self.detected_objects if obj.category == ObjectCategory.OTHER
            ]
            CLog.vision(
                self,
                "CATEGORIZE",
                f"Categorizing {object_names_on_table} with shelves {self.shelves}",
            )

            try:
                cat_status, categorized_shelfs, objects_to_add, _ = (
                    self.subtask_manager.hri.categorize_objects(object_names_on_table, self.shelves)
                )
            except Exception as e:
                CLog.hri(self, "CATEGORIZE", f"Error categorizing: {e}", level="error")
                cat_status = Status.EXECUTION_ERROR

            if cat_status == Status.EXECUTION_SUCCESS:
                self.object_to_placing_shelf = defaultdict(list)
                for shelf_idx in objects_to_add:
                    for obj_name in objects_to_add[shelf_idx]:
                        self.object_to_placing_shelf[obj_name].append(shelf_idx)

                CLog.vision(self, "CATEGORIZE", f"Shelf categories: {categorized_shelfs}")
                CLog.vision(
                    self,
                    "CATEGORIZE",
                    f"Object placement: {dict(self.object_to_placing_shelf)}",
                )

                for obj_name, shelf_idxs in self.object_to_placing_shelf.items():
                    if shelf_idxs:
                        shelf_idx = shelf_idxs[0]
                        level = shelf_levels[shelf_idx] if shelf_idx < len(shelf_levels) else 1
                        # Announce the SHELF's category (where it goes), not the
                        # object's own category.
                        shelf_cats = categorized_shelfs.get(shelf_idx, [])
                        cat = " and ".join(shelf_cats)
                        where = f"shelf {level}" + (f", the {cat} shelf" if cat else "")
                        self.subtask_manager.hri.say(
                            f"The {obj_name} goes on {where}.",
                            wait=False,
                        )
                        self.timeout(0.5)
            else:
                CLog.hri(
                    self,
                    "CATEGORIZE",
                    "Categorization failed, will use default shelf heights.",
                    level="warn",
                )

            self.current_state = PickAndPlaceTM.TaskStates.PLACE_OBJECT

        # ==================== PLACE OBJECT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PLACE_OBJECT:
            self._track_state_change(PickAndPlaceTM.TaskStates.PLACE_OBJECT)
            placement_loc = self.grasped_object.placement_location
            self.subtask_manager.hri.say(
                f"Placing the {self.grasped_object.name} at the {placement_loc.value}.",
                wait=False,
            )

            if placement_loc == Location.DISHWASHER:
                # TODO(dishwasher-inside): cutlery/plate/cup must be placed INSIDE the dishwasher
                # (open the door — ask for help, 0 pts/0 penalty — then lower into the rack), NOT on
                # top. place() below is a generic top-place placeholder until the inside-place
                # primitive exists. The open flow is gated by use_dishwasher (DETERMINE_PLACEMENT).
                status = self.subtask_manager.manipulation.place()
            elif placement_loc == Location.TRASH_BIN:
                status = self.subtask_manager.manipulation.place(is_trash=True)
            elif placement_loc == Location.CABINET:
                # Build the ordered fallback list the first time we enter
                # PLACE_OBJECT for this grasped object. The list has the
                # categorized shelf first, then the other shelves from
                # lowest to highest (most reachable first).
                if not self._shelf_fallback_heights:
                    self._shelf_fallback_heights = self._build_shelf_fallback_list(
                        self.grasped_object
                    )
                    self._shelf_fallback_idx = 0
                    CLog.manip(
                        self,
                        "PLACE",
                        f"Shelf attempt order for {self.grasped_object.name}: "
                        f"{self._shelf_fallback_heights}",
                    )

                shelf_height = self._shelf_fallback_heights[self._shelf_fallback_idx]
                is_fallback = self._shelf_fallback_idx > 0
                CLog.manip(
                    self,
                    "PLACE",
                    f"Placing {self.grasped_object.name} at shelf height {shelf_height}m"
                    + (" (fallback shelf)." if is_fallback else "."),
                )
                # Only scan the shelf once per cabinet visit. If the octomap
                # is already fresh (from SCAN_CABINET_SHELVES on first visit,
                # or from the first attempt on subsequent visits) we skip
                # the scan entirely — retries and shelf fallbacks reuse the
                # existing octomap.
                if not self._cabinet_scan_fresh:
                    CLog.manip(
                        self,
                        "PLACE",
                        f"Scanning shelf at {shelf_height}m (first place attempt this visit).",
                    )
                    self._scan_shelf_level(shelf_height)
                    self._cabinet_scan_fresh = True
                else:
                    CLog.manip(
                        self,
                        "PLACE",
                        "Reusing octomap from current visit (skipping shelf scan).",
                    )
                # Move the arm to a known good starting pose before asking
                # place_on_shelf to plan a trajectory. After the shelf scan
                # the arm is left at the last observation pose (usually
                # looking up at shelf 3) — from that configuration MoveIt
                # fails to find valid IK for the horizontal pre-place pose
                # (AIM_STRAIGHT_FRONT_QUAT), which surfaces as
                # "Unable to sample any valid states for goal tree". Jump
                # back to front_stare so planning starts from a neutral
                # extended-forward configuration that is close to the
                # place target's orientation.
                CLog.manip(self, "PLACE", "Moving arm to front_stare before place planning.")
                self.subtask_manager.manipulation.move_to_position("front_stare")
                opt_status = self.subtask_manager.manipulation.get_optimal_position_for_plane(
                    shelf_height,
                    tolerance=0.1,
                    table_or_shelf=False,
                    approach_plane=False,
                )
                CLog.manip(self, "PLACE", f"get_optimal_position_for_plane → {opt_status}")
                self.subtask_manager.hri.say(
                    f"Placing {self.grasped_object.name} on the shelf.", wait=False
                )
                status = self.subtask_manager.manipulation.place_on_shelf(
                    plane_height=shelf_height,
                    tolerance=0.1,
                )
                CLog.manip(self, "PLACE", f"place_on_shelf → {status}")
            else:
                self.subtask_manager.hri.say(f"Placing {self.grasped_object.name}.", wait=False)
                status = self.subtask_manager.manipulation.place()

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_placed = True
                self.carrying = None  # gripper released the object
                CLog.manip(
                    self,
                    "PLACE",
                    f"Placed {self.grasped_object.name} at {placement_loc.value}.",
                    level="success",
                )
                self.current_attempts = 0
                self._shelf_fallback_heights = []
                self._shelf_fallback_idx = 0
                self.current_object_index += 1
                self.grasped_object = None
                self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP
            else:
                self.current_attempts += 1
                CLog.manip(
                    self,
                    "PLACE",
                    f"Failed to place {self.grasped_object.name} — attempt {self.current_attempts}/{ATTEMPT_LIMIT}",
                    level="error",
                )
                if self.current_attempts < ATTEMPT_LIMIT:
                    # Retry the same placement on the same shelf level.
                    CLog.manip(self, "PLACE", "Retrying place on the same shelf level...")
                    self.timeout(1.0)
                    # Stay in PLACE_OBJECT to retry
                elif placement_loc == Location.CABINET and self._shelf_fallback_idx + 1 < len(
                    self._shelf_fallback_heights
                ):
                    # Exhausted retries on this shelf but more shelves are
                    # available — fall through to the next shelf level.
                    self._shelf_fallback_idx += 1
                    next_height = self._shelf_fallback_heights[self._shelf_fallback_idx]
                    CLog.manip(
                        self,
                        "PLACE",
                        f"All {ATTEMPT_LIMIT} attempts on {shelf_height}m failed; "
                        f"falling back to shelf at {next_height}m.",
                        level="warn",
                    )
                    self.current_attempts = 0
                    self.timeout(1.0)
                    # Stay in PLACE_OBJECT to retry on the new shelf
                else:
                    CLog.manip(
                        self,
                        "PLACE",
                        f"Giving up on {self.grasped_object.name} after exhausting "
                        f"{len(self._shelf_fallback_heights) or 1} shelf level(s) "
                        f"with {ATTEMPT_LIMIT} attempts each.",
                        level="warn",
                    )
                    self.current_attempts = 0
                    self._shelf_fallback_heights = []
                    self._shelf_fallback_idx = 0
                    self.current_object_index += 1
                    self.grasped_object = None
                    self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== START BREAKFAST PREP ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP:
            self._track_state_change(PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP)
            self.subtask_manager.hri.say("Now I will prepare breakfast.", wait=False)
            self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== GET BREAKFAST ITEMS ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS:
            self._track_state_change(PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS)
            self.current_breakfast_item = None
            for item in self.breakfast_items:
                if not item["picked"]:
                    self.current_breakfast_item = item
                    break

            if self.current_breakfast_item:
                CLog.fsm(
                    self,
                    "STATE",
                    f"Next breakfast item: {self.current_breakfast_item['name']} from {self.current_breakfast_item['location'].value}",
                )
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE
            else:
                CLog.fsm(self, "STATE", "All breakfast items collected.", level="success")
                self.current_state = PickAndPlaceTM.TaskStates.END

        # ==================== NAVIGATE TO ITEM SOURCE ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE)
            item_location = self.current_breakfast_item["location"]
            result = self.navigate_to_location(item_location)
            # Cabinet -> stand off ~30 cm; dishwasher top (bowl+spoon source) -> short approach so
            # the base does not bump it; other table-height surfaces -> default dock.
            if item_location == Location.CABINET:
                self.subtask_manager.nav.dock_table(offset=0.30)
            elif item_location == Location.BREAKFAST_ITEMS:
                self.subtask_manager.nav.dock_table(offset=DISHWASHER_DOCK_OFFSET)
            else:
                self.subtask_manager.nav.dock_table()

            if result == Status.EXECUTION_SUCCESS:
                self.current_state = PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM
            else:
                CLog.nav(
                    self,
                    "FAIL",
                    f"Failed to reach {item_location.value}, skipping {self.current_breakfast_item['name']}.",
                    level="error",
                )
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== PICK BREAKFAST ITEM ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            item_location = self.current_breakfast_item["location"]

            # Anti-drop: never open the gripper for this pick while still holding a prior item.
            if not self._ensure_gripper_empty():
                self.current_breakfast_item["picked"] = True  # skip; could not free the gripper
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS
                return

            is_cabinet = item_location == Location.CABINET
            yolo_name = self._to_yolo_name(item_name)
            if is_cabinet:
                # Per-level detect-then-pick so a high-shelf object (e.g. cereal on L3)
                # is framed and picked; table_stare frames only the lower levels.
                # say_name makes it announce the pick intent after the detection.
                status = self._pick_from_shelf(
                    yolo_name, self.shelf_level_heights, say_name=item_name
                )
                self._cabinet_scan_fresh = True  # the per-level scan refreshed the octomap
            else:
                self.subtask_manager.manipulation.move_to_position("table_stare")
                # Name what is at the breakfast surface, then announce the pick intent.
                _, bf_dets = self.subtask_manager.vision.detect_objects()
                self.announce_objects([d.classname for d in (bf_dets or [])])
                self.subtask_manager.hri.say(f"I will pick the {item_name}.", wait=False)
                status = self.subtask_manager.manipulation.pick_object(
                    yolo_name, scan_environment=False
                )

            # Verify gripper actually has the object
            if status == Status.EXECUTION_SUCCESS:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.use_grasp_detector and not self._gripper_has_object:
                    CLog.manip(
                        self,
                        "PICK",
                        f"Gripper reports no object after picking {item_name}.",
                        level="warn",
                    )
                    self.subtask_manager.hri.say("I did not grasp the object.", wait=False)
                    status = Status.EXECUTION_ERROR

            if status == Status.EXECUTION_SUCCESS:
                self.current_attempts = 0
                self.current_breakfast_item["picked"] = True
                self.carrying = self.current_breakfast_item  # gripper now physically holds it
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING
            else:
                self.current_attempts += 1
                CLog.manip(
                    self,
                    "PICK",
                    f"Failed to pick breakfast item {item_name} — "
                    f"attempt {self.current_attempts}/{ATTEMPT_LIMIT}",
                    level="error",
                )
                if self.current_attempts >= ATTEMPT_LIMIT:
                    CLog.manip(
                        self,
                        "PICK",
                        f"Skipping {item_name} after {ATTEMPT_LIMIT} attempts.",
                        level="warn",
                    )
                    self.current_attempts = 0
                    self.current_breakfast_item["picked"] = True  # give up; continue with next item
                    self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS
                # else: stay in PICK_BREAKFAST_ITEM to retry (gripper empty, nothing dropped)

        # ==================== NAVIGATE TO DINING ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING)
            result = self.navigate_to_location(Location.DINING_TABLE)
            self.subtask_manager.nav.dock_table()

            if result == Status.EXECUTION_SUCCESS:
                item_name = self.current_breakfast_item["name"]
                if item_name in ("cereal", "milk") and self.bowl_placed:
                    self.current_state = PickAndPlaceTM.TaskStates.POUR_INTO_BOWL
                else:
                    self.current_state = PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM
            else:
                CLog.nav(self, "FAIL", "Failed to navigate to dining table.", level="error")
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== POUR INTO BOWL ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.POUR_INTO_BOWL:
            self._track_state_change(PickAndPlaceTM.TaskStates.POUR_INTO_BOWL)
            item_name = self.current_breakfast_item["name"]
            CLog.manip(self, "POUR", f"Pouring {item_name} into the bowl.")
            self.subtask_manager.hri.say(f"Pouring {item_name} into the bowl.", wait=False)

            pour_attempts = 2
            for attempt in range(1, pour_attempts + 1):
                status = self.subtask_manager.manipulation.pour(
                    pour_object_name=self._to_yolo_name(item_name),
                    pour_container_name=self._to_yolo_name("bowl"),
                    object_already_grasped=True,
                )
                if status == Status.EXECUTION_SUCCESS:
                    CLog.manip(self, "POUR", f"Poured {item_name} into bowl.", level="success")
                    break
                elif attempt < pour_attempts:
                    CLog.manip(
                        self,
                        "POUR",
                        f"Pour attempt {attempt} failed, retrying...",
                        level="warn",
                    )
                else:
                    CLog.manip(
                        self,
                        "POUR",
                        f"Pour failed for {item_name} after {pour_attempts} attempts, placing without pouring.",
                        level="warn",
                    )

            self.current_state = PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM

        # ==================== PLACE BREAKFAST ITEM ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            self.subtask_manager.hri.say(f"Placing the {item_name}.", wait=False)

            close_to_logical = self.current_breakfast_item.get("close_to", "")
            # Only anchor to the reference if it was actually placed; the close_to
            # chain breaks (empty point, place fails) when an earlier item failed.
            ref_placed = any(
                it["name"] == close_to_logical and it["placed"] for it in self.breakfast_items
            )
            close_to = (
                self._to_yolo_name(close_to_logical) if close_to_logical and ref_placed else ""
            )
            status = self.subtask_manager.manipulation.place(close_to=close_to)

            if status == Status.EXECUTION_SUCCESS:
                self.current_breakfast_item["placed"] = True
                self.carrying = None  # gripper released the object
                if item_name == "bowl":
                    self.bowl_placed = True
                CLog.manip(
                    self,
                    "PLACE",
                    f"Placed breakfast item: {item_name}.",
                    level="success",
                )
            else:
                CLog.manip(
                    self,
                    "PLACE",
                    f"Failed to place breakfast item: {item_name}.",
                    level="error",
                )

            self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== END ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.END:
            CLog.fsm(self, "STATE", "Task completed.", level="success")
            self._track_state_change(PickAndPlaceTM.TaskStates.END)

            total_task_time = (datetime.now() - self.total_start_time).total_seconds()
            CLog.fsm(self, "TIMER", "=== FINAL TIMING REPORT ===")
            CLog.fsm(self, "TIMER", f"Total task time: {total_task_time:.2f}s")

            sorted_states = sorted(self.state_times.items(), key=lambda x: x[1], reverse=True)
            for state, time_spent in sorted_states:
                percentage = (time_spent / total_task_time) * 100 if total_task_time > 0 else 0
                CLog.fsm(self, "TIMER", f"{state}: {time_spent:.2f}s ({percentage:.1f}%)")

            CLog.fsm(self, "TIMER", "=== END TIMING REPORT ===")
            self.subtask_manager.hri.say("I have completed the pick and place task.")
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.running_task = False

        # ==================== DEBUG ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.DEBUG:
            CLog.fsm(self, "STATE", "Debugging task.")
            self._track_state_change(PickAndPlaceTM.TaskStates.DEBUG)
            self.subtask_manager.hri.say("Debugging task.")
            self.current_state = PickAndPlaceTM.TaskStates.END


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTM()
    import os

    if os.environ.get("SHELF_TEST"):
        tgt = node._to_yolo_name(os.environ.get("SHELF_TARGET", "cereal"))
        CLog.fsm(node, "SHELF_TEST", f"target={tgt} heights={node.shelf_level_heights}")
        st = node._pick_from_shelf(tgt, node.shelf_level_heights)
        CLog.fsm(node, "SHELF_TEST", f"RESULT _pick_from_shelf -> {st}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        CLog.fsm(node, "STATE", "Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

"""
Task Manager for Pick and Place Challenge - RoboCup @Home 2026

Strategy: maximize pick+place score in 7 minutes.
- Cleanup phase: up to max_cleanup_objects from dining table, sorted by pick reliability.
- Breakfast phase: bowl, cereal, milk, spoon placed on dining table with pour for cereal/milk.
- Plates are skipped (GPD cannot grasp flat objects).
- Dishwasher support controlled by use_dishwasher flag.
- Cabinet shelf placement uses moondream perception to match categories.
"""

import json
import time
from datetime import datetime
from enum import Enum
from pathlib import Path

import rclpy
from rclpy.node import Node
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task
from frida_constants.vision_classes import BBOX

ATTEMPT_LIMIT = 3

SHELF_LEVEL_NAMES = {1: "bottom", 2: "middle", 3: "top"}


class ObjectCategory(Enum):
    """Object categories for placement decisions"""

    CUTLERY = "cutlery"
    TABLEWARE = "tableware"
    TRASH = "trash"
    OTHER = "other"
    COMMON = "common"


class Location(Enum):
    """Known furniture locations in the arena"""

    DINING_TABLE = "dining_table"
    SIDE_TABLE = "side_table"
    DISHWASHER = "dishwasher"
    CABINET = "cabinet"
    TRASH_BIN = "trash_bin"
    BREAKFAST_SURFACE = "breakfast_surface"
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
        SORT_OBJECTS = "SORT_OBJECTS"
        CLEANUP_LOOP = "CLEANUP_LOOP"
        PICK_OBJECT = "PICK_OBJECT"
        DETERMINE_PLACEMENT = "DETERMINE_PLACEMENT"
        CHECK_DISHWASHER = "CHECK_DISHWASHER"
        REQUEST_DISHWASHER_HELP = "REQUEST_DISHWASHER_HELP"
        NAVIGATE_TO_PLACEMENT = "NAVIGATE_TO_PLACEMENT"
        PERCEIVE_CABINET_SHELVES = "PERCEIVE_CABINET_SHELVES"
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
        self.subtask_manager = SubtaskManager(self, task=Task.PICK_AND_PLACE, mock_areas=["navigation"])

        # ACTION REQUIRED: Adjust before competition
        self.use_side_table = False  # True = use side table objects (penalty per object)
        self.trash_category = "napkin"  # Announced during Setup Days
        self.use_dishwasher = False  # True = cutlery/tableware go to dishwasher

        # YOLO class name mapping: logical name -> YOLO detection class name.
        # Only names that differ need to be listed.
        self.yolo_names = {
            "bowl": "spam_tuna_can",
            "cereal": "blue_cereal_box",
            "milk": "chocomilk_box",
        }
        self.yolo_to_logical = {v: k for k, v in self.yolo_names.items()}

        # Shelf height mapping: shelf level -> height in metres. Adjust to arena.
        self.shelf_level_heights = {1: 0.30, 2: 0.65, 3: 1.00}
        self.default_shelf_height = 1.0

        # Load object->category mapping for shelf matching (e.g. "apple" -> "fruit")
        try:
            from ament_index_python.packages import get_package_share_directory
            objects_path = (
                Path(get_package_share_directory("frida_constants")) / "data" / "objects.json"
            )
            with open(objects_path) as f:
                self._object_to_category = json.load(f).get("object_to_category", {})
        except Exception:
            self._object_to_category = {}

        # Navigation mapping: Location -> (room, sublocation).
        self.nav_locations = {
            Location.KITCHEN: ("kitchen", ""),
            Location.DINING_TABLE: ("kitchen", "dining_table"),
            Location.SIDE_TABLE: ("kitchen", "side_table"),
            Location.DISHWASHER: ("kitchen", "dishwasher"),
            Location.CABINET: ("kitchen", "cabinet"),
            Location.TRASH_BIN: ("kitchen", "trash_bin"),
            Location.BREAKFAST_SURFACE: ("kitchen", "breakfast_surface"),
        }

        # Object tracking
        self.detected_objects: list[ObjectInfo] = []
        self.current_object_index = 0
        self.grasped_object: ObjectInfo = None
        self.first_pick = True

        # Maximum cleanup objects before switching to breakfast (time budget)
        self.max_cleanup_objects = 3

        # Breakfast items — bowl first, then cereal+milk from cabinet, spoon last.
        self.breakfast_items = [
            {
                "name": "bowl",
                "location": Location.BREAKFAST_SURFACE,
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
                "location": Location.BREAKFAST_SURFACE,
                "picked": False,
                "placed": False,
                "close_to": "bowl",
            },
        ]
        self.current_breakfast_item: dict = None
        self.bowl_placed = False

        # Dishwasher state (only used when use_dishwasher=True)
        self.dishwasher_open = False

        # Cabinet shelf categories — populated on first cabinet visit via moondream
        self.shelf_categories: dict = {}

        # Attempt counter — reset on state change
        self.current_attempts = 0
        self.running_task = True

        # State timing
        self.state_start_time = None
        self.state_times: dict = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        self.current_state = PickAndPlaceTM.TaskStates.WAIT_FOR_BUTTON
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        CLog.fsm(self, "STATE", "PickAndPlaceTaskManager has started.")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

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

        CLog.fsm(self, "STATE", f"{self.previous_state} → {self.current_state}" if self.previous_state != self.current_state else self.current_state)

    def navigate_to_location(self, location: Location, say: bool = True):
        """Navigate to a PPC location using the nav_locations mapping"""
        room, sublocation = self.nav_locations[location]
        return self.navigate_to(room, sublocation, say=say)

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to a location with retry logic"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        self.subtask_manager.nav.resume_nav()

        if say:
            CLog.nav(self, "MOVE", f"Moving to {location}")
            self.subtask_manager.hri.say(f"Moving to {location}.", wait=False)

        result = Status.EXECUTION_ERROR
        retry = 0
        while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
            future = self.subtask_manager.nav.move_to_location(location, sublocation)
            if "navigation" not in self.subtask_manager.get_mocked_areas():
                if not hasattr(future, "add_done_callback"):
                    CLog.nav(self, "FAIL", "Navigation action server not available.", level="error")
                    retry += 1
                    continue
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
            else:
                result = Status.EXECUTION_SUCCESS
            retry += 1

        self.subtask_manager.nav.pause_nav()
        return result

    def timeout(self, duration: float = 2.0):
        """Non-blocking wait that keeps spinning ROS"""
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def categorize_object(self, obj_name: str) -> ObjectCategory:
        """Assign an object to a category based on its name"""
        cutlery = ["fork", "knife", "spoon"]
        tableware = ["plate", "cup", "mug", "bowl"]

        name = obj_name.lower()
        if name in cutlery:
            return ObjectCategory.CUTLERY
        elif name in tableware:
            return ObjectCategory.TABLEWARE
        elif name == self.trash_category.lower():
            return ObjectCategory.TRASH
        else:
            return ObjectCategory.OTHER

    def _to_yolo_name(self, logical_name: str) -> str:
        """Translate a logical object name to the YOLO class name."""
        return self.yolo_names.get(logical_name.lower(), logical_name)

    def determine_placement_location(self, obj: ObjectInfo) -> Location:
        """Return the target furniture for an object."""
        if (
            self.use_dishwasher
            and obj.category in (ObjectCategory.CUTLERY, ObjectCategory.TABLEWARE)
        ):
            return Location.DISHWASHER
        elif obj.category == ObjectCategory.TRASH:
            return Location.TRASH_BIN
        else:
            return Location.CABINET

    def _find_shelf_height_for_object(self, obj: ObjectInfo) -> float:
        """Find the best shelf height by matching object to moondream descriptions."""
        if not self.shelf_categories:
            return self.default_shelf_height

        obj_name = obj.name.lower()
        obj_semantic_cat = self._object_to_category.get(obj_name, "")

        for level, description in self.shelf_categories.items():
            desc_lower = description.lower()
            if obj_name in desc_lower:
                height = self.shelf_level_heights.get(level, self.default_shelf_height)
                CLog.vision(self, "SHELF", f"Shelf match (name): '{obj_name}' → level {level} ({height}m)")
                return height
            if obj_semantic_cat and obj_semantic_cat in desc_lower:
                height = self.shelf_level_heights.get(level, self.default_shelf_height)
                CLog.vision(self, "SHELF", f"Shelf match (category '{obj_semantic_cat}'): '{obj_name}' → level {level} ({height}m)")
                return height

        CLog.vision(self, "SHELF", f"No shelf match for '{obj_name}', using default {self.default_shelf_height}m")
        return self.default_shelf_height

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

            CLog.fsm(self, "STATE", "Start button pressed. Pick and Place Challenge begins.", level="success")
            self.current_state = PickAndPlaceTM.TaskStates.START

        # ==================== START ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.START:
            self._track_state_change(PickAndPlaceTM.TaskStates.START)
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.navigate_to_location(Location.KITCHEN, say=False)

            if self.use_side_table:
                CLog.fsm(self, "STATE", "Using side table (common objects).")
                self.subtask_manager.hri.say("I will clean the side table.", wait=False)
            else:
                self.subtask_manager.hri.say("I will clean the dining table.", wait=False)

            self.current_state = PickAndPlaceTM.TaskStates.PERCEIVE_TABLE

        # ==================== PERCEIVE TABLE ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PERCEIVE_TABLE:
            self._track_state_change(PickAndPlaceTM.TaskStates.PERCEIVE_TABLE)
            table_location = Location.SIDE_TABLE if self.use_side_table else Location.DINING_TABLE
            self.navigate_to_location(table_location, say=False)
            self.subtask_manager.manipulation.move_to_position("table_stare")

            status, detections = self.subtask_manager.vision.detect_objects(timeout=5)

            if status == Status.EXECUTION_SUCCESS and detections:
                self.detected_objects = []
                for bbox in detections:
                    raw_name = bbox.classname if bbox.classname else "unknown"
                    obj_name = self.yolo_to_logical.get(raw_name, raw_name)
                    category = (
                        ObjectCategory.COMMON
                        if self.use_side_table
                        else self.categorize_object(obj_name)
                    )
                    self.detected_objects.append(
                        ObjectInfo(
                            name=obj_name,
                            category=category,
                            bbox=bbox,
                        )
                    )
                CLog.vision(self, "DETECT", f"Detected {len(self.detected_objects)} objects on the table.")
                self.current_state = PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS
            else:
                CLog.vision(self, "DETECT", "No objects detected, skipping to breakfast phase.", level="warn")
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== ANNOUNCE OBJECTS ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS:
            self._track_state_change(PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS)
            self.subtask_manager.hri.say(
                f"I have detected {len(self.detected_objects)} objects on the table."
            )

            for obj in self.detected_objects:
                CLog.vision(self, "DETECT", f"Object: {obj.name}, Category: {obj.category.value}")
                self.subtask_manager.hri.say(f"I see a {obj.name}.")
                self.timeout(0.5)

            self.current_object_index = 0
            self.current_state = PickAndPlaceTM.TaskStates.SORT_OBJECTS

        # ==================== SORT OBJECTS BY PRIORITY ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.SORT_OBJECTS:
            self._track_state_change(PickAndPlaceTM.TaskStates.SORT_OBJECTS)

            PRIORITY = {
                ObjectCategory.TRASH: 0,
                ObjectCategory.TABLEWARE: 1,
                ObjectCategory.OTHER: 2,
                ObjectCategory.COMMON: 3,
                ObjectCategory.CUTLERY: 4,
            }

            skip_names = ["plate", "dish"]
            before = len(self.detected_objects)
            self.detected_objects = [
                obj for obj in self.detected_objects
                if obj.name.lower() not in skip_names
            ]
            skipped = before - len(self.detected_objects)
            if skipped > 0:
                CLog.fsm(self, "SORT", f"Skipping {skipped} plate(s) — not graspable.")

            self.detected_objects.sort(key=lambda o: PRIORITY.get(o.category, 99))

            for i, obj in enumerate(self.detected_objects):
                CLog.fsm(self, "SORT", f"Priority {i}: {obj.name} ({obj.category.value})")

            self.current_object_index = 0
            self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== CLEANUP LOOP ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.CLEANUP_LOOP:
            self._track_state_change(PickAndPlaceTM.TaskStates.CLEANUP_LOOP)

            while (
                self.current_object_index < len(self.detected_objects)
                and self.detected_objects[self.current_object_index].is_picked
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
                CLog.fsm(self, "STATE", f"Cleanup done ({placed_count} placed, switching to breakfast).", level="success")
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== PICK OBJECT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PICK_OBJECT:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_OBJECT)

            table_location = Location.SIDE_TABLE if self.use_side_table else Location.DINING_TABLE
            self.navigate_to_location(table_location, say=False)
            self.subtask_manager.manipulation.move_to_position("table_stare")

            self.subtask_manager.hri.say(f"I will pick the {self.grasped_object.name}.", wait=False)

            if self.grasped_object.category == ObjectCategory.CUTLERY:
                status = self.subtask_manager.manipulation.pick_cutlery(
                    self._to_yolo_name(self.grasped_object.name)
                )
            else:
                status = self.subtask_manager.manipulation.pick_object(
                    self._to_yolo_name(self.grasped_object.name)
                )

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_picked = True
                if self.first_pick:
                    CLog.manip(self, "PICK", "FIRST PICK BONUS achieved!", level="success")
                    self.first_pick = False
                self.current_attempts = 0
                self.current_state = PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT
            else:
                self.current_attempts += 1
                CLog.manip(self, "PICK", f"Failed to pick {self.grasped_object.name} — attempt {self.current_attempts}/{ATTEMPT_LIMIT}", level="error")
                if self.current_attempts >= ATTEMPT_LIMIT:
                    CLog.manip(self, "PICK", f"Skipping {self.grasped_object.name} after {ATTEMPT_LIMIT} attempts.", level="warn")
                    self.current_attempts = 0
                    self.grasped_object.is_picked = True
                    self.current_object_index += 1
                    self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== DETERMINE PLACEMENT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT)
            self.grasped_object.placement_location = self.determine_placement_location(
                self.grasped_object
            )
            CLog.fsm(self, "SORT", f"{self.grasped_object.name} → {self.grasped_object.placement_location.value}")

            if (
                self.grasped_object.placement_location == Location.DISHWASHER
                and not self.dishwasher_open
            ):
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
                "Could you please open the dishwasher door for me?", wait=True,
            )
            _, answer = self.subtask_manager.hri.confirm(
                "Please say yes once the dishwasher door is open.",
                use_hotwords=True,
                retries=5,
                wait_between_retries=10.0,
            )
            if answer == "yes":
                CLog.hri(self, "CONFIRM", "Referee confirmed dishwasher is open.", level="success")
            else:
                CLog.hri(self, "CONFIRM", "No confirmation, assuming dishwasher is open.", level="warn")
            self.dishwasher_open = True
            self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        # ==================== NAVIGATE TO PLACEMENT ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT)
            placement_loc = self.grasped_object.placement_location
            result = self.navigate_to_location(placement_loc)

            if result == Status.EXECUTION_SUCCESS:
                if placement_loc == Location.CABINET and not self.shelf_categories:
                    self.current_state = PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES
                else:
                    self.current_state = PickAndPlaceTM.TaskStates.PLACE_OBJECT
            else:
                CLog.nav(self, "FAIL", f"Navigation to {placement_loc.value} failed, skipping object.", level="error")
                self.grasped_object.is_picked = True
                self.current_object_index += 1
                self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== PERCEIVE CABINET SHELVES ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES:
            self._track_state_change(PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES)
            self.subtask_manager.manipulation.move_to_position("front_stare")

            shelf_status, shelf_detections = self.subtask_manager.vision.detect_shelf()

            if shelf_status == Status.EXECUTION_SUCCESS and shelf_detections:
                for shelf in shelf_detections:
                    level = shelf.level
                    shelf_bbox = BBOX(
                        x1=shelf.x1, y1=shelf.y1, x2=shelf.x2, y2=shelf.y2
                    )
                    status_q, description = self.subtask_manager.vision.moondream_crop_query(
                        "What type of objects are on this shelf? Answer in a few words.",
                        shelf_bbox,
                    )
                    if status_q == Status.EXECUTION_SUCCESS and description:
                        self.shelf_categories[level] = description.strip()
                    else:
                        self.shelf_categories[level] = ""

                CLog.vision(self, "SHELF", f"Cabinet shelf categories: {self.shelf_categories}")
                for level, cat in self.shelf_categories.items():
                    height = self.shelf_level_heights.get(level, self.default_shelf_height)
                    self.subtask_manager.hri.say(
                        f"Shelf {level} at {height} metres has {cat}.", wait=False
                    )
            else:
                CLog.vision(self, "SHELF", "Shelf detection failed, using default height.", level="warn")

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
                status = self.subtask_manager.manipulation.place()
            elif placement_loc == Location.TRASH_BIN:
                status = self.subtask_manager.manipulation.place()
            elif placement_loc == Location.CABINET:
                shelf_height = self._find_shelf_height_for_object(self.grasped_object)
                status = self.subtask_manager.manipulation.place_on_shelf(
                    plane_height=shelf_height,
                    tolerance=0.3,
                )
            else:
                status = self.subtask_manager.manipulation.place()

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_placed = True
                CLog.manip(self, "PLACE", f"Placed {self.grasped_object.name} at {placement_loc.value}.", level="success")
            else:
                CLog.manip(self, "PLACE", f"Failed to place {self.grasped_object.name}.", level="error")

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
                CLog.fsm(self, "STATE", f"Next breakfast item: {self.current_breakfast_item['name']} from {self.current_breakfast_item['location'].value}")
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE
            else:
                CLog.fsm(self, "STATE", "All breakfast items collected.", level="success")
                self.current_state = PickAndPlaceTM.TaskStates.END

        # ==================== NAVIGATE TO ITEM SOURCE ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE)
            item_location = self.current_breakfast_item["location"]
            result = self.navigate_to_location(item_location)

            if result == Status.EXECUTION_SUCCESS:
                self.current_state = PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM
            else:
                CLog.nav(self, "FAIL", f"Failed to reach {item_location.value}, skipping {self.current_breakfast_item['name']}.", level="error")
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== PICK BREAKFAST ITEM ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            item_location = self.current_breakfast_item["location"]

            if item_location == Location.CABINET:
                self.subtask_manager.manipulation.move_to_position("front_stare")
            else:
                self.subtask_manager.manipulation.move_to_position("table_stare")

            self.subtask_manager.hri.say(f"I will pick the {item_name}.", wait=False)

            yolo_name = self._to_yolo_name(item_name)

            if self.categorize_object(item_name) == ObjectCategory.CUTLERY:
                status = self.subtask_manager.manipulation.pick_cutlery(yolo_name)
            else:
                status = self.subtask_manager.manipulation.pick_object(yolo_name)

            if status == Status.EXECUTION_SUCCESS:
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING
            else:
                CLog.manip(self, "PICK", f"Failed to pick breakfast item: {item_name}.", level="error")
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== NAVIGATE TO DINING ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING)
            result = self.navigate_to_location(Location.DINING_TABLE)

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

            status = self.subtask_manager.manipulation.pour(
                pour_object_name=self._to_yolo_name(item_name),
                pour_container_name=self._to_yolo_name("bowl"),
                object_already_grasped=True,
            )

            if status == Status.EXECUTION_SUCCESS:
                CLog.manip(self, "POUR", f"Poured {item_name} into bowl.", level="success")
            else:
                CLog.manip(self, "POUR", f"Pour failed for {item_name}, will place without pouring.", level="warn")

            self.current_state = PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM

        # ==================== PLACE BREAKFAST ITEM ====================
        elif self.current_state == PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            self.subtask_manager.hri.say(f"Placing the {item_name}.", wait=False)

            close_to_logical = self.current_breakfast_item.get("close_to", "")
            close_to = self._to_yolo_name(close_to_logical) if close_to_logical else ""
            status = self.subtask_manager.manipulation.place(close_to=close_to)

            if status == Status.EXECUTION_SUCCESS:
                self.current_breakfast_item["placed"] = True
                if item_name == "bowl":
                    self.bowl_placed = True
                CLog.manip(self, "PLACE", f"Placed breakfast item: {item_name}.", level="success")
            else:
                CLog.manip(self, "PLACE", f"Failed to place breakfast item: {item_name}.", level="error")

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

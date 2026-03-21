#!/usr/bin/env python3

"""
Task Manager for Pick and Place Challenge - RoboCup @Home 2026
"""

import time
from datetime import datetime
from enum import Enum

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


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
        PLACE_BREAKFAST_ITEM = "PLACE_BREAKFAST_ITEM"
        VERIFY_BREAKFAST_AREA = "VERIFY_BREAKFAST_AREA"

        END = "END"
        DEBUG = "DEBUG"

    def __init__(self):
        """Initialize the node"""
        super().__init__("pickandplace_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.PICK_AND_PLACE, mock_areas=[])

        # ACTION REQUIRED: Adjust before competition
        self.use_side_table = False  # True = use side table objects (penalty per object)
        self.trash_category = "napkin"  # Announced during Setup Days

        # Object tracking
        self.detected_objects: list[ObjectInfo] = []
        self.current_object_index = 0
        self.grasped_object: ObjectInfo = None
        self.first_pick = True

        # Breakfast items — bowl+spoon first (breakfast_surface), then milk+cereal (cabinet)
        self.breakfast_items = [
            {"name": "bowl",   "location": Location.BREAKFAST_SURFACE, "picked": False, "placed": False},
            {"name": "spoon",  "location": Location.BREAKFAST_SURFACE, "picked": False, "placed": False},
            {"name": "milk",   "location": Location.CABINET,           "picked": False, "placed": False},
            {"name": "cereal", "location": Location.CABINET,           "picked": False, "placed": False},
        ]
        self.current_breakfast_item: dict = None

        # Dishwasher state
        self.dishwasher_open = False

        # Cabinet shelf categories — populated after perception
        # Format: { shelf_level_int: "category_string", ... }  (same as hri.categorize_objects output)
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
        Logger.info(self, "PickAndPlaceTaskManager has started.")


    def _track_state_change(self, new_state: str):
        """Track time spent in each state and reset attempt counter"""
        current_time = datetime.now()

        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent
            Logger.info(self, f"State '{self.previous_state}' took {time_spent:.2f} seconds")

        self.current_attempts = 0  # always reset on state change

        self.previous_state = new_state
        self.state_start_time = current_time

        if self.state_times:
            total_time = sum(self.state_times.values())
            Logger.info(self, f"Total time elapsed: {total_time:.2f} seconds")

        Logger.state(self, self.current_state)

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to a location with retry logic"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        self.subtask_manager.nav.resume_nav()

        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(f"Moving to {location}.", wait=False)

        result = Status.EXECUTION_ERROR
        retry = 0
        while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
            future = self.subtask_manager.nav.move_to_location(location, sublocation)
            if "navigation" not in self.subtask_manager.get_mocked_areas():
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

    def determine_placement_location(self, obj: ObjectInfo) -> Location:
        """Return the target furniture for an object"""
        if obj.category in (ObjectCategory.CUTLERY, ObjectCategory.TABLEWARE):
            return Location.DISHWASHER
        elif obj.category == ObjectCategory.TRASH:
            return Location.TRASH_BIN
        else:
            return Location.CABINET

    def run(self):
        """Finite State Machine"""

        # ==================== WAIT FOR START ====================
        if self.current_state == PickAndPlaceTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)

            Logger.success(self, "Start button pressed. Pick and Place Challenge begins.")
            self.current_state = PickAndPlaceTM.TaskStates.START

        # ==================== START ====================
        if self.current_state == PickAndPlaceTM.TaskStates.START:
            self._track_state_change(PickAndPlaceTM.TaskStates.START)
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.navigate_to("kitchen", say=False)

            if self.use_side_table:
                Logger.info(self, "Using side table (common objects).")
                self.subtask_manager.hri.say("I will clean the side table.", wait=False)
            else:
                self.subtask_manager.hri.say("I will clean the dining table.", wait=False)

            self.current_state = PickAndPlaceTM.TaskStates.PERCEIVE_TABLE

        # ==================== PERCEIVE TABLE ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PERCEIVE_TABLE:
            self._track_state_change(PickAndPlaceTM.TaskStates.PERCEIVE_TABLE)
            table_location = "side_table" if self.use_side_table else "dining_table"
            self.navigate_to(table_location, say=False)
            self.subtask_manager.manipulation.move_to_position("table_stare")

            # detect_objects returns (Status, list[BBOX])
            # Each BBOX has: .classname, .point3d, .x, .y, .distance, etc.
            status, detections = self.subtask_manager.vision.detect_objects(timeout=5)

            if status == Status.EXECUTION_SUCCESS and detections:
                self.detected_objects = []
                for bbox in detections:
                    obj_name = bbox.classname if bbox.classname else "unknown"
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
                Logger.info(self, f"Detected {len(self.detected_objects)} objects on the table.")
                self.current_state = PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS
            else:
                Logger.warning(self, "No objects detected, skipping to breakfast phase.")
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== ANNOUNCE OBJECTS ====================
        if self.current_state == PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS:
            self._track_state_change(PickAndPlaceTM.TaskStates.ANNOUNCE_OBJECTS)
            self.subtask_manager.hri.say(
                f"I have detected {len(self.detected_objects)} objects on the table."
            )

            # Per rules: robot must communicate perception to the referee.
            # Saying the name is sufficient; pointing is ideal but not required to score.
            for obj in self.detected_objects:
                Logger.info(self, f"Object: {obj.name}, Category: {obj.category.value}")
                self.subtask_manager.hri.say(f"I see a {obj.name}.")
                self.timeout(0.5)

            self.current_object_index = 0
            self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== CLEANUP LOOP ====================
        if self.current_state == PickAndPlaceTM.TaskStates.CLEANUP_LOOP:
            self._track_state_change(PickAndPlaceTM.TaskStates.CLEANUP_LOOP)

            # Skip already handled objects
            while (
                self.current_object_index < len(self.detected_objects)
                and self.detected_objects[self.current_object_index].is_picked
            ):
                self.current_object_index += 1

            if self.current_object_index < len(self.detected_objects):
                self.grasped_object = self.detected_objects[self.current_object_index]
                self.current_state = PickAndPlaceTM.TaskStates.PICK_OBJECT
            else:
                Logger.success(self, "All objects cleaned up.")
                self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP

        # ==================== PICK OBJECT ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PICK_OBJECT:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_OBJECT)

            table_location = "side_table" if self.use_side_table else "dining_table"
            self.navigate_to(table_location, say=False)
            self.subtask_manager.manipulation.move_to_position("table_stare")

            self.subtask_manager.hri.say(
                f"I will pick the {self.grasped_object.name}.", wait=False
            )

            # pick_object expects the object name; manipulation server uses active vision
            status = self.subtask_manager.manipulation.pick_object(
                self.grasped_object.name
            )

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_picked = True
                if self.first_pick:
                    Logger.success(self, "FIRST PICK BONUS achieved!")
                    self.first_pick = False
                self.current_attempts = 0
                self.current_state = PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT
            else:
                Logger.error(self, f"Failed to pick {self.grasped_object.name}.")
                self.current_attempts += 1
                if self.current_attempts >= ATTEMPT_LIMIT:
                    Logger.warning(self, f"Skipping {self.grasped_object.name} after max attempts.")
                    self.current_attempts = 0
                    self.grasped_object.is_picked = True   # mark as handled to advance loop
                    self.current_object_index += 1
                    self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP
                # else stay in PICK_OBJECT to retry

        # ==================== DETERMINE PLACEMENT ====================
        if self.current_state == PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.DETERMINE_PLACEMENT)
            self.grasped_object.placement_location = self.determine_placement_location(
                self.grasped_object
            )
            Logger.info(
                self,
                f"Placing {self.grasped_object.name} → {self.grasped_object.placement_location.value}",
            )

            if (
                self.grasped_object.placement_location == Location.DISHWASHER
                and not self.dishwasher_open
            ):
                self.current_state = PickAndPlaceTM.TaskStates.CHECK_DISHWASHER
            else:
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        # ==================== CHECK DISHWASHER ====================
        if self.current_state == PickAndPlaceTM.TaskStates.CHECK_DISHWASHER:
            self._track_state_change(PickAndPlaceTM.TaskStates.CHECK_DISHWASHER)
            self.navigate_to(Location.DISHWASHER.value, say=False)
            status, answer = self.subtask_manager.vision.moondream_query(
                "Is the dishwasher door open? Reply only with yes or no."
            )
            if status == Status.EXECUTION_SUCCESS and "yes" in answer.lower():
                self.dishwasher_open = True
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT
            else:
                self.current_state = PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP

        # ==================== REQUEST DISHWASHER HELP ====================
        if self.current_state == PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP:
            self._track_state_change(PickAndPlaceTM.TaskStates.REQUEST_DISHWASHER_HELP)
            # Per rules: requesting help with dishwasher door/rack has NO score penalty
            self.subtask_manager.hri.say(
                "I cannot open the dishwasher door. Could you please open it for me?",
                wait=True,
            )
            _, answer = self.subtask_manager.hri.confirm(
                "Please say yes once the dishwasher door is open.",
                use_hotwords=True,
                retries=5,
                wait_between_retries=10.0,
            )
            if answer == "yes":
                Logger.success(self, "Referee confirmed dishwasher is open.")
                self.subtask_manager.hri.say("Thank you! I will now place the items.", wait=False)
            else:
                Logger.warning(self, "No confirmation received; assuming dishwasher is open.")
                self.subtask_manager.hri.say(
                    "I did not hear a confirmation, but I will proceed.", wait=False
                )
            self.dishwasher_open = True
            self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT

        # ==================== NAVIGATE TO PLACEMENT ====================
        if self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_PLACEMENT)
            placement_loc = self.grasped_object.placement_location
            result = self.navigate_to(placement_loc.value)

            if result == Status.EXECUTION_SUCCESS:
                # Only perceive cabinet shelves on first cabinet visit
                if placement_loc == Location.CABINET and not self.shelf_categories:
                    self.current_state = PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES
                else:
                    self.current_state = PickAndPlaceTM.TaskStates.PLACE_OBJECT
            else:
                Logger.error(self, "Navigation to placement failed, skipping object.")
                self.grasped_object.is_picked = True    # avoid re-trying this object
                self.current_object_index += 1
                self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== PERCEIVE CABINET SHELVES ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES:
            self._track_state_change(PickAndPlaceTM.TaskStates.PERCEIVE_CABINET_SHELVES)
            self.subtask_manager.manipulation.move_to_position("cabinet_stare")

            # detect_shelf returns (Status, list[ShelfDetection])
            # Each ShelfDetection has .level, .x1, .x2, .y1, .y2
            shelf_status, shelf_detections = self.subtask_manager.vision.detect_shelf()

            if shelf_status == Status.EXECUTION_SUCCESS and shelf_detections:
                # Build a dict { level: [object_names...] } from moondream queries per shelf
                raw_shelves: dict[int, list[str]] = {}
                for shelf in shelf_detections:
                    level = shelf.level
                    status_q, description = self.subtask_manager.vision.moondream_query(
                        "List the objects on this shelf, separated by commas."
                    )
                    objects_on_shelf = []
                    if status_q == Status.EXECUTION_SUCCESS and description:
                        objects_on_shelf = [o.strip() for o in description.split(",") if o.strip()]
                    raw_shelves[level] = objects_on_shelf

                # Use hri.categorize_objects to get semantic grouping
                # It returns (Status, {level: "category_str"}, {level: [objects_to_add]}, {level: [cats]})
                table_object_names = [obj.name for obj in self.detected_objects if not obj.is_placed]
                cat_status, shelf_cat_str, _, _ = self.subtask_manager.hri.categorize_objects(
                    table_objects=table_object_names,
                    shelves=raw_shelves,
                )
                if cat_status == Status.EXECUTION_SUCCESS:
                    self.shelf_categories = shelf_cat_str   # {level: "category string"}
                    Logger.info(self, f"Cabinet shelf categories: {self.shelf_categories}")
                    # Per rules: must announce shelf perception to the referee
                    for level, cat in self.shelf_categories.items():
                        self.subtask_manager.hri.say(
                            f"Shelf {level} contains {cat}.", wait=False
                        )
                        self.timeout(0.5)
                else:
                    Logger.warning(self, "Categorization failed, using empty shelf map.")
                    self.shelf_categories = {}
            else:
                Logger.warning(self, "Shelf detection failed, proceeding without shelf map.")
                self.shelf_categories = {}

            self.current_state = PickAndPlaceTM.TaskStates.PLACE_OBJECT

        # ==================== PLACE OBJECT ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PLACE_OBJECT:
            self._track_state_change(PickAndPlaceTM.TaskStates.PLACE_OBJECT)
            placement_loc = self.grasped_object.placement_location
            self.subtask_manager.hri.say(
                f"Placing the {self.grasped_object.name} at the {placement_loc.value}.",
                wait=False,
            )

            if placement_loc == Location.DISHWASHER:
                # place() lets the manipulation server decide exact position in the rack
                status = self.subtask_manager.manipulation.place()

            elif placement_loc == Location.TRASH_BIN:
                status = self.subtask_manager.manipulation.place()

            elif placement_loc == Location.CABINET:
                # place_on_shelf needs an estimated height; use a generic mid-cabinet height.
                # A refinement would be to look up the height from shelf_categories.
                status = self.subtask_manager.manipulation.place_on_shelf(
                    plane_height=1.0,   # metres — adjust to arena cabinet height
                    tolerance=0.3,
                )
            else:
                status = self.subtask_manager.manipulation.place()

            if status == Status.EXECUTION_SUCCESS:
                self.grasped_object.is_placed = True
                Logger.success(self, f"Placed {self.grasped_object.name} at {placement_loc.value}.")
            else:
                Logger.error(self, f"Failed to place {self.grasped_object.name}.")

            self.current_object_index += 1
            self.grasped_object = None
            self.current_state = PickAndPlaceTM.TaskStates.CLEANUP_LOOP

        # ==================== START BREAKFAST PREP ====================
        if self.current_state == PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP:
            self._track_state_change(PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP)
            self.subtask_manager.hri.say("Now I will prepare breakfast.", wait=False)
            self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== GET BREAKFAST ITEMS ====================
        if self.current_state == PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS:
            self._track_state_change(PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS)
            self.current_breakfast_item = None
            for item in self.breakfast_items:
                if not item["picked"]:
                    self.current_breakfast_item = item
                    break

            if self.current_breakfast_item:
                Logger.info(
                    self,
                    f"Next breakfast item: {self.current_breakfast_item['name']} "
                    f"from {self.current_breakfast_item['location'].value}",
                )
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE
            else:
                Logger.success(self, "All breakfast items collected.")
                self.current_state = PickAndPlaceTM.TaskStates.VERIFY_BREAKFAST_AREA

        # ==================== NAVIGATE TO ITEM SOURCE ====================
        if self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_ITEM_SOURCE)
            item_location = self.current_breakfast_item["location"]
            result = self.navigate_to(item_location.value)

            if result == Status.EXECUTION_SUCCESS:
                self.current_state = PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM
            else:
                Logger.error(
                    self,
                    f"Failed to reach {item_location.value}, "
                    f"skipping {self.current_breakfast_item['name']}.",
                )
                self.current_breakfast_item["picked"] = True   # avoid infinite loop
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== PICK BREAKFAST ITEM ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PICK_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            self.subtask_manager.manipulation.move_to_position("table_stare")
            self.subtask_manager.hri.say(f"I will pick the {item_name}.", wait=False)

            status = self.subtask_manager.manipulation.pick_object(item_name)

            if status == Status.EXECUTION_SUCCESS:
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING
            else:
                Logger.error(self, f"Failed to pick {item_name}.")
                # Mark as picked anyway to avoid being stuck; breakfast item is skipped
                self.current_breakfast_item["picked"] = True
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== NAVIGATE TO DINING ====================
        if self.current_state == PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING:
            self._track_state_change(PickAndPlaceTM.TaskStates.NAVIGATE_TO_DINING)
            result = self.navigate_to("dining_table")

            if result == Status.EXECUTION_SUCCESS:
                self.current_state = PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM
            else:
                Logger.error(self, "Failed to navigate to dining table.")
                self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== PLACE BREAKFAST ITEM ====================
        if self.current_state == PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM:
            self._track_state_change(PickAndPlaceTM.TaskStates.PLACE_BREAKFAST_ITEM)
            item_name = self.current_breakfast_item["name"]
            self.subtask_manager.hri.say(f"Placing the {item_name}.", wait=False)

            # Per rules: spoon next to bowl; cereal and milk next to each other; 5 cm clearance.
            # place() lets the manipulation server handle exact positioning on the table.
            status = self.subtask_manager.manipulation.place()

            if status == Status.EXECUTION_SUCCESS:
                self.current_breakfast_item["placed"] = True
                Logger.success(self, f"Placed {item_name}.")
            else:
                Logger.error(self, f"Failed to place {item_name}.")

            self.current_state = PickAndPlaceTM.TaskStates.GET_BREAKFAST_ITEMS

        # ==================== VERIFY BREAKFAST AREA ====================
        if self.current_state == PickAndPlaceTM.TaskStates.VERIFY_BREAKFAST_AREA:
            self._track_state_change(PickAndPlaceTM.TaskStates.VERIFY_BREAKFAST_AREA)
            # Per rules: objects within 5 cm of breakfast items incur a penalty.
            # Use moondream to check if the area around the breakfast items is clear.
            status, answer = self.subtask_manager.vision.moondream_query(
                "Is there any object within 5 centimetres of the bowl, spoon, milk, or cereal? "
                "Reply only with yes or no."
            )
            if status == Status.EXECUTION_SUCCESS and "yes" in answer.lower():
                Logger.warning(self, "Breakfast area not clear — objects too close.")
                self.subtask_manager.hri.say(
                    "There are objects too close to the breakfast items. "
                    "I will try to move them.",
                    wait=False,
                )
                # Best-effort: detect and remove nearby objects
                det_status, nearby = self.subtask_manager.vision.detect_objects(timeout=5)
                if det_status == Status.EXECUTION_SUCCESS and nearby:
                    # Pick the closest object to the breakfast area and drop it elsewhere
                    nearest = min(nearby, key=lambda b: b.distance)
                    pick_st = self.subtask_manager.manipulation.pick_object(nearest.classname)
                    if pick_st == Status.EXECUTION_SUCCESS:
                        self.subtask_manager.manipulation.place_on_floor()
            else:
                Logger.success(self, "Breakfast area is clear.")

            self.subtask_manager.hri.say("Breakfast is ready!")
            self.current_state = PickAndPlaceTM.TaskStates.END

        # ==================== END ====================
        if self.current_state == PickAndPlaceTM.TaskStates.END:
            Logger.state(self, "Task completed.")
            self._track_state_change(PickAndPlaceTM.TaskStates.END)

            total_task_time = (datetime.now() - self.total_start_time).total_seconds()
            Logger.info(self, "=== FINAL TIMING REPORT ===")
            Logger.info(self, f"Total task time: {total_task_time:.2f} seconds")

            sorted_states = sorted(self.state_times.items(), key=lambda x: x[1], reverse=True)
            for state, time_spent in sorted_states:
                percentage = (time_spent / total_task_time) * 100 if total_task_time > 0 else 0
                Logger.info(self, f"{state}: {time_spent:.2f}s ({percentage:.1f}%)")

            Logger.info(self, "=== END TIMING REPORT ===")
            self.subtask_manager.hri.say("I have completed the pick and place task.")
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.running_task = False

        # ==================== DEBUG ====================
        if self.current_state == PickAndPlaceTM.TaskStates.DEBUG:
            Logger.state(self, "Debugging task.")
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
        Logger.info(node, "Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
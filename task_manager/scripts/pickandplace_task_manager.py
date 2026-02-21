#!/usr/bin/env python3

"""
Task Manager for Pick and Place Challenge - RoboCup @Home 2026
"""

from enum import Enum

# import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from utils.logger import Logger

ATTEMPT_LIMIT = 3


class ObjectCategory(Enum):
    """Object categories for classification"""

    CUTLERY = "cutlery"
    TABLEWARE = "tableware"
    TRASH = "trash"
    OTHER = "other"
    BREAKFAST = "breakfast"
    COMMON = "common"
    DISHWASHER_TAB = "dishwasher_tab"


class Location(Enum):
    """Known locations in the arena"""

    DINING_TABLE = "dining_table"
    SIDE_TABLE = "side_table"
    DISHWASHER = "dishwasher"
    CABINET = "cabinet"
    TRASH_BIN = "trash_bin"
    FLOOR = "floor"
    BREAKFAST_SURFACE = "breakfast_surface"
    KITCHEN = "kitchen"
    LIVING_ROOM = "living_room"


class ObjectInfo:
    """Class to store object information"""

    def __init__(
        self,
        name: str = None,
        category: ObjectCategory = None,
        position: dict = None,
        detection_data: dict = None,
    ):
        self.name = name
        self.category = category
        self.position = position or {}
        self.detection_data = detection_data or {}
        self.is_picked = False
        self.is_placed = False


class PickAndPlaceTM(Node):
    """Task Manager for Pick and Place Challenge"""

    class TaskStates:
        """States for the task state machine"""

        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        WAIT_FOR_DOOR = "WAIT_FOR_DOOR"
        NAVIGATE_TO_KITCHEN = "NAVIGATE_TO_KITCHEN"
        DECIDE_TABLE = "DECIDE_TABLE"

        # Cleanup Phase
        PERCEIVE_DINING_TABLE = "PERCEIVE_DINING_TABLE"
        PERCEIVE_SIDE_TABLE = "PERCEIVE_SIDE_TABLE"
        ANNOUNCE_OBJECTS = "ANNOUNCE_OBJECTS"
        CLEANUP_LOOP = "CLEANUP_LOOP"
        PICK_OBJECT = "PICK_OBJECT"
        DETERMINE_PLACEMENT = "DETERMINE_PLACEMENT"
        NAVIGATE_TO_PLACEMENT = "NAVIGATE_TO_PLACEMENT"
        CHECK_DISHWASHER = "CHECK_DISHWASHER"
        REQUEST_DISHWASHER_OPEN = "REQUEST_DISHWASHER_OPEN"
        PERCEIVE_CABINET_SHELVES = "PERCEIVE_CABINET_SHELVES"
        PLACE_OBJECT = "PLACE_OBJECT"

        # Breakfast Phase
        START_BREAKFAST_PREP = "START_BREAKFAST_PREP"
        GET_BREAKFAST_ITEMS = "GET_BREAKFAST_ITEMS"
        NAVIGATE_TO_BREAKFAST_SURFACE = "NAVIGATE_TO_BREAKFAST_SURFACE"
        PICK_BREAKFAST_ITEM = "PICK_BREAKFAST_ITEM"
        NAVIGATE_TO_DINING = "NAVIGATE_TO_DINING"
        ARRANGE_BREAKFAST = "ARRANGE_BREAKFAST"
        VERIFY_BREAKFAST_AREA = "VERIFY_BREAKFAST_AREA"

        # Optional Tasks
        OPTIONAL_TASKS_DECIDER = "OPTIONAL_TASKS_DECIDER"
        FLOOR_CLEANUP = "FLOOR_CLEANUP"
        OPEN_MILK = "OPEN_MILK"
        POUR_MILK = "POUR_MILK"
        POUR_CEREAL = "POUR_CEREAL"
        DISHWASHER_TAB_TASK = "DISHWASHER_TAB_TASK"

        END = "END"
        DEBUG = "DEBUG"

    def track_state_change(self, new_state: str):
        """Track state changes and time spent in each state"""
        current_time = datetime.now()

        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent

            Logger.info(self, f"State '{self.previous_state}' took {time_spent:.2f} seconds")

        self.previous_state = new_state
        self.state_start_time = current_time

        if self.state_times:
            total_time = sum(self.state_times.values())
            Logger.info(self, f"Total time elapsed: {total_time:.2f} seconds")


def main(args=None):
    rclpy.init(args=args)
    tm = PickAndPlaceTM()
    rclpy.spin(tm)
    tm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

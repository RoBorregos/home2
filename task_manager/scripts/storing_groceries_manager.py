#!/usr/bin/env python3

from collections import defaultdict
from enum import Enum

import rclpy
from frida_constants.vision_classes import BBOX, ShelfDetection
from pydantic import BaseModel
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task


class ExecutionStates(Enum):
    START = 0

    INIT_NAV_TO_SHELF = 10
    FAILED_NAV_TO_SHELF = 11
    SUCCEDED_NAV_TO_SHELF = 12

    VIEW_SHELF_AND_SAVE_OBJECTS = 20
    INIT_NAV_TO_TABLE = 30
    VIEW_AND_SAVE_OBJECTS_ON_TABLE = 40
    CATEGORIZE_OBJECTS = 50
    SAY_5_OBJECTS_CATEGORIZED = 55

    PLAN_NEXT = 59

    NAV_TO_TABLE = 60
    NAV_TO_SHELF = 70

    PICK_OBJECT = 60
    PLACE_OBJECT = 70

    CEREAL_ANALYSIS = 100
    CEREAL_PICK = 110
    CEREAL_PLACE = 120

    END = 200

    # TRY_HEAL = 500


class Shelf(BaseModel):
    id: int
    tag: str = None
    objects: list[str] = []


class StoringGroceriesManager(Node):
    def __init__(self):
        super().__init__("storing_groceries_manager")
        self.logger = Logger()
        self.subtask_manager = SubtaskManager(
            self, task=Task.STORING_GROCERIES, mock_areas=["manipulation"]
        )
        self.state = ExecutionStates.START
        self.state_data = {}
        self.shelves: dict[int, Shelf] = defaultdict(Shelf)
        self.objects_on_table: list[BBOX] = []
        self.object_names_on_table: list[str] = []
        self.shelves_count = 0
        self.object_to_placing_shelf: dict[str, list[int]]
        self.current_object: str = None

        self.retry_count = 0

    def nav_to(self, location: str, sub_location: str = "", say: bool = True) -> Status:
        Logger.info(self, f"Navigating to {location} {sub_location}")
        try:
            if say:
                self.subtask_manager.hri.say(text=f"Going to {location} {sub_location}", wait=True)
            future = self.subtask_manager.nav.move_to_location(location, sub_location)
            rclpy.spin_until_future_complete(self, future)
            hres = future.result()
            return Status.EXECUTION_SUCCESS if hres == 1 else Status.EXECUTION_ERROR
        except Exception as e:
            Logger.error(self, f"Error navigating to {location}: {e}")
        return Status.EXECUTION_ERROR

    def exec_state(self):
        Logger.info(self, f"Executing state: {self.state.name}")
        if self.state == ExecutionStates.START:
            self.state = ExecutionStates.INIT_NAV_TO_SHELF
        elif self.state == ExecutionStates.END:
            Logger.info(self, "Ending Storing Groceries Manager...")
            self.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
            return
        elif self.state == ExecutionStates.INIT_NAV_TO_SHELF:
            hres: Status = self.nav_to("kitchen", "shelve")
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.SUCCEDED_NAV_TO_SHELF
            else:
                self.state = ExecutionStates.FAILED_NAV_TO_SHELF
        elif self.state == ExecutionStates.SUCCEDED_NAV_TO_SHELF:
            self.state = ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS
        elif self.state == ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS:
            status, results = self.subtask_manager.vision.detect_shelf(timeout=10)
            results: list[ShelfDetection]
            self.shelves_count = max([i.level for i in results]) + (
                1 if len(0 in [i.level for i in results]) > 0 else 0
            )
            if status == Status.TIMEOUT:
                pass  # should retry
                return
            elif status == Status.EXECUTION_SUCCESS:
                pass  # rn todo
            else:
                Logger.error(self, "Unknown status")
                return

            status, res = self.subtask_manager.vision.detect_objects()

            if status == Status.EXECUTION_SUCCESS:
                Logger.info(self, f"Detected objects: {res}")
                res: list[BBOX]
            else:
                Logger.error(self, "Unknown status")
                return

            for det in res:
                # Logger.info(self, f"Detected object: {det.classname}")
                detected_shelf: ShelfDetection = None
                center_x = det.x1 + det.w / 2
                center_y = det.y1 + det.h / 2
                for shelf in results:
                    if shelf.x1 < center_x < shelf.x2 and shelf.y1 < center_y < shelf.y2:
                        detected_shelf = shelf
                        break
                if detected_shelf is None:
                    Logger.error(self, f"Detected object {det.classname} not in any shelf")
                    continue
                Logger.info(
                    self, f"Detected object {det.classname} in shelf {detected_shelf.level}"
                )
                self.shelves[detected_shelf.level].id = detected_shelf.level
                self.shelves[detected_shelf.level].objects.append(det.classname)

            Logger.info(self, "Shelves: %s", self.shelves)
            self.state = ExecutionStates.INIT_NAV_TO_TABLE

            len(self.shelves)

        elif self.state == ExecutionStates.INIT_NAV_TO_TABLE:
            hres: Status = self.nav_to("kitchen", "table")
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE
            else:
                self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                Logger.error(self, "Failed to navigate to table")
        elif self.state == ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE:
            status, result = self.subtask_manager.vision.detect_objects(timeout=10)
            if status == Status.TIMEOUT:
                # pass
                return
            elif status == Status.EXECUTION_SUCCESS:
                pass
            else:
                Logger.error(self, "Unknown status")
                return
            Logger.info(self, f"Detected objects: {result}")
            result: list[BBOX]
            self.objects_on_table = result
            self.object_names_on_table = [i.classname for i in result]
            self.state = ExecutionStates.CATEGORIZE_OBJECTS
        elif self.state == ExecutionStates.CATEGORIZE_OBJECTS:
            missing_shelves = [i for i in range(self.shelves_count) if i not in self.shelves]
            for i in missing_shelves:
                self.shelves[i] = Shelf(id=i)
            shelfs: dict[int, list[str]] = {}
            for i in self.shelves:
                shelfs[i] = self.shelves[i].objects
            status, categorized_shelfs, objects_to_add = (
                self.subtask_manager.hri.categorize_objects(self.object_names_on_table, shelfs)
            )
            if not status == Status.EXECUTION_SUCCESS:
                Logger.error(self, "Failed to categorize objects")
                return
            Logger.info(self, f"Categorized shelfs: {categorized_shelfs}")
            Logger.info(self, f"Objects to add: {objects_to_add}")
            for i in categorized_shelfs:
                self.shelves[i].tag = categorized_shelfs[i]
            for i in objects_to_add:
                for j in objects_to_add[i]:
                    self.object_to_placing_shelf[j].append(i)
            Logger.info(self, f"Shelves: {self.shelves}")
            Logger.info(self, f"Objects to place: {self.object_to_placing_shelf}")
            self.state = ExecutionStates.SAY_5_OBJECTS_CATEGORIZED
            self.subtask_manager.hri.say(
                text=f"Categorized {len(self.object_names_on_table)} objects", wait=True
            )
        elif self.state == ExecutionStates.SAY_5_OBJECTS_CATEGORIZED:
            for i in list(set(self.object_names_on_table))[
                : max(5, len(self.object_names_on_table))
            ]:
                self.subtask_manager.hri.say(
                    text=f"Categorized object: {i} as {self.shelves[self.object_to_placing_shelf[i]].tag}",
                    wait=True,
                )
            self.state = ExecutionStates.PLAN_NEXT
        elif self.state == ExecutionStates.PLAN_NEXT:
            if len(self.object_names_on_table) == 0:
                self.state = ExecutionStates.END
                return
            self.state = ExecutionStates.NAV_TO_TABLE
        elif self.state == ExecutionStates.NAV_TO_TABLE:
            hres: Status = self.nav_to("kitchen", "table")
            if not hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                return
            self.state = ExecutionStates.PICK_OBJECT
        elif self.state == ExecutionStates.PICK_OBJECT:
            status, objs = self.subtask_manager.vision.detect_objects(timeout=10)
            if status == Status.TIMEOUT:
                # pass
                return
            elif not status == Status.EXECUTION_SUCCESS:
                Logger.error(self, "Unknown status")
                return
            objs: list[BBOX]
            Logger.info(self, f"Detected objects: {objs}")
            if len(objs) == 0:
                Logger.error(self, "No objects detected")
                self.subtask_manager.hri.say(text="No objects detected", wait=True)
                return
            Logger.info(self, f"Detected objects: {objs}")
            min_distance = 1000000000
            min_distance_obj: BBOX = None
            for obj in objs:
                if obj.distance < min_distance:
                    min_distance = obj.distance
                    min_distance_obj = obj
            if min_distance_obj is None:
                # Logger.error(self, "No objects detected")
                # self.subtask_manager.hri.say(
                #     text="No objects detected", wait=True)
                Logger.error(self, "AAAAASADQWEQWEASDQWEQWEQWEQWEQWE")
                return
            Logger.info(
                self,
                f"Detected object: {min_distance_obj.classname} at distance {min_distance_obj.distance}",
            )

            # Logger.info(self,
            #     f"Detected object: {min_distance_obj.classname}")
            self.subtask_manager.hri.say(
                text=f"I'm going to pick the object: {min_distance_obj.classname}", wait=True
            )
            self.current_object = min_distance_obj.classname
            hres: Status = self.subtask_manager.manipulation.pick_object(
                min_distance_obj.classname, timeout=10
            )
            if not hres == Status.EXECUTION_SUCCESS:
                # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                Logger.error(self, "Failed to pick object")
                return
            self.state = ExecutionStates.NAV_TO_SHELF
        elif self.state == ExecutionStates.NAV_TO_SHELF:
            hres: Status = self.nav_to("kitchen", "shelve")
            if not hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                return
            self.state = ExecutionStates.PLACE_OBJECT
        elif self.state == ExecutionStates.PLACE_OBJECT:
            status = self.subtask_manager.manipulation.place_in_shelf(
                self.object_to_placing_shelf[self.current_object].pop(0), self.current_object
            )
            if not status == Status.EXECUTION_SUCCESS:
                Logger.error(self, "Failed to place object")
                return
            Logger.info(
                self,
                f"Placed object: {self.current_object} in shelf {self.object_to_placing_shelf[self.current_object]}",
            )
            self.current_object = None
            self.state = ExecutionStates.PLAN_NEXT
        else:
            Logger.error(self, f"Unknown state: {self.state}")
            return

    def run(self):
        Logger.info(self, "Starting Storing Groceries Manager...")
        if self.state != ExecutionStates.END:
            self.exec_state()
            if self.state == ExecutionStates.END:
                Logger.info(self, "Ending Storing Groceries Manager...")
                self.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
                return
        Logger.info(self, "Starting Storing Groceries Manager...")


def main(args=None):
    """Main function"""
    print("Starting Storing Groceries Manager...")
    rclpy.init(args=args)
    node = StoringGroceriesManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

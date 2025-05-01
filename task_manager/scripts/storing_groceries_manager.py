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
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import PointTransformation
import time

POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"


class ExecutionStates(Enum):
    START = 0

    INIT_NAV_TO_SHELF = 10
    FAILED_NAV_TO_SHELF = 11
    SUCCEDED_NAV_TO_SHELF = 12

    VIEW_SHELF_AND_SAVE_OBJECTS = 20
    INIT_NAV_TO_TABLE = 30
    VIEW_AND_SAVE_OBJECTS_ON_TABLE = 40
    CATEGORIZE_OBJECTS = 50
    CATEGORIZE_OBJECTS2 = 51
    SAY_5_OBJECTS_CATEGORIZED = 55

    PLAN_NEXT = 59

    NAV_TO_TABLE = 60
    NAV_TO_SHELF = 70

    PICK_OBJECT = 65
    DEUX_PICK_OBJECT = 66
    PLACE_OBJECT = 75
    DEUX_PLACE_OBJECT = 76

    CEREAL_ANALYSIS = 100
    CEREAL_PICK = 110
    CEREAL_PLACE = 120

    END = 200

    # TRY_HEAL = 500


for i in ExecutionStates:
    for j in ExecutionStates:
        if i.value == j.value and i.name != j.name:
            raise ValueError(f"Duplicate value found: {i.value} for {i.name} and {j.name}")

STATE_TO_DEUX = {
    ExecutionStates.PICK_OBJECT: ExecutionStates.DEUX_PICK_OBJECT,
    ExecutionStates.PLACE_OBJECT: ExecutionStates.DEUX_PLACE_OBJECT,
}


class Shelf(BaseModel):
    id: int = 0
    tag: str = None
    objects: list[str] = []


class StoringGroceriesManager(Node):
    def __init__(self):
        super().__init__("storing_groceries_manager")
        self.logger = Logger()
        self.subtask_manager = SubtaskManager(self, task=Task.STORING_GROCERIES, mock_areas=[])
        self.state = ExecutionStates.START
        self.state_data = {}
        self.shelves: dict[int, Shelf] = defaultdict(Shelf)
        self.objects_on_table: list[BBOX] = []
        self.object_names_on_table: list[str] = []
        self.shelves_count = 0
        self.object_to_placing_shelf: dict[str, list[int]]
        self.current_object: str = None
        self.point_pub = self.create_publisher(PointStamped, "point_visualize", 10)
        self.retry_count = 0
        self.prev_state = None
        self.check_manual_levels = True
        self.manual_heights = [
            0.445,  # 0.445 (0.1 +-) -> 0.345 0.545
            0.805,  # 0.805 (0.1 +-) -> 0.705 0.905
            1.165,  # 1.165 (0.1 +-) -> 1.065 1.265
            1.525,  # 1.525 (0.1 +-) -> 1.425 1.625
        ]  # remember rest 15cm from the base_link and the measure is in m
        self.shelf_level_threshold = 0.30
        self.shelf_level_down_threshold = 0.05
        self.transform_tf = self.create_client(PointTransformation, POINT_TRANSFORMER_TOPIC)

    def generate_manual_levels(self):
        """Generate manual levels"""
        self.shelves_count = 0
        self.shelves = {}
        for i in range(len(self.manual_heights)):
            self.shelves[i] = Shelf(id=i, tag="", objects=[])
            self.shelves_count += 1

    def nav_to(self, location: str, sub_location: str = "", say: bool = True) -> Status:
        Logger.info(self, f"Navigating to {location} {sub_location}")
        try:
            if say:
                self.subtask_manager.hri.say(text=f"Going to {location} {sub_location}", wait=True)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="nav_pose", velocity=0.5, degrees=True
            )
            future = self.subtask_manager.nav.move_to_location(location, sub_location)
            rclpy.spin_until_future_complete(self, future)
            hres = future.result()
            return Status.EXECUTION_SUCCESS if hres == 1 else Status.EXECUTION_ERROR
        except Exception as e:
            Logger.error(self, f"Error navigating to {location}: {e}")
        return Status.EXECUTION_ERROR

    def convert_to_height(self, detection: BBOX) -> float:
        """Convert the object to height"""
        try:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = "zed_left_camera_optical_frame"
            stamped_point.header.stamp = self.get_clock().now().to_msg()
            stamped_point.point.x = detection.px
            stamped_point.point.y = detection.py
            stamped_point.point.z = detection.pz
            transform_message = PointTransformation.Request()
            transform_message.target_frame = "base_link"
            transform_message.point = stamped_point
            transform_frame = self.transform_tf.call_async(transform_message)
            rclpy.spin_until_future_complete(self, transform_frame)
            transformed_point = transform_frame.result()
            if not transformed_point.success:
                Logger.error(self, f"{transformed_point.error_message}")
                return None
            transformed_point = transformed_point.transformed_point
            Logger.info(self, f"Actual height: {transformed_point.point.z}")
            return transformed_point.point.z
        except Exception as e:
            Logger.error(self, f"Error converting to height: {e}")
            return None

    def get_new_height(self, bbox: BBOX) -> float:
        return bbox.pz + 0.1

    def exec_state(self):
        Logger.info(self, f"Executing state: {self.state.name}")
        if self.state == ExecutionStates.START:
            self.get_logger().info("Waiting for TF system to initialize...")
            rclpy.spin_once(self, timeout_sec=2.0)
            # self.state = ExecutionStates.INIT_NAV_TO_SHELF
            self.state = ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS

        elif self.state == ExecutionStates.END:
            Logger.info(self, "Ending Storing Groceries Manager...")
            self.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
            return
        elif self.state == ExecutionStates.INIT_NAV_TO_SHELF:
            hres: Status = self.nav_to("kitchen", "shelve")
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.SUCCEDED_NAV_TO_SHELF
            else:
                return
                self.state = ExecutionStates.FAILED_NAV_TO_SHELF
        elif self.state == ExecutionStates.SUCCEDED_NAV_TO_SHELF:
            self.state = ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS
        elif self.state == ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS:
            if not self.check_manual_levels:
                status, results = self.subtask_manager.vision.detect_shelf(timeout=10)
                results: list[ShelfDetection]
                self.shelves_count = max([i.level for i in results]) + (
                    1 if 0 in [i.level for i in results] > 0 else 0
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

                # for det in res:
                #     # Logger.info(self, f"Detected object: {det.classname}")
                #     detected_shelf: ShelfDetection = None
                #     center_x = det.x1 + det.w / 2
                #     center_y = det.y1 + det.h / 2
                #     for shelf in results:
                #         if shelf.x1 < center_x < shelf.x2 and shelf.y1 < center_y < shelf.y2:
                #             detected_shelf = shelf
                #             break
                #     if detected_shelf is None:
                #         Logger.error(self, f"Detected object {det.classname} not in any shelf")
                #         continue
                #     Logger.info(
                #         self, f"Detected object {det.classname} in shelf {detected_shelf.level}"
                #     )
                #     self.shelves[detected_shelf.level].id = detected_shelf.level
                #     self.shelves[detected_shelf.level].objects.append(det.classname)
                Logger.info(self, f"Shelves: {self.shelves}")
                self.state = ExecutionStates.INIT_NAV_TO_TABLE

                len(self.shelves)
            else:
                # Manual levels
                self.shelves_count = 0
                self.shelves = {}
                # for i, height in enumerate(self.manual_heights):
                #     self.shelves[i] = Shelf(id=i, tag="", objects=[])
                #     self.subtask_manager.manipulation.move_to_height(height)
                #     bbox = self.subtask_manager.manipulation.get_plane_bbox()
                #     new_target = self.get_new_height(bbox)
                #     self.subtask_manager.manipulation.move_to_height(new_target)
                #     status, res = self.subtask_manager.vision.detect_objects()

                #     for count, det in enumerate(res):
                #         if det is not None:
                #             self.shelves[i].objects.append(det.classname)
                #             self.shelves[i].id = i
                #             Logger.info(
                #                 self,
                #                 f"Detected object {det.classname} in shelf {self.shelves[i].tag}",
                #             )

                #     self.shelves_count += 1

                # self.subtask_manager.manipulation.move_joint_positions(
                # named_position="front_stare", velocity=0.5, degrees=True
                # )
                self.generate_manual_levels()
                for i in range(len(self.manual_heights)):
                    self.shelves[i] = Shelf(id=i, tag="", objects=[])
                    self.subtask_manager.manipulation.get_optimal_position_for_plane(
                        self.manual_heights[i], tolerance=0.2
                    )
                    status, res = self.subtask_manager.vision.detect_objects()
                    rettry = 0
                    while status != Status.EXECUTION_SUCCESS and rettry < 5:
                        Logger.error(self, f"Error detecting objects: {status}")
                        time.sleep(1)
                        status, res = self.subtask_manager.vision.detect_objects()
                        rettry += 1
                    if status != Status.EXECUTION_SUCCESS:
                        Logger.error(self, f"Error detecting objects: {status}")
                        return

                    if len(res) == 0:
                        Logger.error(self, f"No objects detected: {status}")
                        continue
                    for det in res:
                        if det is not None:
                            height = self.convert_to_height(det)
                            while height is None:
                                height = self.convert_to_height(det)
                            distance_check = height - self.manual_heights[i]
                            if (
                                distance_check < 0
                                and abs(distance_check) < self.shelf_level_down_threshold
                            ) or (
                                distance_check >= 0 and distance_check < self.shelf_level_threshold
                            ):
                                self.shelves[i].objects.append(det.classname)
                                self.shelves[i].id = i
                                Logger.info(
                                    self,
                                    f"Detected object {det.classname} in shelf {self.shelves[i].tag}",
                                )
                                break
                    self.shelves_count += 1
                # self.subtask_manager.manipulation.move_joint_positions(
                #     named_position="front_stare", velocity=0.5, degrees=True
                # )

                # status, res = self.subtask_manager.vision.detect_objects()
                # for count, det in enumerate(res):
                #     if det is not None:
                #         height = self.convert_to_height(det)
                #         while height is None:
                #             height = self.convert_to_height(det)
                #         for i in self.shelves:
                #             distance_check = height - self.manual_heights[i]
                #             if (
                #                 distance_check < 0
                #                 and abs(distance_check) < self.shelf_level_down_threshold
                #             ) or (
                #                 distance_check >= 0 and distance_check < self.shelf_level_threshold
                #             ):
                #                 self.shelves[i].objects.append(det.classname)
                #                 self.shelves[i].id = i
                #                 Logger.info(
                #                     self,
                #                     f"Detected object {det.classname} in shelf {self.shelves[i].tag}",
                #                 )
                #                 break

                if status == Status.EXECUTION_SUCCESS:
                    self.state = ExecutionStates.INIT_NAV_TO_TABLE
        elif self.state == ExecutionStates.INIT_NAV_TO_TABLE:
            hres: Status = self.nav_to("kitchen", "table")
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE
            else:
                # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                Logger.error(self, "Failed to navigate to table")
                return
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
            self.object_names_on_table = []
            self.object_names_on_table = [i.classname for i in result]
            Logger.info(self, f"Detected objects: {self.object_names_on_table}")
            self.object_to_placing_shelf = defaultdict(list)
            self.state = ExecutionStates.CATEGORIZE_OBJECTS
        elif self.state == ExecutionStates.CATEGORIZE_OBJECTS:
            missing_shelves = [i for i in range(self.shelves_count) if i not in self.shelves]
            for i in missing_shelves:
                self.shelves[i] = Shelf(id=i)
            shelfs: dict[int, list[str]] = {}
            for i in self.shelves:
                shelfs[i] = self.shelves[i].objects
            try:
                status, categorized_shelfs, objects_to_add = (
                    self.subtask_manager.hri.categorize_objects(self.object_names_on_table, shelfs)
                )
            except Exception as e:
                Logger.error(self, f"Error categorizing objects: {e}")
                return
            if not status == Status.EXECUTION_SUCCESS:
                Logger.error(self, "Failed to categorize objects")
                if self.retry_count > 5:
                    # self.state = ExecutionStates.END
                    self.state = ExecutionStates.CATEGORIZE_OBJECTS2
                    self.retry_count = 0
                    return
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
            # self.state = ExecutionStates.SAY_5_OBJECTS_CATEGORIZED
            self.state = ExecutionStates.SAY_5_OBJECTS_CATEGORIZED
            self.subtask_manager.hri.say(
                text=f"Categorized {len(self.object_names_on_table)} objects", wait=True
            )
        elif self.state == ExecutionStates.CATEGORIZE_OBJECTS2:
            for i in self.object_names_on_table:
                # random shelf
                self.object_to_placing_shelf[i].append(self.shelves_count % len(self.shelves))
                self.shelves[self.shelves_count % len(self.shelves)].objects.append(i)
                self.shelves[self.shelves_count % len(self.shelves)].tag = "random"
                self.shelves_count += 1
            # self.subtask_manager.hri.say(
            #     text=f"Categorized {len(self.object_names_on_table)} objects", wait=True
            # )
            # for i in self.object_names_on_table:
            #     self.subtask_manager.hri.say(
            #         text=f"Categorized object: {i} as {self.shelves[self.object_to_placing_shelf[i]].tag}",
            #         wait=True,
            #     )
            # self.state = ExecutionStates.PLAN_NEXT
            self.state = ExecutionStates.SAY_5_OBJECTS_CATEGORIZED
        elif self.state == ExecutionStates.SAY_5_OBJECTS_CATEGORIZED:
            for i in list(set(self.object_names_on_table))[
                : max(5, len(self.object_names_on_table))
            ]:
                try:
                    Logger.info(self, f"Categorized object: {i} as {self.shelves[i].tag}")
                    Logger.info(
                        self,
                        f"Categorized object: {i} as {self.shelves[self.object_to_placing_shelf[i]].tag}",
                    )
                    self.subtask_manager.hri.say(
                        text=f"Categorized object: {i} as {self.shelves[self.object_to_placing_shelf[i]].tag}",
                        wait=True,
                    )
                except Exception as e:
                    Logger.error(self, f"Error categorizing object: {e}")
                    continue
            self.state = ExecutionStates.PLAN_NEXT
        elif self.state == ExecutionStates.PLAN_NEXT:
            if len(self.object_names_on_table) == 0:
                self.state = ExecutionStates.END
                return
            self.state = ExecutionStates.NAV_TO_TABLE
        elif self.state == ExecutionStates.NAV_TO_TABLE:
            print("qpd papu")
            print(f"self.state: {self.state}")
            # return
            try:
                hres: Status = self.nav_to("kitchen", "table")
            except Exception as e:
                Logger.error(self, f"Error navigating to table: {e}")

            self.state = ExecutionStates.PICK_OBJECT
            # return
            # if not hres == Status.EXECUTION_SUCCESS:
            # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
            #   return
            time.sleep(1)

        elif self.state == ExecutionStates.PICK_OBJECT:
            status = self.subtask_manager.manipulation.get_optimal_position_for_plane(
                0.75, tolerance=0.2
            )
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
            hres: Status = self.subtask_manager.manipulation.pick_object(min_distance_obj.classname)
            if not hres == Status.EXECUTION_SUCCESS:
                # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                Logger.error(self, "Failed to pick object")
                return
            self.state = ExecutionStates.NAV_TO_SHELF
        elif self.state == ExecutionStates.NAV_TO_SHELF:
            hres: Status = self.nav_to("kitchen", "shelve")
            # if not hres == Status.EXECUTION_SUCCESS:
            #     # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
            #     return
            time.sleep(1)
            self.state = ExecutionStates.PLACE_OBJECT
        elif self.state == ExecutionStates.PLACE_OBJECT:
            # self.state = ExecutionStates.DEUX_PICK_OBJECT
            # return
            status = self.subtask_manager.manipulation.place(
                self.object_to_placing_shelf[self.current_object].pop(0), self.current_object
            )
            self.shelves[self.object_to_placing_shelf[self.current_object]].objects.append(
                self.current_object
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
        elif self.state == ExecutionStates.DEUX_PICK_OBJECT:
            Logger.info(self, "DEUX_PICK_OBJECT")
            status = self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say(
                f"Please hand me the {self.current_object} object", wait=True
            )
            # wait 2 seconds
            tries = 0
            while not self.subtask_manager.hri.confirm(
                "Have you handed me the object?", timeout=30
            ):
                self.subtask_manager.hri.say("Please hand me the object when ready.")
                tries += 1
                if tries > 5:
                    Logger.error(self, "Failed to pick object")
                    return
            status = self.subtask_manager.manipulation.close_gripper()
            self.state = ExecutionStates.NAV_TO_SHELF
        elif self.state == ExecutionStates.DEUX_PLACE_OBJECT:
            Logger.info(self, "DEUX_PLACE_OBJECT")
            shelf = self.object_to_placing_shelf[self.current_object][0]
            self.subtask_manager.hri.say(
                f"Please place the {self.current_object} object in the shelf number {shelf}",
                wait=True,
            )
            self.subtask_manager.hri.say("Other objects in the shelf are:")
            for i in self.shelves[shelf].objects:
                self.subtask_manager.hri.say(f"{i}", wait=True)
            tries = 0
            self.subtask_manager.manipulation.open_gripper()
            while not self.subtask_manager.hri.confirm("Have you placed the object?", timeout=30):
                self.subtask_manager.hri.say("Please place the object when ready.")
                tries += 1
                if tries > 5:
                    Logger.error(self, "Failed to place object")
                    return
            status = self.subtask_manager.manipulation.close_gripper()
            self.shelves[shelf].objects.append(self.current_object)
            self.object_to_placing_shelf[self.current_object].pop(0)
            self.state = ExecutionStates.PLAN_NEXT
        else:
            Logger.error(self, f"Unknown state: {self.state}")
            return

    def run(self):
        Logger.info(self, "Starting Storing Groceries Manager...")
        if self.state != ExecutionStates.END:
            if self.prev_state == self.state:
                Logger.info(self, f"State: {self.state.name}")
                Logger.info(self, f"Retry count: {self.retry_count}")
                Logger.info(self, f"State data: {self.state_data}")
                Logger.info(self, f"State: {self.state.name}")
                Logger.info(self, f"Retry count: {self.retry_count}")
                Logger.info(self, f"Prev: {self.prev_state}")
                self.retry_count += 1
                if self.retry_count > 10:
                    Logger.error(self, "Retry count exceeded")
                    if self.state in STATE_TO_DEUX:
                        self.prev_state = self.state
                        self.state = STATE_TO_DEUX[self.state]
                        self.retry_count = 0
                    else:
                        self.state = ExecutionStates.END
                        return ExecutionStates.END
            else:
                self.retry_count = 0
                self.prev_state = self.state
            try:
                self.exec_state()
            except Exception as e:
                # self.logger.error("Error executing state: %s", e)
                Logger.error(self, f"Error executing state: {str(e)}")
            if self.state == ExecutionStates.END:
                Logger.info(self, "Ending Storing Groceries Manager...")
                self.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
                return ExecutionStates.END
        Logger.info(self, "Starting Storing Groceries Manager...")


def main(args=None):
    """Main function"""
    print("Starting Storing Groceries Manager...")
    rclpy.init(args=args)
    node = StoringGroceriesManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.run() == ExecutionStates.END:
                break
        node.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

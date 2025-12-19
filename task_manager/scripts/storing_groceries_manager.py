#!/usr/bin/env python3

import random
import time
from collections import defaultdict
from enum import Enum

import rclpy
from frida_constants.hri_constants import DISPLAY_VIEW_TOPIC
from frida_constants.vision_classes import BBOX
from frida_interfaces.srv import PointTransformation
from geometry_msgs.msg import PointStamped, Twist
from pydantic import BaseModel
from rclpy.node import Node
from std_msgs.msg import String
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"
ATTEMPT_LIMIT = 5
# after this amount of objects have been stored, do pour task
PICKED_OBJECTS_TO_POUR = 5
MAX_SHELF_HEIGHT = 1.2


class Retries(Enum):
    DEFAULT = 10
    NAVIGATION = 5


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

    POUR_OBJECT = 101
    CEREAL_ANALYSIS = 100
    CEREAL_PICK = 110
    CEREAL_PLACE = 120

    NOT_ALL_OBJECTS_PICKED = 130

    END = 200


STATE_TO_DEUX = {
    ExecutionStates.START: ExecutionStates.INIT_NAV_TO_SHELF,
    ExecutionStates.SUCCEDED_NAV_TO_SHELF: ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS,
    ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS: ExecutionStates.INIT_NAV_TO_TABLE,
    ExecutionStates.PICK_OBJECT: ExecutionStates.DEUX_PICK_OBJECT,
    ExecutionStates.PLACE_OBJECT: ExecutionStates.DEUX_PLACE_OBJECT,
    ExecutionStates.INIT_NAV_TO_SHELF: ExecutionStates.SUCCEDED_NAV_TO_SHELF,
    ExecutionStates.INIT_NAV_TO_TABLE: ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE,
    ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE: ExecutionStates.PLAN_NEXT,
    ExecutionStates.CATEGORIZE_OBJECTS: ExecutionStates.CATEGORIZE_OBJECTS2,
    ExecutionStates.SAY_5_OBJECTS_CATEGORIZED: ExecutionStates.PLAN_NEXT,
    ExecutionStates.PLAN_NEXT: ExecutionStates.NAV_TO_TABLE,
    ExecutionStates.NAV_TO_SHELF: ExecutionStates.PLACE_OBJECT,
    ExecutionStates.NAV_TO_TABLE: ExecutionStates.PICK_OBJECT,
    ExecutionStates.NOT_ALL_OBJECTS_PICKED: ExecutionStates.PICK_OBJECT,
}

STATE_RETRIES = {
    ExecutionStates.INIT_NAV_TO_SHELF: 5,
    ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS: 2,
    ExecutionStates.INIT_NAV_TO_TABLE: 5,
    ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE: 5,
    ExecutionStates.CATEGORIZE_OBJECTS: 5,
    ExecutionStates.SAY_5_OBJECTS_CATEGORIZED: 2,
    ExecutionStates.NAV_TO_TABLE: 10,
    ExecutionStates.NAV_TO_SHELF: 10,
    ExecutionStates.PICK_OBJECT: 2,
    ExecutionStates.PLACE_OBJECT: 1,
    ExecutionStates.CEREAL_ANALYSIS: 10,
    ExecutionStates.CEREAL_PICK: 10,
    ExecutionStates.CEREAL_PLACE: 10,
    "DEFAULT": 10,
}

for i in ExecutionStates:
    for j in ExecutionStates:
        if i.value == j.value and i.name != j.name:
            raise ValueError(f"Duplicate value found: {i.value} for {i.name} and {j.name}")


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
        self.object_to_placing_shelf: dict[str, list[int]] = defaultdict(list)
        self.current_object: str = None
        self.point_pub = self.create_publisher(PointStamped, "point_visualize", 10)
        self.retry_count = 0
        self.prev_state = None
        self.pour_objects = [
            "cornflakes",
            "transparent_box",
            # "blue_cereal",
            # "cup",
            # "spam_tuna",
            # "tuna_can",
            # "tuna",  # ignore this bc of false detects
        ]
        self.poured_object = False
        # self.manual_heights = [  # 0.2
        #     0.45,  # 0.45 +- 0.2 -> 0.25 0.65
        #     0.8,  # 0.8 +- 0.2 -> 0.6 1.0
        #     1.17,  # 1.17 +- 0.2 -> 0.97 1.37
        #     # 1.525,  # 1.525 (0.1 +-) -> 1.425 1.625
        # ]  # remember rest 15cm from the base_link and the measure is in m
        # self.manual_heights = [
        #     0.077,
        #     0.463,
        #     0.84
        # ]
        # self.manual_heights = [0.55, 0.9]
        # self.manual_heights = [0.36, 0.87, 1.38]
        self.manual_heights = [0.4, 0.70, 1.05]
        self.shelf_level_threshold = 0.20

        #         self.manual_heights = [0.04, 0.43, 0.67]
        #         self.shelf_level_threshold = 0.30

        self.poseeeeensahsajsajasjhasjha = None
        self.es_primer_vez = True
        self.shelf_level_down_threshold = 0.05
        self.picked_objects = 0
        self.prev_uid = None
        self.pick_uid = None
        self.transform_tf = self.create_client(PointTransformation, POINT_TRANSFORMER_TOPIC)
        self.transform_tf = self.create_client(PointTransformation, POINT_TRANSFORMER_TOPIC)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.display_view_pub = self.create_publisher(String, DISPLAY_VIEW_TOPIC, 10)

    def generate_manual_levels(self):
        """Generate manual levels"""
        self.shelves_count = 0
        self.shelves = {}
        for i in range(len(self.manual_heights)):
            self.shelves[i] = Shelf(id=i, tag="", objects=[])
            self.shelves_count += 1

    def nav_to(self, location: str, sub_location: str = "", say: bool = True) -> Status:
        Logger.info(self, f"Navigating to {location} {sub_location} ")
        try:
            if say:
                self.subtask_manager.hri.say(text=f"Going to {location} {sub_location}", wait=False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.subtask_manager.nav.resume_nav()
            result = Status.EXECUTION_ERROR
            retry = 0
            while result == Status.EXECUTION_ERROR and retry < Retries.NAVIGATION.value:
                future = self.subtask_manager.nav.move_to_location(location, sub_location)
                if "navigation" not in self.subtask_manager.get_mocked_areas():
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()
                else:
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()
                    if result.success:
                        result = Status.EXECUTION_SUCCESS
                retry += 1
            self.subtask_manager.nav.pause_nav()
            return result
        except Exception as e:
            Logger.error(self, f"Error navigating to {location}: {e}")
            self.subtask_manager.nav.pause_nav()
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
            Logger.info(self, "Starting Storing Groceries Manager...")
            self.subtask_manager.hri.say("Press the button to start")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            
            msg = String()
            msg.data = "store_groceries"
            self.display_view_pub.publish(msg)
            
            self.subtask_manager.hri.say(text="Waiting for door to open")
            self.state = ExecutionStates.INIT_NAV_TO_SHELF
            # self.state = ExecutionStates.INIT_NAV_TO_TABLE
            # self.state = ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE

            # self.state = ExecutionStates.INIT_NAV_TO_SHELF
            # self.state = ExecutionStates.CATEGORIZE_OBJECTS

        elif self.state == ExecutionStates.END:
            Logger.info(self, "Ending Storing Groceries Manager...")
            self.subtask_manager.hri.say(text="Ending Storing Groceries Manager...", wait=True)
            return
        elif self.state == ExecutionStates.INIT_NAV_TO_SHELF:
            res = "closed"
            while res == "closed":
                time.sleep(1)
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")

            Logger.info(self, "Door OPENED GOING TO NEXT STAT")
            # hres: Status = self.nav_to("kitchen", "shelve")
            self.subtask_manager.hri.say(
                text="Hello, Can you please help me opening the shelve cabinet for me?"
            )
            self.subtask_manager.hri.confirm(
                "Can you please open the shelve cabinet for me? Please say yes when done!",
                use_hotwords=True,
            )
            hres: Status = self.nav_to("kitchen", "shelve")
            # hres: Status = self.nav_to("kitchen", "shelve", say=False)
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.SUCCEDED_NAV_TO_SHELF
            else:
                Logger.error(self, f"Failed to navigate to shelf {hres}")
                return

        elif self.state == ExecutionStates.SUCCEDED_NAV_TO_SHELF:
            self.state = ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS
        elif self.state == ExecutionStates.VIEW_SHELF_AND_SAVE_OBJECTS:
            # Manual levels
            self.shelves_count = 0
            self.shelves = defaultdict(Shelf)
            self.generate_manual_levels()
            for i in range(5):
                s, detections = self.subtask_manager.vision.detect_objects()
                if len(detections) > 0:
                    break
                self.timeout(1)
            for i in range(len(self.manual_heights)):
                self.subtask_manager.hri.say(text=f"Detecting shelf number {i}", wait=False)
                if i not in self.shelves:
                    self.shelves[i] = Shelf(id=i, tag="", objects=[])
                Logger.info(self, f"Moving to height {self.manual_heights[i]}")
                self.subtask_manager.manipulation.get_optimal_position_for_plane(
                    self.manual_heights[i],
                    tolerance=0.1,
                    table_or_shelf=False,
                    approach_plane=True,
                )
                Logger.info(self, "Moved, now detecting objects")
                time.sleep(3)
                status, res = self.subtask_manager.vision.detect_objects(
                    ignore_labels=self.pour_objects
                )
                rettry = 0
                while status != Status.EXECUTION_SUCCESS and rettry < 5:
                    Logger.error(self, f"Error detecting objects: {status}")
                    time.sleep(1)
                    status, res = self.subtask_manager.vision.detect_objects(
                        ignore_labels=self.pour_objects
                    )
                    rettry += 1
                if status != Status.EXECUTION_SUCCESS:
                    Logger.error(self, f"Error detecting objects: {status}")
                    return

                if len(res) == 0:
                    Logger.error(self, f"No objects detected: {status}")
                    continue
                for det in res:
                    Logger.info(self, f"Detected object: {det.classname}, projecting...")
                    if det is not None:
                        height = self.convert_to_height(det)
                        while height is None:
                            height = self.convert_to_height(det)
                        distance_check = height - self.manual_heights[i]
                        if (
                            distance_check < 0
                            and abs(distance_check) < self.shelf_level_down_threshold
                        ) or (distance_check >= 0 and distance_check < self.shelf_level_threshold):
                            self.shelves[i].objects.append(det.classname)
                            self.shelves[i].id = i
                            Logger.info(
                                self,
                                f"Detected object {det.classname} in shelf {self.shelves[i].tag}",
                            )
                            break
                        else:
                            for j in range(len(self.manual_heights)):
                                distance_check = height - self.manual_heights[j]
                                if (
                                    distance_check < 0
                                    and abs(distance_check) < self.shelf_level_down_threshold
                                ) or (
                                    distance_check >= 0
                                    and distance_check < self.shelf_level_threshold
                                ):
                                    if j != i and j not in self.shelves[j].objects:
                                        self.shelves[j].objects.append(det.classname)
                                        self.shelves[j].id = j
                                        Logger.info(
                                            self,
                                            f"Detected object {det.classname} in shelf {self.shelves[j].tag}",
                                        )
                                        break
                                # if j != i and j not in self.shelves[j].objects:
                                #     self.shelves[j].objects.append(det.classname)
                                #     self.shelves[j].id = j
                                #     Logger.info(
                                #         self,
                                #         f"Detected object {det.classname} in shelf {self.shelves[j].tag}",
                                #     )
                                #     break
                # self.shelves_count += 1

            Logger.info(self, f"Shelves: {self.shelves}")
            Logger.info(self, f"Objects: {[i.objects for i in self.shelves.values()]}")

            # self.subtask_manager.manipulation.move_joint_positions(
            #     named_position="front_stare", velocity=0.5, degrees=True
            # )

            # status, res = self.subtask_manager.vision.detect_objects(ignore_labels=self.pour_objects)
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
            self.state = ExecutionStates.INIT_NAV_TO_TABLE
            # if status = Status.EXECUTION_SUCCESS

            # return

        elif self.state == ExecutionStates.INIT_NAV_TO_TABLE:
            hres: Status = self.nav_to("kitchen", "table")
            hres = Status.EXECUTION_SUCCESS
            time.sleep(2)
            self.subtask_manager.hri.say(text="Finding Table", wait=False)
            pose = self.subtask_manager.manipulation.get_optimal_pose_for_plane(
                0.4, tolerance=0.3, projected_distance=0.35
            )
            if pose is None:
                Logger.warn(self, "Pose not found couldn't navigate to table")
                if self.poseeeeensahsajsajasjhasjha is not None:
                    future = self.subtask_manager.nav.move_to_pose(pose)
                    rclpy.spin_until_future_complete(self, future)
                    Logger.info(self, f"Pose navigation result: {future.result()}")
                else:
                    for i in range(10):
                        pose = self.subtask_manager.manipulation.get_optimal_pose_for_plane(
                            0.4, tolerance=0.3, projected_distance=0.35
                        )
                        if pose is not None:
                            break
                        else:
                            twist_msg = Twist()
                            start_time = time.time()
                            while time.time() - start_time < 2.0:
                                twist_msg.linear.x = 0.075
                                self.cmd_vel_pub.publish(twist_msg)
                                time.sleep(0.001)
                            self.cmd_vel_pub.publish(Twist())
                            time.sleep(2)

                    if pose is None:
                        Logger.error(self, "Pose not found, cannot navigate to table")
                        return
                    future = self.subtask_manager.nav.move_to_pose(pose)
                    rclpy.spin_until_future_complete(self, future)
                    Logger.info(self, f"Pose navigation result: {future.result()}")
            else:
                self.poseeeeensahsajsajasjhasjha = pose
                future = self.subtask_manager.nav.move_to_pose(pose)
                rclpy.spin_until_future_complete(self, future)
                Logger.info(self, f"Pose navigation result: {future.result()}")
            # hres = Status.EXECUTION_SUCCESS
            # hres: Status = self.nav_to("kitchen", "table", say=False)
            if hres == Status.EXECUTION_SUCCESS:
                self.state = ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE
            else:
                # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                Logger.error(self, "Failed to navigate to table")
                return
        elif self.state == ExecutionStates.VIEW_AND_SAVE_OBJECTS_ON_TABLE:
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            time.sleep(1.5)
            status, result = self.subtask_manager.vision.detect_objects(
                timeout=10, ignore_labels=self.pour_objects
            )
            if status == Status.TIMEOUT:
                # pass
                return
            elif status == Status.EXECUTION_SUCCESS:
                pass
            else:
                Logger.error(self, "Unknown status")
                return
            if len(result) == 0:
                Logger.error(self, "No objects detected")
                self.subtask_manager.hri.say(text="No objects detected", wait=False)
                return
            Logger.info(self, f"Detected objects: {result}")
            result: list[BBOX]
            self.objects_on_table = result
            self.object_names_on_table = []
            max_distance_from_camera = 1.2  # 1.2 meters
            self.object_names_on_table = [
                i.classname for i in result if i.distance < max_distance_from_camera
            ]
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
                # self.object_names_on_table = ["apple", "squash", "coke", "bowl"]
                # shelfs = {0: [], 1: ["water", "sprite_can"], 2: ["squash", "orange", "banana"]}
                # self.object_names_on_table = ["apple", "fresca_can", "bowl", "yellow_bowl", "pringles"]
                # shelfs = {0: [], 1: ["bottle", "coke_can"], 2: ["pear"]}
                status, categorized_shelfs, objects_to_add, resulting_array = (
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
            for i in list(set([str(k) for k in self.object_names_on_table]))[
                : min(5, len(self.object_names_on_table))
            ]:
                try:
                    # Logger.info(self, f"Categorized object: {i} as {self.shelves[i].tag}")
                    asdasdajfjfjfjfjfj = self.subtask_manager.hri.deterministic_categorization(i)
                    Logger.info(
                        self,
                        f"Categorized object: {i} as {asdasdajfjfjfjfjfj} going in shelf of {self.shelves[self.object_to_placing_shelf[i][0]].tag}",
                    )
                    self.subtask_manager.hri.say(
                        text=f"Categorized object: {i} as {asdasdajfjfjfjfjfj} going in shelf of {self.shelves[self.object_to_placing_shelf[i][0]].tag}",
                        wait=False,
                    )
                except Exception as e:
                    Logger.error(self, f"Error categorizing object: {e}")
                    continue
            self.state = ExecutionStates.PLAN_NEXT

        elif self.state == ExecutionStates.PLAN_NEXT:
            if len(self.object_names_on_table) == 0 or self.picked_objects >= 5:
                self.subtask_manager.hri.say(text="I have finished placing the objects", wait=True)
                self.state = ExecutionStates.NAV_TO_TABLE
                return
            self.pick_uid = random.random()
            if self.prev_uid != self.pick_uid:
                self.picked_objects += 1
                self.prev_uid = self.pick_uid
            self.state = ExecutionStates.NAV_TO_TABLE

        elif self.state == ExecutionStates.NOT_ALL_OBJECTS_PICKED:
            if len([i for k, i in self.object_to_placing_shelf.values() if k == 0]) == 0:
                status, res = self.subtask_manager.hri.confirm(
                    text="Did I pick all the objects?", use_hotwords=False
                )
                if res == "yes":
                    self.subtask_manager.hri.say(
                        text="I have finished placing the objects", wait=True
                    )
                    self.state = ExecutionStates.END
                    return
                else:
                    self.subtask_manager.hri.say(text="I will pick the objects again", wait=True)
                    self.state = ExecutionStates.PICK_OBJECT
                    return

            # if nothing works then go to deux: ask the user if there are more objects to pick
            status, res = self.subtask_manager.hri.confirm(
                text="Did I pick all the objects?", use_hotwords=False
            )
            if res == "yes":
                self.subtask_manager.hri.say(text="I have finished placing the objects", wait=True)
                self.state = ExecutionStates.END
                return
            else:
                self.subtask_manager.hri.say(text="I will pick the objects again", wait=True)
                self.state = ExecutionStates.PICK_OBJECT
                return

        elif self.state == ExecutionStates.NAV_TO_TABLE:
            print("qpd papu")
            print(f"self.papustate: {self.state}")
            if self.es_primer_vez:
                self.state = ExecutionStates.PICK_OBJECT
                self.es_primer_vez = False
                return
            # return
            try:
                # hres: Status = self.nav_to("kitchen", "table")
                # self.subtask_manager.hri.say(
                #     text="Can you please help me opening the shelve cabinet for me?"
                # )
                # self.subtask_manager.hri.confirm("Can you please open the shelve cabinet for me? Please say yes when done!", use_hotwords=True)
                hres: Status = self.nav_to("kitchen", "table")
                hres = Status.EXECUTION_SUCCESS
                time.sleep(2)
                self.subtask_manager.hri.say(text="Finding Table", wait=False)

                pose = self.subtask_manager.manipulation.get_optimal_pose_for_plane(
                    0.4, tolerance=0.3, projected_distance=0.35
                )
                if pose is None:
                    Logger.warn(self, "Pose not found couldn't navigate to table")
                    if self.poseeeeensahsajsajasjhasjha is not None:
                        future = self.subtask_manager.nav.move_to_pose(pose)
                        rclpy.spin_until_future_complete(self, future)
                        Logger.info(self, f"Pose navigation result: {future.result()}")
                    else:
                        for i in range(10):
                            pose = self.subtask_manager.manipulation.get_optimal_pose_for_plane(
                                0.4, tolerance=0.3, projected_distance=0.35
                            )
                            if pose is not None:
                                break
                            else:
                                twist_msg = Twist()
                                start_time = time.time()
                                while time.time() - start_time < 2.0:
                                    twist_msg.linear.x = 0.075
                                    self.cmd_vel_pub.publish(twist_msg)
                                    time.sleep(0.001)
                                self.cmd_vel_pub.publish(Twist())
                                time.sleep(2)
                            # else:
                            #     twist_msg = Twist()
                            #     twist_msg.linear.x = 0.2
                            #     self.cmd_vel_pub.publish(twist_msg)
                            #     time.sleep(2)
                            #     self.cmd_vel_pub.publish(Twist())
                            #     time.sleep(2)

                        if pose is None:
                            Logger.error(self, "Pose not found, cannot navigate to table")
                            return
                        future = self.subtask_manager.nav.move_to_pose(pose)
                        rclpy.spin_until_future_complete(self, future)
                        Logger.info(self, f"Pose navigation result: {future.result()}")
                else:
                    future = self.subtask_manager.nav.move_to_pose(pose)
                    rclpy.spin_until_future_complete(self, future)
                    Logger.info(self, f"Pose navigation result: {future.result()}")

                # hres: Status = self.nav_to("kitchen", "table", say=False)
            except Exception as e:
                Logger.error(self, f"Error navigating to table: {e}")

            if False and self.picked_objects > PICKED_OBJECTS_TO_POUR and not self.poured_object:
                self.state = ExecutionStates.POUR_OBJECT
            else:
                self.state = ExecutionStates.PICK_OBJECT
            Logger.info(self, f"State changed to: {self.state.name}")
            # return
            # if not hres == Status.EXECUTION_SUCCESS:
            # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
            #   return
            time.sleep(1)
            return

        elif self.state == ExecutionStates.PICK_OBJECT:
            # status = self.subtask_manager.manipulation.get_optimal_position_for_plane(
            #     0.75, tolerance=0.2, table_or_shelf=True
            # )
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            time.sleep(2.5)
            status, objs = self.subtask_manager.vision.detect_objects(
                timeout=10, ignore_labels=self.pour_objects
            )
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
                self.subtask_manager.hri.say(text="No objects detected", wait=False)
                return
            Logger.info(self, f"Detected objects: {objs}")
            min_distance = 1000000000
            min_distance_obj: BBOX = None
            tried_objects = []
            for INDEX_ASD in range(3):
                for obj in objs:
                    if obj.distance < min_distance and obj.classname not in tried_objects:
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
                    text=f"I'm going to pick the object: {min_distance_obj.classname}", wait=False
                )
                tried_objects.append(min_distance_obj.classname)
                self.current_object = min_distance_obj.classname
                hres: Status = self.subtask_manager.manipulation.pick_object(
                    min_distance_obj.classname
                )
                if not hres == Status.EXECUTION_SUCCESS:
                    # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                    Logger.error(self, "Failed to pick object")
                    return
                elif hres == Status.EXECUTION_SUCCESS:
                    break

            Logger.info(
                self,
                f"Picked object: {min_distance_obj.classname} at distance {min_distance_obj.distance}",
            )
            self.state = ExecutionStates.NAV_TO_SHELF
            time.sleep(2)
            status, new_objs = self.subtask_manager.vision.detect_objects(
                timeout=10, ignore_labels=self.pour_objects
            )
            new_objs: list[BBOX]
            if status == Status.TIMEOUT:
                # pass
                return
            elif not status == Status.EXECUTION_SUCCESS:
                Logger.error(self, "Unknown status")
                return
            if len(new_objs) == 0:
                Logger.error(self, "No objects detected")
                # self.subtask_manager.hri.say(text="No objects detected", wait=True)
                return
            Logger.info(self, f"Detected objects: {new_objs}")

            # for each object check if classname is same and if less than 0.05 then Logger.info
            v_obj = [min_distance_obj.px, min_distance_obj.py, min_distance_obj.pz]
            for obj in new_objs:
                obj: BBOX
                if obj.classname == min_distance_obj.classname:
                    v = [obj.px, obj.py, obj.pz]
                    distance = (
                        (v[0] - v_obj[0]) ** 2 + (v[1] - v_obj[1]) ** 2 + (v[2] - v_obj[2]) ** 2
                    ) ** 0.5
                    if distance < 0.09:
                        Logger.info(
                            self,
                            f"Detected object: {obj.classname} at distance {distance}",
                        )
                        Logger.error(self, "Didnt pick the object, detected it again")
                        self.state = ExecutionStates.PICK_OBJECT
                        return
                        # self.subtask_manager.hri.say(
                        #     text=f"Detected object: {obj.classname} at distance {distance}", wait=True
                        # )

        elif self.state == ExecutionStates.NAV_TO_SHELF:
            # hres: Status = self.nav_to("kitchen", "shelve")
            hres: Status = self.nav_to("kitchen", "shelve")
            # hres: Status = self.nav_to("kitchen", "shelve", say=False)
            # if not hres == Status.EXECUTION_SUCCESS:
            #     # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
            #     return
            time.sleep(1)
            self.state = ExecutionStates.PLACE_OBJECT

        elif self.state == ExecutionStates.PLACE_OBJECT:
            # self.state = ExecutionStates.DEUX_PICK_OBJECT
            # return
            if len(self.object_to_placing_shelf[self.current_object]) == 0:
                # put it in a random shelf
                Logger.info(self, "No shelf found for object")
                # self.object_to_placing_shelf[self.current_object].append(
                #     self.shelves_count % len(self.manual_heights)
                # )
                # status, categorized_shelfs, objects_to_add
                status, resulting_clas, objects_to_add_2, resulting_array = (
                    self.subtask_manager.hri.categorize_objects(
                        table_objects=[self.current_object],
                        shelves={i: self.shelves[i].objects for i in range(self.shelves_count)},
                    )
                )
                if not status == Status.EXECUTION_SUCCESS:
                    Logger.error(self, "Failed to categorize objects")
                    return
                # resulting_clas: dict[int, str]
                # objects_to_add_2: dict[int, list[str]]
                shelf_to_be_placed = None
                for shelf_idx, objects in objects_to_add_2.items():
                    Logger.info(self, f"Object to add: {objects}")
                    for obj in objects:
                        if obj == self.current_object:
                            shelf_to_be_placed = shelf_idx
                            self.object_to_placing_shelf[self.current_object].append(
                                shelf_to_be_placed
                            )

                            obj_cat = self.subtask_manager.hri.deterministic_categorization(
                                self.current_object
                            )

                            self.subtask_manager.hri.say(
                                f"I have classified the object: {self.current_object} as {obj_cat}, im going to place it in shelf number {shelf_to_be_placed} corresponging to {resulting_clas}",
                                wait=True,
                            )
                            break
                    if shelf_to_be_placed is not None:
                        Logger.success(
                            self,
                            f"Object {self.current_object} will be placed in shelf {shelf_to_be_placed}",
                        )
                        break
                if shelf_to_be_placed is None:
                    Logger.error(self, "Failed to categorize object")

            elif self.object_to_placing_shelf[self.current_object][0] > len(self.shelves):
                self.object_to_placing_shelf[self.current_object][0] = self.object_to_placing_shelf[
                    self.current_object
                ][0] % len(self.manual_heights)

            shelf = self.object_to_placing_shelf[self.current_object][0] % len(self.manual_heights)

            # status, detections = self.subtask_manager.vision.detect_objects(ignore_labels=self.pour_objects)
            # objsasdq_place = True
            # rettry = 0
            # while status != Status.EXECUTION_SUCCESS and rettry < 5:
            #     Logger.error(self, f"Error detecting objects: {status}")
            #     time.sleep(1)
            #     status, res = self.subtask_manager.vision.detect_objects(
            #         ignore_labels=self.pour_objects
            #     )
            #     rettry += 1
            # if status != Status.EXECUTION_SUCCESS or len(res) == 0:
            #     objsasdq_place = False

            obj_cat = self.subtask_manager.hri.deterministic_categorization(self.current_object)

            self.subtask_manager.hri.say(
                text=f"I'm going to place the object: {self.current_object} in shelf number {shelf}"
                + f". corresponding to {obj_cat}. Shelf contains categories: {self.shelves[shelf].tag if self.shelves[shelf].tag is not None else ''}"
                if self.shelves[shelf].tag != "" or self.shelves[shelf].tag != "random"
                else "",
                wait=False,
            )

            shelf_height = self.manual_heights[shelf]

            if shelf_height > MAX_SHELF_HEIGHT:
                self.state = ExecutionStates.DEUX_PLACE_OBJECT
                return

            self.subtask_manager.manipulation.get_optimal_position_for_plane(
                shelf_height, tolerance=0.1, table_or_shelf=False, approach_plane=False
            )

            # objs = self.subtask_manager.vision.detect_objects()
            # for i in

            status = self.subtask_manager.manipulation.place_on_shelf(
                plane_height=shelf_height, tolerance=0.1
            )

            # status = self.subtask_manager.manipulation.place(
            #     self.object_to_placing_shelf[self.current_object].pop(0), self.current_object
            # )
            # self.shelves[self.object_to_placing_shelf[self.current_object]].objects.append(
            #     self.current_object
            # )
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
            while (
                self.subtask_manager.hri.confirm(
                    "Have you handed me the object?", use_hotwords=False
                )[1]
                != "yes"
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
            tries = 0
            if (
                self.current_object not in self.object_to_placing_shelf
                or len(self.object_to_placing_shelf[self.current_object]) == 0
            ):
                self.subtask_manager.hri.say(
                    f"Please place the {self.current_object} object", wait=True
                )
                while (
                    self.subtask_manager.hri.confirm(
                        "Have you grabbed the object?", use_hotwords=False
                    )[1]
                    != "yes"
                ):
                    self.subtask_manager.hri.say("Please place the object when ready.")
                    tries += 1
                    if tries > 5:
                        Logger.error(self, "Failed to place object")
                        return
                self.subtask_manager.hri.say("Please place the object when ready.")
                tries += 1
                if tries > 5:
                    Logger.error(self, "Failed to place object")
                    return
                self.state = ExecutionStates.PLAN_NEXT
                return
            shelf = self.object_to_placing_shelf[self.current_object][0]
            self.subtask_manager.hri.say(
                f"Please place the {self.current_object} object in the shelf number {shelf}",
                wait=True,
            )
            self.subtask_manager.hri.say(
                f"Other objects in the shelf are: {self.shelves[shelf].objects}", wait=True
            )
            Logger.info(self, f"Other objects in the shelf are: {self.shelves[shelf].objects}")
            tries = 0

            self.subtask_manager.hri.confirm(
                "Have you grabbed the object?", use_hotwords=False, retries=5
            )
            self.subtask_manager.manipulation.open_gripper()
            while (
                self.subtask_manager.hri.confirm("Have you placed the object?", use_hotwords=False)[
                    1
                ]
                != "yes"
            ):
                self.subtask_manager.hri.say("Please place the object when ready.", wait=False)
                tries += 1
                if tries > 5:
                    Logger.error(self, "Failed to place object")
                    return
            # status = self.subtask_manager.manipulation.close_gripper()
            self.shelves[shelf].objects.append(self.current_object)
            self.object_to_placing_shelf[self.current_object].pop(0)
            self.state = ExecutionStates.PLAN_NEXT

        elif self.state == ExecutionStates.POUR_OBJECT:
            # status = self.subtask_manager.manipulation.get_optimal_position_for_plane(
            #     0.75, tolerance=0.2, table_or_shelf=True
            # )
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            time.sleep(2.5)
            for i in range(3):
                # Logger.info(self,
                #     f"Detected object: {min_distance_obj.classname}")
                self.subtask_manager.hri.say(
                    text=f"I'm going to pour the object: {self.pour_objects[0]} in {self.pour_objects[1]}",
                    wait=False,
                )
                try:
                    hres: Status = self.subtask_manager.manipulation.pour(
                        self.pour_objects[0], self.pour_objects[1]
                    )
                except Exception as e:
                    Logger.error(self, f"Error pouring object: {e}")
                    # hres = Status.EXECUTION_ERROR
                    self.poured_object = False
                    return
                if not hres == Status.EXECUTION_SUCCESS:
                    # self.state = ExecutionStates.FAILED_NAV_TO_SHELF
                    Logger.error(self, "Failed to pour object")
                    self.poured_object = False
                    # self.state = ExecutionStates.PICK_OBJECT
                    return
                elif hres == Status.EXECUTION_SUCCESS:
                    Logger.info(self, "Successfully poured object")
                    self.poured_object = True
                    self.state = ExecutionStates.PICK_OBJECT
                    break
            Logger.info(
                self,
                f"Done pouring object: {self.pour_objects[0]} in {self.pour_objects[1]}",
            )
            status = self.subtask_manager.manipulation.place()
            self.poured_object = True
            self.state = ExecutionStates.PICK_OBJECT

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
                retries_val = 0
                if self.state in STATE_RETRIES:
                    retries_val = STATE_RETRIES[self.state]
                else:
                    retries_val = STATE_RETRIES["DEFAULT"]
                if self.retry_count > retries_val:
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

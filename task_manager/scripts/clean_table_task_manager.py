#!/usr/bin/env python3

"""
Task Manager for Clean the Table
Robocup @Home 2025

Required nav locations:
- kitchen table
- kitchen trashbin
-kitchen dishwasher

"""

import rclpy
from rclpy.node import Node
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task
from utils.status import Status
import time
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

ATTEMPT_LIMIT = 3


class CleanTableTM(Node):
    """Class to manage the clean the table task."""

    class TaskStates:
        """Class to manage task states."""

        START = "START"
        WAIT_FOR_DOOR = "WAIT_FOR_DOOR"
        DETECT_OBJECTS = "DETECT_OBJECTS"
        PICK_OBJECT = "PICK_OBJECT"
        PLACE_OBJECT = "PLACE_OBJECT"
        NAVIGATE_TO_DROPOFF = "NAVIGATE_TO_PLACE_LOCATION"
        NAVIGATE_TO_TABLE = "NAVIGATE_TO_TABLE"
        END = "END"

    def __init__(self):
        """Initialize the node."""
        super().__init__("clean_table_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.CLEAN_TABLE)
        self.current_state = CleanTableTM.TaskStates.START
        self.running_task = True
        self.current_attempt = 0

        self.pick_objects = ["drink", "drink", "cup", "bowl", "spoon", "fork"]
        self.object_index = 0

        self.trash_place = PointStamped(
            header=Header(stamp=Time(sec=1752174502, nanosec=834552352), frame_id="map"),
            point=Point(x=2.2094616889953613, y=-2.8220369815826416, z=0.0023679733276367188),
        )

        self.detected_object = None

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.subtask_manager.hri.say(
                f"I'll guide you to the {location}. Please follow me.", wait=False
            )
        result = Status.EXECUTION_ERROR
        retry = 0
        while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
            future = self.subtask_manager.nav.move_to_location(location, sublocation)
            if "navigation" not in self.subtask_manager.get_mocked_areas():
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
            retry += 1

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def run(self):
        """State machine"""

        if self.current_state == CleanTableTM.TaskStates.START:
            Logger.state(self, "Starting Clean Table Task")
            self.subtask_manager.hri.say("Starting Clean the Table Task", wait=False)
            self.current_state = CleanTableTM.TaskStates.WAIT_FOR_DOOR

        if self.current_state == CleanTableTM.TaskStates.WAIT_FOR_DOOR:
            Logger.state(self, "Waiting for door to open")
            self.subtask_manager.hri.say("Please open the door to proceed", wait=False)
            res = "closed"
            while res == "closed":
                time.sleep(1)
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")

            self.current_state = CleanTableTM.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == CleanTableTM.TaskStates.NAVIGATE_TO_TABLE:
            Logger.state(self, "Navigating to the table")
            self.navigate_to("kitchen", "table")
            self.current_state = CleanTableTM.TaskStates.DETECT_OBJECTS

        if self.current_state == CleanTableTM.TaskStates.DETECT_OBJECTS:
            Logger.state(self, "Detecting objects on the table")
            s, detections = self.subtask_manager.vision.detect_objects()
            s, labels = self.subtask_manager.vision.get_labels(detections)

            object_to_pick = self.pick_objects[self.object_index]
            status, target = self.subtask_manager.hri.find_closest(
                labels,
                object_to_pick,
            )
            self.detected_object = target[0]

            self.subtask_manager.hri.say(
                f"I have detected a {self.detected_object} on the table. I will now pick it up.",
                wait=False,
            )
            self.current_state = CleanTableTM.TaskStates.PICK_OBJECT

        if self.current_state == CleanTableTM.TaskStates.PICK_OBJECT:
            Logger.state(self, "Picking object from the table")
            self.subtask_manager.manipulation.pick_object(self.detected_object)
            self.current_state = CleanTableTM.TaskStates.NAVIGATE_TO_DROPOFF

        if self.current_state == CleanTableTM.TaskStates.NAVIGATE_TO_DROPOFF:
            Logger.state(self, "Navigating to the trashbin or dishwasher")
            if self.pick_objects[self.object_index] == "drink":
                self.navigate_to("kitchen", "trashbin", say=False)
            else:
                self.navigate_to("kitchen", "dishwasher", say=False)
            self.current_state = CleanTableTM.TaskStates.PLACE_OBJECT

        if self.current_state == CleanTableTM.TaskStates.PLACE_OBJECT:
            Logger.state(self, "Placing object in the trashbin or dishwasher")
            self.timeout()  # Small timeout to finish moving
            if self.pick_objects[self.object_index] == "drink":
                self.subtask_manager.manipulation.place_in_point(self.trash_place)
                # self.subtask_manager.manipulation.move_joint_positions("trash")
                self.subtask_manager.manipulation.open_gripper()
            else:
                self.subtask_manager.manipulation.place()

            self.object_index += 1
            if self.object_index < len(self.pick_objects):
                self.current_state = CleanTableTM.TaskStates.NAVIGATE_TO_TABLE
            else:
                self.current_state = CleanTableTM.TaskStates.END

        if self.current_state == CleanTableTM.TaskStates.END:
            Logger.state(self, "Ending Clean Table Task")
            self.running_task = False


def main(args=None):
    """Main function to run the Clean Table Task Manager."""
    rclpy.init(args=args)
    node = CleanTableTM()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        Logger.info(node, "Clean Table Task Manager stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

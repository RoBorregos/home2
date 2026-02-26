#!/usr/bin/env python3

"""
Task Manager for Clean the Table
Robocup @Home 2025

Required nav locations:
- kitchen table
- kitchen trashbin
-kitchen dishwasher

"""

import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from std_msgs.msg import Header
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

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
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        END = "END"

    def __init__(self):
        """Initialize the node."""
        super().__init__("clean_table_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.STORING_GROCERIES, mock_areas=[])
        self.current_state = CleanTableTM.TaskStates.WAIT_FOR_DOOR
        self.running_task = True
        self.current_attempt = 0

        self.pick_objects = ["drink", "drink", "bowl", "cup", "plate", "spoon", "fork"]
        self.drinks = ["coke", "kuat_soda", "milk", "orange_juice", "fanta", "coffee"]
        self.deus_picks = ["spoon", "fork", "plate", "red_plate"]
        self.object_index = 0
        # BUENA
        self.trash_place = PointStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
            point=Point(x=-5.8760528564453125, y=-1.982506513595581, z=0.4),
        )
        self.dish_washer = PointStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
            point=Point(x=-5.19636869430542, y=2.062105894088745, z=0.4),
        )
        # Testing

        # self.trash_place = PointStamped(
        #     header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
        #     point=Point(x=-4.802899360656738, y=1.0740711688995361, z=0.4),
        # )
        # self.dish_washer = PointStamped(
        #     header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
        #     point=Point(x=-4.27923583984375, y=2.618645191192627, z=0.85),
        # )

        self.detected_object = None

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            Logger.info(self, f"Moving to {location}")
            # self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.subtask_manager.nav.resume_nav()
            # self.subtask_manager.hri.say(
            #     f"I'll guide you to the {location}. Please follow me.", wait=False
            # )
        result = Status.EXECUTION_ERROR
        retry = 0
        while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
            future = self.subtask_manager.nav.move_to_location(location, sublocation)
            if "navigation" not in self.subtask_manager.get_mocked_areas():
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
            retry += 1
        self.subtask_manager.nav.pause_nav()

    def deus_pick(self, object):
        deus_machina_retries = 0
        self.subtask_manager.hri.say(
            f"I couldn't make a pick on my own. I will require some help picking the {object}!"
        )
        while True:
            if deus_machina_retries >= 3:
                self.subtask_manager.hri.say(
                    "I couldn't hear your confirmation, I will abort picking the object."
                )
                return Status.TARGET_NOT_FOUND, ""
            s, res = self.subtask_manager.hri.confirm(
                f"Have you placed the {object} on my gripper?", use_hotwords=False
            )
            s, detections = self.subtask_manager.vision.detect_objects(timeout=10)
            if res == "yes":
                self.subtask_manager.hri.say("Thank you. I will close my gripper")
                return self.subtask_manager.manipulation.close_gripper(), ""
            else:
                deus_machina_retries += 1

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def run(self):
        """State machine"""
        if self.current_state == CleanTableTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Press the button to start")
            # while not self.subtask_manager.hri.start_button_clicked:
            #     rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            self.current_state = CleanTableTM.TaskStates.START

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
                # else:
                # Logger.error(self, .say(
            #     f"I'll guide you to the {location}. Please follow me.", wait=False
            # )"Failed to check door status")

            self.current_state = CleanTableTM.TaskStates.NAVIGATE_TO_TABLE

        if self.current_state == CleanTableTM.TaskStates.NAVIGATE_TO_TABLE:
            Logger.state(self, "Navigating to the table")
            self.navigate_to("kitchen")
            self.subtask_manager.hri.say(
                "Please move the chairs away from the table, so I can pick objects."
            )
            self.subtask_manager.hri.confirm("Say yes when the table is ready")
            self.navigate_to("kitchen", "table")

            self.current_state = CleanTableTM.TaskStates.DETECT_OBJECTS

        if self.current_state == CleanTableTM.TaskStates.DETECT_OBJECTS:
            Logger.state(self, "Detecting objects on the table")
            labels = []
            detections = []
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            while len(detections) == 0:
                # s, detections = self.subtask_manager.vision.detect_objects()
                time.sleep(2.5)
                try:
                    status, detections = self.subtask_manager.vision.detect_objects(timeout=10)
                    print(detections)
                except Exception as e:
                    print(e)
                    pass

                if len(detections) > 0:
                    break
                self.timeout(1)
            # for i in range (5):
            #     try:
            #         s, detections = self.subtask_manager.vision.detect_objects()
            #         if len(detections) == 0:

            #         break
            #     except Exception as e:
            #         pass
            labels = self.subtask_manager.vision.get_labels(detections)
            self.detected_object = self.pick_objects[self.object_index]
            if self.detected_object == "drink":
                for label in labels:
                    if label in self.drinks:
                        self.detected_object = label
                        break

            if self.detected_object == "bowl":
                for label in labels:
                    if label == "bowl" or label == "red_bowl":
                        self.detected_object = label
                        break

            if self.detected_object == "plate":
                for label in labels:
                    if label == "red_plate" or label == "plate":
                        self.detected_object = label
                        break
            # status, target = self.subtask_manager.hri.find_closest(
            #     labels,
            #     object_to_pick,
            # )
<<<<<<< HEAD
            # self.detected_object = target[0]
=======
            # self.detected_object = target.results[0]
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c

            self.subtask_manager.hri.say(
                f"I have detected a {self.detected_object} on the table. I will now try to pick it up.",
                wait=False,
            )
            self.current_state = CleanTableTM.TaskStates.PICK_OBJECT

        if self.current_state == CleanTableTM.TaskStates.PICK_OBJECT:
            Logger.state(self, "Picking object from the table")
            if self.detected_object in self.deus_picks:
                self.deus_pick(self.detected_object)
            else:
                s = Status.EXECUTION_ERROR
                for i in range(3):
                    s = self.subtask_manager.manipulation.pick_object(self.detected_object)
                    if s == Status.EXECUTION_SUCCESS:
                        break
                if s == Status.EXECUTION_ERROR:
                    self.deus_pick(self.detected_object)
            self.current_state = CleanTableTM.TaskStates.NAVIGATE_TO_DROPOFF

        if self.current_state == CleanTableTM.TaskStates.NAVIGATE_TO_DROPOFF:
            Logger.state(self, "Navigating to the trashbin or dishwasher")
            if self.pick_objects[self.object_index] == "drink":
                self.navigate_to("kitchen", "waste_basket", say=False)
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
                # self.subtask_manager.manipulation.place()
                self.subtask_manager.manipulation.place_in_point(self.dish_washer)
                self.subtask_manager.manipulation.open_gripper()

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


if __name__ == "__main__":
    main()
    main()

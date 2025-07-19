#!/usr/bin/env python3

"""
Task Manager for E-GPSR task of Robocup @Home 2025

Required nav locations:
- trashbins (area: kitchen, subarea: trashbin)
"""

import time

import rclpy
from rclpy.node import Node
from subtask_managers.gpsr_single_tasks import GPSRSingleTask
from subtask_managers.gpsr_tasks import GPSRTask

# from subtask_managers.gpsr_test_commands import get_gpsr_comands
from utils.baml_client.types import CommandListLLM
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import PointTransformation
from frida_constants.vision_classes import BBOX
from frida_constants.vision_enums import DetectBy


import json
from collections import deque

import os
from ament_index_python.packages import get_package_share_directory

POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"
ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
MAX_TRASH_SOLVED = 1
MAX_OBJECTS_PLACED = 1


def confirm_command(interpreted_text, target_info):
    return f"Did you say {target_info}?"


def search_command(command, objects: list[object]):
    for object in objects:
        if hasattr(object, command):
            method = getattr(object, command)
            if callable(method):
                return method
    return None


class EGPSRTM(Node):
    """Class to manage the GPSR task"""

    class States:
        WAITING_FOR_BUTTON = -1
        START = 0
        EXPLORATION = 1
        HANDLE_TRASH = 2
        HANDLE_MISPLACED_OBJECT = 3
        WAIT_FOR_COMMAND = 4
        EXECUTE_COMMAND = 5
        DONE = 6
        FIND = 7
        EXECUTING_COMMAND = 8
        FINISHED_COMMAND = 9



    def __init__(self):
        """Initialize the node"""
        super().__init__("egpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.GPSR, mock_areas=[""])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = EGPSRTM.States.WAITING_FOR_BUTTON 
        self.running_task = True
        self.current_attempt = 0
        self.exploration_locations = []
        self.current_exploration_index = 0
        self.found_misplaced_objects = deque()
        self.found_trash = deque()
        self.people_to_ask_command = deque()
        self.exploration_complete = False
        self.problems_solved = {"trash": 0, "misplaced_objects": 0, "commands": 0}
        self.current_misplaced_object_index = 0
        self.index = 3
        self.pick_count = 0
        self.curr_location = "start_area"
        self.curr_sublocation = "safe_place"
        self.transform_tf = self.create_client(PointTransformation, POINT_TRANSFORMER_TOPIC)


        self.LOCATION_TO_CATEGORY = {
            ("bedroom", "side_table"): "snack",
            ("kitchen", "table"): "dish",
            ("kitchen", "dishwasher"): "cleaning_supply",
            ("office", "desk"): "fruit",
            ("office", "bar"): "drink",
            ("living_room", "cabinet"): "food",
        }

        self.CATEGORY_TO_LOCATION = {
            "snack": ("bedroom", "side_table"),
            "dishe": ("kitchen", "table"),
            "cleaning_supply": ("kitchen", "dishwasher"),
            "fruit": ("office", "desk"),
            "drink": ("office", "bar"),
            "food": ("living_room", "cabinet"),
        }
        self.PLACEMENT_LOCATIONS = [
            ("bedroom", "bedside_table"), ("bedroom", "side_table"), 
            ("bedroom", "bed"), ("kitchen", "table"), 
            ("kitchen", "dishwasher"), ("kitchen", "sink"),
            ("kitchen", "shelf"), ("kitchen", "refrigerator"),
            ("office", "desk"), ("office", "bar"),
            ("living_room", "tv_stand"), ("living_room", "cabinet"),
            ("living_room", "sofa")
        ]
        self.exploration_locations = [
            ["living_room", "house_entry"], ["living_room", "sofa"], ["living_room", "tv_stand"], ["office", "safe_place"], 
            ["office", "desk"], ["office", "bar"], ["bedroom", "safe_place"], ["bedroom", "side_table"], ["bedroom", "bed"], 
            ["bedroom", "bedside_table"], ["kitchen", "safe_place"], ["kitchen", "dishwasher"], 
            ["kitchen", "sink"], ["kitchen", "table"], ["kitchen", "shelf"], ["living_room", "cabinet"]
        ]
        

        package_share_directory = get_package_share_directory("frida_constants")
        # Load areas from the JSON file
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")
        with open(file_path, "r") as file:
            self.areas = json.load(file)

        for loc in self.PLACEMENT_LOCATIONS:
            if loc[0] not in self.areas:
                raise ValueError(f"{loc[0]} {loc[1]} not in areas.json")
            elif loc[1] not in self.areas[loc[0]]:
                raise ValueError(f"{loc[0]} not in {loc[1]} in areas.json")
        
        for loc in self.exploration_locations:
            if loc[0] not in self.areas:
                raise ValueError(f"{loc[0]} {loc[1]} not in areas.json")
            elif loc[1] not in self.areas[loc[0]]:
                raise ValueError(f"{loc[0]} not in {loc[1]} in areas.json")


        self.commands = deque()
        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands

        self.commands = []

        Logger.info(self, "EGPSRTaskManager has started.")

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location and update current position"""
        if say:
            self.subtask_manager.hri.say(f"I will go to the {location} {sublocation}.")
            # self.subtask_manager.manipulation.`follow_face`(False)
        
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        # self.subtask_manager.nav.resume_nav()
        # Log navigation
        self.get_logger().info(
            f"Navigating from {self.curr_location}/{self.curr_sublocation} to {location}/{sublocation or 'safe_place'}"
        )

        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

        # Update location using NavigationTasks after navigation
        # self.update_current_location() using NavigationTasks
        self.subtask_manager.nav.pause_nav()
        self.curr_location = location
        self.curr_sublocation = sublocation if sublocation else "safe_place"

    def detect_objects(self, retries=3, timeout=10):
        detections = []

        for _ in range(retries):
            try:
                time.sleep(2)
                status, detections = self.subtask_manager.vision.detect_objects(timeout=timeout)
            except Exception as e:
                print(e)
            if status != Status.EXECUTION_SUCCESS or detections is None or len(detections) == 0:
                detections = []
                continue
            if len(detections) > 0:
                break
        return detections

    def objects_on_floor(self, timeout=10, retries=4, below_z=0.13):
        detections = []
        results = []
        for _ in range(retries):
            try:
                time.sleep(1)
                status, detections = self.subtask_manager.vision.detect_objects(timeout=timeout)
            except Exception as e:
                print(e)
            if status != Status.EXECUTION_SUCCESS or detections is None or len(detections) == 0:
                detections = []
                continue
            if len(detections) > 0:
                break

        for det in detections:
            # if det.point3d.point.z <= below_z:
            height = self.convert_to_height(det)
            while height is None:
                height = self.convert_to_height(det)
            if height <= below_z:
                results.append(det)
                self.get_logger().info(f"Detected {det.classname} on the floor below z={below_z}m")
            else:
                self.get_logger().info(f"Detected {det.classname} above the floor at height {height}m, ignoring it")
        if len(results) == 0:
            self.get_logger().info(f"No objects detected on the floor below z={below_z}m")
        else:
            self.get_logger().info(f"Detected {len(results)} objects on the floor below z={below_z}m")
        return results
    
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

    def detect_people_with_raised_arms(self):
        """Detect people with raised arms in the current location"""
        time.sleep(1.5)
        s, cur_pose = self.subtask_manager.vision.find_person_info(DetectBy.GESTURES.value)
        self.get_logger().info(
            f"cur_pose person {cur_pose}"
            )
        try:
            if cur_pose in ["raising_right_arm", "raising_left_arm", "waving"]:
                self.get_logger().info("Detected people with raised arms")
                return True
        except Exception as e:
            self.get_logger().error(f"Error detecting people with raised arms: {e}")
            return False
        return False

    def run(self):
        if self.current_state == EGPSRTM.States.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.start_button_clicked = False
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, egpsr task will begin now")
            self.current_state = EGPSRTM.States.START
        elif self.current_state == EGPSRTM.States.START:
            res = "closed"
            while res == "closed":
                self.subtask_manager.hri.say("Waiting for door to be opened")
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")
                time.sleep(4)
            self.navigate_to("living_room", "house_entry", False)
            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I will search for some tasks.", wait=False
            )
            self.current_state = EGPSRTM.States.FIND

        elif self.current_state == EGPSRTM.States.EXPLORATION:
            self.index += 1
            self.index = self.index % len(self.exploration_locations)
            self.navigate_to(
                self.exploration_locations[self.index][0],
                self.exploration_locations[self.index][1],
                True,
            )
            self.current_state = EGPSRTM.States.FIND
        elif self.current_state == EGPSRTM.States.FIND:
            time.sleep(3)
            location = (
                self.exploration_locations[self.index][0],
                self.exploration_locations[self.index][1],
            )
            if location in self.PLACEMENT_LOCATIONS:
                res = []
            else:
                res = self.objects_on_floor(below_z=0.5, retries=4, timeout=10)
                if len(res) > 0:
                    for i in res:
                        # self.subtask_manager.hri.confirm(
                        #     f"Can you please help me picking the {i.classname} from the floor? please say yes when you are done"
                        # )
                        # self.subtask_manager.hri.confirm(
                        #     f"Can you please help me throwing the {i.classname} into the trashbin? please say yes when you are done"
                        # )
                        s, confirmation = self.subtask_manager.hri.confirm(f"Can you please help me picking the {i.classname} and placing it into the trashbin? please say yes when you are done")                               
                if len(res) == 0: 
                    self.subtask_manager.hri.say("I didnt find any trash objects in the floor")

            if location in self.PLACEMENT_LOCATIONS:
                self.subtask_manager.manipulation.move_to_position("table_stare")
                time.sleep(1)
                objs = self.detect_objects()
                new_objs = []
                max_distance = 1.0
                for objeton in objs:
                    if objeton.point3d.point.z <= max_distance:
                        new_objs.append(objeton)
                objs = new_objs
                for i in objs:
                    category = self.subtask_manager.hri.deterministic_categorization(i.classname)
                    if location != self.CATEGORY_TO_LOCATION[category]:
                        status_pick = None
                        self.subtask_manager.hri.say(
                            f"I have detected a {i.classname} on the {location[0]} {location[1]} which is not its correct location. I will try to pick it up. and place it on the {self.CATEGORY_TO_LOCATION[category][0]} {self.CATEGORY_TO_LOCATION[category][1]}."
                        )
                        s, a = self.gpsr_individual_tasks.pick_object({"action": "pick_object", "object_to_pick": i.classname})

                        if s == Status.TARGET_NOT_FOUND:
                            break
                        correct_loc = self.CATEGORY_TO_LOCATION[category]
                        self.navigate_to(correct_loc[0], correct_loc[1])
                        self.subtask_manager.manipulation.move_to_position("table_stare")
                        self.subtask_manager.hri.say(
                            f"Now I will place the {i.classname} on the {correct_loc[0]} {correct_loc[1]} corresponding to its category {category}."
                        )
                        self.subtask_manager.manipulation.move_to_position("table_stare")
                        if correct_loc[1] == "shelve" or correct_loc[1] == "shelf" or correct_loc[1] == "cabinet":
                            self.subtask_manager.manipulation.get_optimal_pose_for_plane(0.6, 0.5)
                            self.subtask_manager.manipulation.place_on_shelf(
                                0.7, 0.5
                            )
                        else:
                            self.gpsr_individual_tasks.place_object(None)
                        self.current_state = EGPSRTM.States.EXPLORATION
                        return
            
            if location in self.PLACEMENT_LOCATIONS:
                self.current_state = EGPSRTM.States.EXPLORATION
                return
            # execute command
            self.subtask_manager.hri.say("Raise your hand If you have any tasks or commands for me, please raise your hand and keep it raised")
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
            people = self.detect_people_with_raised_arms()
            max_retries = 2
            while not people and max_retries > 0:
                self.subtask_manager.hri.say("Raise your hand if you have a task for me please and keep it raised", wait=False)
                time.sleep(2)
                people = self.detect_people_with_raised_arms()
                max_retries -= 1
            if not people:
                self.subtask_manager.hri.say(
                    "I did not detect any person with raised arms. I will continue exploring."
                )
                self.current_state = EGPSRTM.States.EXPLORATION
                return
            if people:
                self.subtask_manager.hri.say(
                    "I have detected a person with raised arms. Can you please approach me?"
                )
                self.subtask_manager.hri.confirm(
                    "Please confirm if you are ready to give me a command. Say yes when you are ready."
                )

                s, user_command = self.subtask_manager.hri.ask_and_confirm(
                    "What is your command?",
                    "LLM_command",
                    context="The user was asked to say a command. We want to infer his complete instruction from the response",
                    confirm_question=confirm_command,
                    use_hotwords=False,
                    retries=ATTEMPT_LIMIT,
                    min_wait_between_retries=5.0,
                    skip_extract_data=True,
                )
                max_retries = 3
                while s != Status.EXECUTION_SUCCESS and max_retries > 0:
                    self.subtask_manager.hri.say(
                        "I did not understand your command. Please try again."
                    )
                    s, user_command = self.subtask_manager.hri.ask_and_confirm(
                        "What is your command?",
                        "LLM_command",
                        context="The user was asked to say a command. We want to infer his complete instruction from the response",
                        confirm_question=confirm_command,
                        use_hotwords=False,
                        retries=ATTEMPT_LIMIT,
                        min_wait_between_retries=5.0,
                        skip_extract_data=True,
                    )
                    max_retries -= 1
                if s != Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        "I am sorry, I could not understand your command."
                    )
                    self.current_state = EGPSRTM.States.EXPLORATION
                    return
                self.subtask_manager.hri.say(
                    "heard: " + user_command, wait=False
                )
                self.subtask_manager.hri.say(
                    "I am planning how to perform your command, please wait a moment", wait=False
                )
                s, commands = self.subtask_manager.hri.command_interpreter(user_command)

                self.get_logger().info(
                    f"Interpreted command: {user_command} -> {str(commands)}"
                )
                if s != Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        "I am sorry, I could not understand your command."
                    )
                    self.current_state = EGPSRTM.States.EXPLORATION
                    return
                self.subtask_manager.hri.say("I will now execute your command", wait=False)
                self.commands = commands
                self.current_state = EGPSRTM.States.EXECUTING_COMMAND
                return
            self.subtask_manager.hri.say(
                "I did not detect any person with raised arms. I will continue exploring."
            )
            self.current_state = EGPSRTM.States.EXPLORATION
            return
        elif self.current_state == EGPSRTM.States.EXECUTING_COMMAND:
            if len(self.commands) == 0:
                self.current_state = EGPSRTM.States.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)

                self.get_logger().info(f"Executing command: {str(command)}")

                try:
                    exec_commad = search_command(
                        command.action,
                        [self.gpsr_tasks, self.gpsr_individual_tasks],
                    )
                    if exec_commad is None:
                        self.get_logger().error(
                            f"Command {command} is not implemented in GPSRTask or in the subtask managers."
                        )
                    else:
                        status, res = exec_commad(command)
                        self.get_logger().info(f"status-> {str(status)}")
                        self.get_logger().info(f"res-> {str(res)}")
                        self.subtask_manager.hri.add_command_history(
                            command,
                            res,
                            status,
                        )
                except Exception as e:
                    self.get_logger().warning(
                        f"Error occured while executing command ({str(command)}): " + str(e)
                    )
        elif self.current_state == EGPSRTM.States.FINISHED_COMMAND:
            """Handle the finished command state"""
            self.subtask_manager.hri.say(
                "I have finished executing the command.",
                wait=False,
            )
            self.current_state = EGPSRTM.States.EXPLORATION
            return
        # elif self.current_state == self.States.HANDLE_TRASH:
        #     self._handle_trash()

        elif self.current_state == self.States.HANDLE_MISPLACED_OBJECT:
            self._handle_misplaced_object()

        elif self.current_state == self.States.WAIT_FOR_COMMAND:
            self._handle_wait_for_command()
        elif self.current_state == self.States.EXECUTE_COMMAND:
            self._handle_execute_command()

        elif self.current_state == self.States.DONE:
            """Handle completion state"""
            self.subtask_manager.hri.say(
                "I am done with the task. I will now return to my home position.",
                wait=False,
            )
            self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = EGPSRTM()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

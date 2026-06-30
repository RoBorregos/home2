#!/usr/bin/env python3

"""
Task Manager for E-GPSR task of Robocup @Home 2025

Required nav locations:
- trashbins (area: kitchen, subarea: trashbin)
"""

import time

import rclpy
from rclpy.node import Node
from task_manager.subtask_managers.gpsr_single_tasks import GPSRSingleTask
from task_manager.subtask_managers.gpsr_tasks import GPSRTask

# from subtask_managers.gpsr_test_commands import get_gpsr_comands
from task_manager.utils.baml_client.types import CommandListLLM
from task_manager.utils.exploration_planner import ExplorationPlanner
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import PointTransformation
from frida_constants.vision_classes import BBOX
from frida_constants.vision_enums import DetectBy


from collections import deque

POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"
ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
MAX_TRASH_SOLVED = 1
MAX_OBJECTS_PLACED = 1

# Subarea names that exist in areas.json but are not real placement surfaces —
# they are pure navigation waypoints or special spots. Anything else is a
# candidate placement location.
NON_PLACEMENT_SUBAREAS = {
    "safe_place",
    "polygon",
    "trashbin",
    "house_entry",
    "entrance",
}

# Objects the gripper struggles to handle; skip during misplaced-object cleanup.
IGNORE_PICK = ["orange_juice", "cornflakes", "tuna_can"]

# Fallback when hri.query_location cannot resolve a category to a location.
# Used only as a safety net so the task does not abort if the locations
# collection is unseeded or returns junk.
FALLBACK_CATEGORY_TO_LOCATION = {
    "snack": ("bedroom", "side_table"),
    "dish": ("kitchen", "table"),
    "cleaning_supply": ("kitchen", "dishwasher"),
    "fruit": ("office", "desk"),
    "drink": ("office", "bar"),
    "food": ("living_room", "cabinet"),
}


def confirm_command(interpreted_text, target_info):
    return f"Did you say {target_info}?"


def search_command(command, objects: list[object]):
    for object in objects:
        if hasattr(object, command):
            method = getattr(object, command)
            if callable(method):
                return method
    return None


class FINALS_TM(Node):
    """Class to manage the GPSR task"""

    class States:
        WAITING_FOR_BUTTON = -1
        START = 0
        EXPLORATION = 1
        WELCOME_GUEST = 2
        CHECK_TRASH = 3
        CHECK_OBJECTS = 4
        CHECK_PERSON = 5
        EXECUTING_COMMAND = 6
        FINISHED_COMMAND = 7
        DONE = 8

    def __init__(self):
        """Initialize the node"""
        super().__init__("finals_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.GPSR, mock_areas=[])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = FINALS_TM.States.WAITING_FOR_BUTTON
        self.running_task = True
        self.current_attempt = 0
        self.exploration_locations = []
        self.current_exploration_index = 0
        self.found_misplaced_objects = deque()
        self.found_trash = deque()
        self.people_to_ask_command = deque()
        self.exploration_complete = False
        self.problems_solved = {"trash": 0, "misplaced_objects": 0, "commands": 0}
        self.guest_welcomed = False
        self.current_misplaced_object_index = 0
        self.index = 0
        self.pick_count = 0
        self.curr_location = "start_area"
        self.curr_sublocation = "safe_place"
        self.transform_tf = self.create_client(PointTransformation, POINT_TRANSFORMER_TOPIC)

        # Areas / route are loaded dynamically on START so the task picks up
        # whatever map the navigation stack is currently using (matches the
        # pattern in gpsr_task_manager.py: nav.retrieve_areas() with
        # nav.areas_backup as fallback).
        self._areas: dict | None = None
        self.exploration_locations: list[tuple[str, str]] = []
        self.placement_locations: set[tuple[str, str]] = set()

        self.commands = deque()
        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands

        Logger.info(self, "FINALS_TM has started.")

    def timeout(self, timeout: int = 2):
        time.sleep(timeout)

    def _get_areas(self) -> dict:
        """Fetch the live areas map. Prefers the retrieve_areas service so
        poses reflect the current map; falls back to nav.areas_backup
        (loaded from areas.json at package init) when the service is
        unavailable or returns junk."""
        if self._areas is not None:
            return self._areas
        areas = None
        try:
            status, data = self.subtask_manager.nav.retrieve_areas()
            if status == Status.EXECUTION_SUCCESS and isinstance(data, dict) and data:
                areas = data
            else:
                Logger.warn(self, f"retrieve_areas returned status={status}; using backup map")
        except Exception as e:
            Logger.warn(self, f"retrieve_areas failed: {e}; using backup map")
        if areas is None:
            backup = getattr(self.subtask_manager.nav, "areas_backup", None)
            areas = backup if isinstance(backup, dict) else {}
        self._areas = areas
        return self._areas

    def _build_route_from_areas(self, areas: dict) -> None:
        """Populate self.exploration_locations and self.placement_locations
        from the live areas map. Exploration order uses ExplorationPlanner
        (nearest-neighbor over rooms) and within each room visits its
        subareas in the order they appear in the map."""
        if not areas:
            Logger.error(self, "No areas available; exploration route will be empty")
            self.exploration_locations = []
            self.placement_locations = set()
            return

        try:
            planner = ExplorationPlanner(areas)
            room_order = planner.plan_exploration_order(start_area=self.curr_location)
        except Exception as e:
            Logger.warn(self, f"ExplorationPlanner failed ({e}); falling back to insertion order")
            room_order = [r for r in areas.keys() if r not in ("start_area", "entrance")]

        route: list[tuple[str, str]] = []
        placement: set[tuple[str, str]] = set()
        for room in room_order:
            subareas = areas.get(room, {})
            if not isinstance(subareas, dict):
                continue
            for sub, pose in subareas.items():
                if sub in NON_PLACEMENT_SUBAREAS:
                    continue
                if not isinstance(pose, list) or len(pose) < 2:
                    continue
                route.append((room, sub))
                placement.add((room, sub))
            # If the room has a safe_place, visit it once at the start of the
            # room so the robot has a known anchor before sweeping surfaces.
            if "safe_place" in subareas and (room, "safe_place") not in route:
                route.insert(
                    next((i for i, loc in enumerate(route) if loc[0] == room), len(route)),
                    (room, "safe_place"),
                )

        self.exploration_locations = route
        self.placement_locations = placement
        Logger.info(
            self,
            f"Built exploration route ({len(route)} waypoints, "
            f"{len(placement)} placement spots): {route}",
        )

    def _resolve_category_location(self, category: str) -> tuple[str, str] | None:
        """Resolve a category name (e.g. 'snack') to (area, subarea) using
        hri.query_location, with FALLBACK_CATEGORY_TO_LOCATION as safety net."""
        try:
            hits = self.subtask_manager.hri.query_location(category, top_k=1)
            if hits:
                hit = hits[0]
                area = getattr(hit, "area", None)
                sub = getattr(hit, "subarea", None) or "safe_place"
                if area:
                    return (area, sub)
        except Exception as e:
            Logger.warn(self, f"query_location('{category}') failed: {e}")
        fallback = FALLBACK_CATEGORY_TO_LOCATION.get(category)
        if fallback:
            Logger.info(self, f"Using fallback location for category '{category}': {fallback}")
        return fallback

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location and update current position"""
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.clear_collision_objects()
        self.subtask_manager.manipulation.move_to_position("nav_pose")

        if say:
            self.subtask_manager.hri.say(f"I will go to the {location} {sublocation}.")

        # Log navigation
        self.get_logger().info(
            f"Navigating from {self.curr_location}/{self.curr_sublocation} to {location}/{sublocation or 'safe_place'}"
        )

        self.subtask_manager.nav.move_to_location(location, sublocation)

        self.curr_location = location
        self.curr_sublocation = sublocation if sublocation else "safe_place"

    def detect_objects(self, retries=3, timeout=10, ignore=False):
        detections = []

        for _ in range(retries):
            try:
                time.sleep(2)
                status, detections = self.subtask_manager.vision.detect_objects(
                    timeout=timeout, ignore_labels=[] if not ignore else IGNORE_PICK
                )
            except Exception as e:
                self.get_logger().error(f"Error detecting objects: {e}")
            if status != Status.EXECUTION_SUCCESS or detections is None or len(detections) == 0:
                detections = []
                continue
            if len(detections) > 0:
                break
        return detections

    def objects_on_floor(self, timeout=10, retries=4, below_z=0.00):
        detections = []
        results = []
        for _ in range(retries):
            try:
                time.sleep(1)
                status, detections = self.subtask_manager.vision.detect_objects(timeout=timeout)
            except Exception as e:
                self.get_logger().error(f"Error detecting objects on floor: {e}")
            if status != Status.EXECUTION_SUCCESS or detections is None or len(detections) == 0:
                detections = []
                continue
            if len(detections) > 0:
                break

        for det in detections:
            height = self.convert_to_height(det)
            if height is not None and height <= below_z:
                results.append(det)
                self.get_logger().info(f"Detected {det.classname} on the floor below z={below_z}m")
            elif height is not None:
                self.get_logger().info(
                    f"Detected {det.classname} above the floor at height {height}m, ignoring it"
                )
        if len(results) == 0:
            self.get_logger().info(f"No objects detected on the floor below z={below_z}m")
        else:
            self.get_logger().info(
                f"Detected {len(results)} objects on the floor below z={below_z}m"
            )
        return results

    def convert_to_height(self, detection: BBOX) -> float:
        """Convert the object to height using the point transformer service"""
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

            future = self.transform_tf.call_async(transform_message)

            # Wait for the future to complete without blocking the whole executor
            start_wait = time.time()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.01)
                if time.time() - start_wait > 5.0:
                    self.get_logger().error("Point transformation service timed out")
                    return None

            transformed_point = future.result()
            if not transformed_point.success:
                Logger.error(self, f"{transformed_point.error_message}")
                return None

            Logger.info(self, f"Actual height: {transformed_point.transformed_point.point.z}")
            return transformed_point.transformed_point.point.z
        except Exception as e:
            Logger.error(self, f"Error converting to height: {e}")
            return None

    def detect_people_with_raised_arms(self):
        """Detect people with raised arms in the current location"""

        time.sleep(1.5)
        s, cur_pose = self.subtask_manager.vision.find_person_info(DetectBy.GESTURES.value)
        self.get_logger().info(f"cur_pose person {cur_pose}")
        try:
            if cur_pose in ["raising_right_arm", "raising_left_arm", "waving"]:
                self.get_logger().info("Detected people with raised arms")
                return True
        except Exception as e:
            self.get_logger().error(f"Error detecting people with raised arms: {e}")
            return False
        return False

    def run(self):
        if self.current_state == FINALS_TM.States.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.reset_task_status()
            self.subtask_manager.hri.publish_display_topic("/vision/detections_image")
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, egpsr task will begin now")
            self.current_state = FINALS_TM.States.START
        elif self.current_state == FINALS_TM.States.START:
            res = "closed"
            while res == "closed":
                self.subtask_manager.hri.say("Waiting for door to be opened")
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")
                time.sleep(4)
            # Build the exploration route from the live map before we start
            # moving. This replaces the previous hardcoded location list.
            self._build_route_from_areas(self._get_areas())
            if not self.exploration_locations:
                Logger.error(self, "Empty exploration route; cannot proceed with exploration")

            self.navigate_to("living_room", "house_entry", False)
            self.subtask_manager.hri.say(
                "I will start now. Referee please stay close to me in case I need help.", wait=False
            )
            self.current_state = FINALS_TM.States.EXPLORATION

        elif self.current_state == FINALS_TM.States.EXPLORATION:
            self.index += 1
            self.index = self.index % len(self.exploration_locations)
            location = self.exploration_locations[self.index]
            self.navigate_to(location[0], location[1], True)

            if location[0] == "exit" and not self.guest_welcomed:
                self.current_state = FINALS_TM.States.WELCOME_GUEST
            else:
                self.current_state = FINALS_TM.States.CHECK_TRASH

        elif self.current_state == FINALS_TM.States.WELCOME_GUEST:
            self.subtask_manager.hri.say(
                "I am at the exit door. Referee, please open the door for me so I can welcome the guest."
            )
            # Wait for door to open
            res = "closed"
            while res == "closed":
                status, res = self.subtask_manager.nav.check_door()
                if status != Status.EXECUTION_SUCCESS:
                    Logger.error(self, "Failed to check door status")
                    break
                if res == "closed":
                    time.sleep(2)

            self.subtask_manager.hri.say(
                "Welcome to our home! I am Frida. Please tell me, how can I help you today?"
            )
            self.guest_welcomed = True
            # The guest is right here, so we transition directly to CHECK_PERSON
            # to hear their command without doing other checks.
            self.current_state = FINALS_TM.States.CHECK_PERSON

        elif self.current_state == FINALS_TM.States.CHECK_TRASH:
            location = tuple(self.exploration_locations[self.index])
            if location not in self.placement_locations:
                """Handle trash objects on the floor"""
                if self.problems_solved["trash"] < 2:
                    res = self.objects_on_floor(below_z=0.1, retries=4, timeout=10)
                    if len(res) > 0:
                        for i in res:
                            s, confirmation = self.subtask_manager.hri.confirm(
                                f"Can you please help me picking the {i.classname} and placing it into the trashbin? please say yes when you are done",
                                retries=2,
                            )
                            if s == Status.EXECUTION_SUCCESS:
                                self.problems_solved["trash"] += 1
                                if self.problems_solved["trash"] >= 2:
                                    break
                    else:
                        self.subtask_manager.hri.say("I didn't find any trash objects on the floor")
                else:
                    self.get_logger().info("Already solved 2 trash problems, skipping.")

            self.current_state = FINALS_TM.States.CHECK_OBJECTS

        elif self.current_state == FINALS_TM.States.CHECK_OBJECTS:
            location = tuple(self.exploration_locations[self.index])
            if location in self.placement_locations:
                """Handle objects that are not in their correct location"""
                if self.problems_solved["misplaced_objects"] < 2:
                    self.subtask_manager.manipulation.move_to_position("table_stare")
                    time.sleep(1)
                    objs = self.detect_objects(ignore=True)
                    new_objs = []
                    max_distance = 1.0
                    for objeton in objs:
                        if objeton.point3d.point.z <= max_distance:
                            new_objs.append(objeton)
                    objs = new_objs

                    for i in objs:
                        category = self.subtask_manager.hri.deterministic_categorization(
                            i.classname
                        )
                        correct_loc = self._resolve_category_location(category)
                        if correct_loc is None:
                            Logger.warn(
                                self,
                                f"No location resolved for category '{category}'; skipping {i.classname}",
                            )
                            continue
                        if location != correct_loc:
                            self.subtask_manager.hri.say(
                                f"I have detected a {i.classname} on the {location[0]} {location[1]} which is not its correct location. I will try to pick it up and place it on the {correct_loc[0]} {correct_loc[1]}."
                            )
                            s, a = self.gpsr_individual_tasks.pick_object(
                                {"action": "pick_object", "object_to_pick": i.classname}
                            )

                            if s == Status.TARGET_NOT_FOUND:
                                continue

                            self.navigate_to(correct_loc[0], correct_loc[1])
                            self.subtask_manager.manipulation.move_to_position("table_stare")
                            self.subtask_manager.hri.say(
                                f"Now I will place the {i.classname} on the {correct_loc[0]} {correct_loc[1]} corresponding to its category {category}."
                            )
                            self.subtask_manager.manipulation.move_to_position("table_stare")
                            if correct_loc[1] in ["shelve", "shelf", "cabinet"]:
                                self.subtask_manager.manipulation.get_optimal_pose_for_plane(
                                    0.6, 0.5
                                )
                                self.subtask_manager.manipulation.place_on_shelf(0.7, 0.5)
                            else:
                                self.gpsr_individual_tasks.place_object(None)

                            self.problems_solved["misplaced_objects"] += 1
                            self.pick_count += 1
                            if self.problems_solved["misplaced_objects"] >= 2:
                                break
                else:
                    self.get_logger().info("Already solved 2 misplaced object problems, skipping.")

            self.current_state = FINALS_TM.States.CHECK_PERSON

        elif self.current_state == FINALS_TM.States.CHECK_PERSON:
            """Handle waiting for and interpreting human commands"""
            if self.problems_solved["commands"] < 3:  # Just a sane limit
                self.subtask_manager.hri.say(
                    "If you have any tasks or commands for me, please raise your hand and keep it raised"
                )
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="front_stare", velocity=0.5, degrees=True
                )

                people = self.detect_people_with_raised_arms()
                max_retries = 2
                while not people and max_retries > 0:
                    self.subtask_manager.hri.say(
                        "Raise your hand if you have a task for me please and keep it raised",
                        wait=False,
                    )
                    time.sleep(2)
                    people = self.detect_people_with_raised_arms()
                    max_retries -= 1

                if not people:
                    self.subtask_manager.hri.say(
                        "I did not detect any person with raised arms. I will continue exploring."
                    )
                else:
                    self.subtask_manager.hri.say(
                        "I have detected a person with raised arms. Can you please approach me?"
                    )
                    self.subtask_manager.hri.confirm(
                        "Please confirm if you are ready to give me a command. Say yes when you are ready.",
                        retries=2,
                    )

                    s, user_command = self.subtask_manager.hri.ask_and_confirm(
                        "What is your command?",
                        "LLM_command",
                        context="The user was asked to say a command. We want to infer his complete instruction from the response",
                        confirm_question=confirm_command,
                        use_hotwords=False,
                        retries=2,
                        min_wait_between_retries=2.0,
                        skip_extract_data=True,
                    )

                    if s != Status.EXECUTION_SUCCESS:
                        self.subtask_manager.hri.say(
                            "I am sorry, I could not understand your command."
                        )
                    else:
                        self.subtask_manager.hri.say("heard: " + user_command, wait=False)
                        self.subtask_manager.hri.say(
                            "I am planning how to perform your command, please wait a moment",
                            wait=False,
                        )
                        s, commands = self.subtask_manager.hri.command_interpreter(user_command)

                        self.get_logger().info(
                            f"Interpreted command: {user_command} -> {str(commands)}"
                        )
                        if s != Status.EXECUTION_SUCCESS:
                            self.subtask_manager.hri.say(
                                "I am sorry, I could not understand your command."
                            )
                        else:
                            self.subtask_manager.hri.say(
                                "I will now execute your command", wait=False
                            )
                            self.commands = (
                                list(commands) if not isinstance(commands, list) else commands
                            )
                            self.problems_solved["commands"] += 1
                            self.current_state = FINALS_TM.States.EXECUTING_COMMAND
            else:
                self.get_logger().info("Already solved enough command problems, skipping.")

            # If handling command didn't trigger a state change to EXECUTING_COMMAND, go back to exploration
            if self.current_state == FINALS_TM.States.CHECK_PERSON:
                self.current_state = FINALS_TM.States.EXPLORATION

        elif self.current_state == FINALS_TM.States.EXECUTING_COMMAND:
            if len(self.commands) == 0:
                self.current_state = FINALS_TM.States.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)

                self.get_logger().info(f"Executing command: {str(command)}")

                try:
                    exec_command = search_command(
                        command.action,
                        [self.gpsr_tasks, self.gpsr_individual_tasks],
                    )
                    if exec_command is None:
                        self.get_logger().error(
                            f"Command {command} is not implemented in GPSRTask or in the subtask managers."
                        )
                    else:
                        status, res = exec_command(command)
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
        elif self.current_state == FINALS_TM.States.FINISHED_COMMAND:
            """Handle the finished command state"""
            self.subtask_manager.hri.say(
                "I have finished executing the command.",
                wait=False,
            )
            self.current_state = FINALS_TM.States.EXPLORATION
            return

        elif self.current_state == FINALS_TM.States.DONE:
            """Handle completion state"""
            self.subtask_manager.hri.say(
                "I am done with the task. I will now return to my home position.",
                wait=False,
            )
            self.subtask_manager.hri.reset_task_status()
            self.running_task = False


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = FINALS_TM()

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

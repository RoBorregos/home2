#!/usr/bin/env python3

"""
Task Manager for E-GPSR task of Robocup @Home 2025

Required nav locations:
- trashbins (area: kitchen, subarea: trashbin)
"""

import json
import os
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from exploration_planner import ExplorationPlanner
from rclpy.node import Node
from subtask_managers.gpsr_single_tasks import GPSRSingleTask
from subtask_managers.gpsr_tasks import GPSRTask

# from subtask_managers.gpsr_test_commands import get_gpsr_comands
from utils.baml_client.types import CommandListLLM
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

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

    def __init__(self):
        """Initialize the node"""
        super().__init__("egpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.EGPSR, mock_areas=[""])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = EGPSRTM.States.START
        self.running_task = True
        self.current_attempt = 0
        self.commands = deque()
        self.exploration_locations = []
        self.current_exploration_index = 0
        self.found_misplaced_objects = deque()
        self.found_trash = deque()
        self.people_to_ask_command = deque()
        self.exploration_complete = False
        self.problems_solved = {"trash": 0, "misplaced_objects": 0, "commands": 0}
        self.trash_bin_locations = {}
        self.areas = {}
        self.current_misplaced_object_index = 0
        self.trash_location = ""
        self.solve_immediately = (
            False  # If True: solve problems as encountered, if False: explore first then solve
        )
        self.interrupted_exploration_location = None  # Track where we were when interrupted
        self.interrupted_exploration_subarea = None  # Track subarea we were exploring

        # Track current robot location
        self.curr_location = "start_area"
        self.curr_sublocation = "safe_place"

        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")

        self.exploration_planner = ExplorationPlanner(file_path)

        # Get nearest trash bin locations
        self.trash_bin_locations = self.exploration_planner.get_nearest_trashbin()

        # Get optimal exploration order
        self.exploration_locations = self.exploration_planner.plan_exploration_order()

        # Log the planned order
        total_distance = self.exploration_planner.get_total_exploration_distance(
            self.exploration_locations
        )
        Logger.info(self, f"Planned exploration order: {self.exploration_locations}")
        Logger.info(self, f"Total exploration distance: {total_distance:.2f} meters")

        # Load areas from the JSON file
        with open(file_path, "r") as file:
            self.areas = json.load(file)

        # Check which area the robot is in
        for area, subarea in self.areas.items():
            if "trashbin" in subarea:
                self.trash_bin_locations[area] = subarea["trashbin"]

        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands

        # self.commands = get_gpsr_comands("custom")

        Logger.info(self, "EGPSRTaskManager has started.")

    def explore_environment(self):
        """Explore the environment to find trash and misplaced objects"""
        if self.current_exploration_index >= len(self.exploration_locations):
            # Exploration complete
            self.exploration_complete = True
            # locate a person to ask for commands
            if self.people_to_ask_command:
                self.subtask_manager.hri.say(
                    "I have finished exploring the environment. I will now look for people to ask for commands."
                )
                self.current_state = EGPSRTM.States.WAITING_FOR_COMMAND
                location = self.people_to_ask_command.pop(0)["location"]
                self.subtask_manager.hri.say(
                    f"I will now navigate to {location} to ask for commands."
                )
                self.navigate_to(location, "", False)
            else:
                self.subtask_manager.hri.say(
                    "I have finished exploring the environment. I will now return to the start position."
                )
                self.current_state = EGPSRTM.States.WAITING_FOR_COMMAND
                # Navigate back to start area
                self.navigate_to("start_area", "", False)
            return

        current_location = self.exploration_locations[self.current_exploration_index]

        self.subtask_manager.hri.say(
            f"I will now explore the {current_location} to look for trash and misplaced objects.",
            wait=False,
        )

        # Navigate to the location
        self.navigate_to(current_location, "", False)

        # Look around and analyze the scene
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="front_stare", velocity=0.5, degrees=True
        )

        # Detect objects and analyze them
        self.analyze_location_for_anomalies(current_location)

        # Move to next location
        self.current_exploration_index += 1

    def analyze_location_for_anomalies(self, location: str):
        """Analyze current location for trash and misplaced objects"""
        try:
            # Detect trash and people at the main location
            self.detect_trash()
            self.detect_people_with_raised_arms()

            # Only check subareas of the CURRENT location
            if location in self.areas:
                current_area_subareas = self.areas[location]

                for subarea_name, subarea_coords in current_area_subareas.items():
                    # Skip non-explorable subareas
                    if (
                        subarea_name not in ["safe_place", "trashbin"]
                        and subarea_coords is not None
                    ):
                        self.get_logger().info(f"Exploring subarea: {location}/{subarea_name}")
                        self.navigate_to(location, subarea_name, False)
                        self.detect_misplaced_objects()

                # Return to safe_place after exploring subareas
                self.navigate_to(location, "safe_place", False)
            else:
                self.get_logger().warning(f"Location {location} not found in areas dictionary")

        except Exception as e:
            self.get_logger().error(f"Error during location analysis: {str(e)}")

    # function to sort expected locations by distance of all misplaced objects
    def sort_misplaced_objects_by_distance(self):
        """Sort misplaced objects by distance from the current location"""
        if not self.found_misplaced_objects:
            return []

        self.found_misplaced_objects.sort(
            key=lambda obj: (
                self.exploration_planner.get_distance_between_area_subarea(
                    self.curr_location,
                    self.curr_sublocation,
                    obj["current_location"],
                    obj["current_sublocation"],
                )
                + self.exploration_planner.get_distance_between_area_subarea(
                    obj["current_location"],
                    obj["current_sublocation"],
                    obj["expected_location"],
                    obj["expected_sublocation"],
                )
            )
        )

    def sort_trash_by_distance(self):
        """Sort trash items by distance from the current location"""
        if not self.found_trash:
            return []

        self.found_trash.sort(
            key=lambda item: (
                self.exploration_planner.get_distance_between_area_subarea(
                    self.curr_location, self.curr_sublocation, item["location"], "safe_place"
                )
                + self.exploration_planner.get_distance_between_area_subarea(
                    item["location"],
                    "safe_place",
                    self.trash_bin_locations[item["location"]],
                    "trashbin",
                )
            )
        )

    def sort_people_by_distance(self):
        """Sort people by distance from the current location"""
        if not self.people_to_ask_command:
            return []

        self.people_to_ask_command.sort(
            key=lambda person: (
                self.exploration_planner.get_distance_between_area_subarea(
                    self.curr_location, self.curr_sublocation, person["location"], "safe_place"
                )
            )
        )

    def detect_trash(self):
        """Detect trash in the current location"""
        if self.problems_solved.get("trash", 0) < MAX_TRASH_SOLVED:
            s, trash_detected = self.subtask_manager.vision.detect_trash()
            if s == Status.EXECUTION_SUCCESS and trash_detected:
                self.found_trash.append(
                    {"object": "trash", "location": self.curr_location, "timestamp": time.time()}
                )
                self.get_logger().info(f"Trash detected in {self.curr_location}")
                if self.solve_immediately:
                    # Save current exploration state
                    self.interrupted_exploration_location = self.curr_location
                    self.get_logger().info(
                        f"Interrupting exploration at {self.interrupted_exploration_location} to handle trash"
                    )

                    # Change state to handle trash immediately
                    self.current_state = EGPSRTM.States.HANDLE_TRASH

    def detect_misplaced_objects(self):
        """Detect misplaced objects in the current location"""
        s, detected_objects = self.subtask_manager.vision.detect_objects()

        if s == Status.EXECUTION_SUCCESS:
            labels = self.subtask_manager.vision.get_labels(detected_objects)

            for obj_name in labels:
                # Use HRI to get expected location with context
                expected_locations = self.subtask_manager.hri.query_location(
                    obj_name, top_k=3, use_context=True
                )

                if expected_locations and len(expected_locations) > 0:
                    best_match = expected_locations[0]

                    expected_area = best_match.area
                    expected_subarea = best_match.subarea

                    # Check if object is misplaced
                    if (
                        self.curr_location != expected_area
                        or self.curr_sublocation != expected_subarea
                    ):
                        self.found_misplaced_objects.append(
                            {
                                "object": obj_name,
                                "current_location": self.curr_location,
                                "current_sublocation": self.curr_sublocation,
                                "expected_location": expected_area,
                                "expected_sublocation": expected_subarea,
                                "timestamp": time.time(),
                            }
                        )

                        self.get_logger().info(
                            f"Misplaced object detected: {obj_name} in {self.curr_location}/{self.curr_sublocation}, should be in {expected_area}/{expected_subarea}"
                        )

                        if self.solve_immediately:
                            # Save current exploration state
                            self.interrupted_exploration_location = self.curr_location
                            self.interrupted_exploration_subarea = self.curr_sublocation
                            self.get_logger().info(
                                f"Interrupting exploration at {self.curr_location}/{self.curr_sublocation} to handle misplaced object"
                            )

                            # Change state to handle misplaced object immediately
                            self.current_state = EGPSRTM.States.HANDLE_MISPLACED_OBJECT
                            return  # Exit to handle the object immediately

    def detect_people_with_raised_arms(self):
        """Detect people with raised arms in the current location"""
        s, cur_pose = self.subtask_manager.vision.find_person_info("pose")

        if s == Status.EXECUTION_SUCCESS:
            if cur_pose is not None and cur_pose in ["raising_right_arm", "raising_left_arm"]:
                self.people_to_ask_command.append(
                    {"location": self.curr_location, "timestamp": time.time()}
                )
                self.get_logger().info(f"Found person with raised arm in {self.curr_location}")

    def pick_trash(self):
        """Handle the trash detection and pickup process"""

        self.subtask_manager.hri.say(
            "I have detected trash in this location. I will now receive the trash."
        )

        # Request trash from person
        while True:
            self.subtask_manager.hri.say("Please hand me the trash.")
            self.subtask_manager.manipulation.open_gripper()
            s, confirmation = self.subtask_manager.hri.confirm(
                "Have you given me the trash? Say yes when you are done.",
                False,
                retries=5,
                wait_between_retries=2,
            )

            if confirmation == "yes":
                self.subtask_manager.hri.say("Thank you.")
                break

        self.subtask_manager.manipulation.close_gripper()
        self.subtask_manager.hri.say(
            "I have received the trash. I will now take it to the trash bin."
        )

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location and update current position"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )

        # Log navigation
        self.get_logger().info(
            f"Navigating from {self.curr_location}/{self.curr_sublocation} to {location}/{sublocation or 'safe_place'}"
        )

        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

        # Update location using NavigationTasks after navigation
        # self.update_current_location() using NavigationTasks
        self.curr_location = location
        self.curr_sublocation = sublocation if sublocation else "safe_place"

    def _handle_start(self):
        """Handle start state"""
        self.navigate_to("start_area", "", False)
        self.subtask_manager.hri.say(
            "Hi, my name is Frida. I am a general purpose robot. I can help you with some tasks."
        )

    def _handle_trash(self):
        """Handle trash detection and disposal"""
        if not self.found_trash or self.problems_solved["trash"] >= MAX_TRASH_SOLVED:
            # No more trash or limit reached
            if self.solve_immediately and self.interrupted_exploration_location:
                # Return to interrupted exploration
                self.get_logger().info(
                    f"Returning to interrupted exploration at {self.interrupted_exploration_location}"
                )
                self.current_state = self.States.EXPLORATION
                # Don't increment exploration index since we're returning
            elif self.found_misplaced_objects:
                self.current_state = self.States.HANDLE_MISPLACED_OBJECT
            else:
                self.current_state = self.States.WAIT_FOR_COMMAND
            return

        self.sort_trash_by_distance()

        # Get first trash item
        trash_item = self.found_trash.popleft()
        trash_location = trash_item["location"]

        # Navigate to trash location
        self.navigate_to(trash_location, "safe_place", False)
        self.subtask_manager.hri.say(
            f"I have detected trash at {trash_location}. I will go ahead and pick it up."
        )

        # Handle trash pickup
        self.pick_trash()

        # Navigate to nearest trash bin
        nearest_trash_bin = self.trash_bin_locations.get(
            trash_location, list(self.trash_bin_locations.keys())[0]
        )
        self.subtask_manager.hri.say("I will now navigate to the trashbin.")
        self.navigate_to(nearest_trash_bin, "trashbin", False)

        # Drop trash
        self.subtask_manager.hri.say("I will now drop the trash in the trashbin.")
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say("I have disposed of the trash successfully.")

        self.problems_solved["trash"] += 1

        # Continue with next priority or return to exploration
        if self.solve_immediately and self.interrupted_exploration_location:
            # Return to interrupted exploration
            self.get_logger().info(
                f"Trash handled. Returning to exploration at {self.interrupted_exploration_location}"
            )
            self.navigate_to(self.interrupted_exploration_location, "safe_place", False)
            self.interrupted_exploration_location = None  # Clear interruption state
            self.current_state = self.States.EXPLORATION
        elif self.found_trash and self.problems_solved["trash"] < MAX_TRASH_SOLVED:
            # Stay in trash handling state for next trash
            pass
        elif self.found_misplaced_objects:
            self.current_state = self.States.HANDLE_MISPLACED_OBJECT
        else:
            self.current_state = self.States.WAIT_FOR_COMMAND

    def _handle_wait_for_command(self):
        """Handle waiting for and receiving commands"""
        if self.problems_solved["commands"] >= MAX_COMMANDS:
            self.current_state = self.States.DONE
            return

        # Navigate to person if available
        if self.people_to_ask_command:
            self.sort_people_by_distance()
            person_location = self.people_to_ask_command.popleft()["location"]
            self.subtask_manager.hri.say(
                f"I will now navigate to {person_location} to ask for commands."
            )
            self.navigate_to(person_location, "", False)

        # Ask for command
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="front_stare", velocity=0.5, degrees=True
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

        if s != Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say("I am sorry, I could not understand you.")
            self.current_attempt += 1
            # Stay in wait state to try again
        else:
            self.subtask_manager.hri.say(
                "I am planning how to perform your command, please wait a moment", wait=False
            )
            s, self.commands = self.subtask_manager.hri.command_interpreter(user_command)

            self.get_logger().info(f"Interpreted command: {user_command} -> {str(self.commands)}")
            self.subtask_manager.hri.say("I will now execute your command")
            self.current_state = self.States.EXECUTE_COMMAND

    def _handle_execute_command(self):
        """Handle command execution"""
        if len(self.commands) == 0:
            # Commands finished
            self.subtask_manager.hri.say(
                "I have finished executing your command. I will return to the start position to await for new commands.",
                wait=False,
            )
            self.problems_solved["commands"] += 1
            self.current_state = self.States.WAIT_FOR_COMMAND
            return

        # Execute next command
        command = self.commands.popleft()
        self.get_logger().info(f"Executing command: {str(command)}")

        try:
            exec_command = search_command(
                command.action, [self.gpsr_tasks, self.gpsr_individual_tasks]
            )

            if exec_command is None:
                self.get_logger().error(
                    f"Command {command} is not implemented in GPSRTask or in the subtask managers."
                )
            else:
                status, res = exec_command(command)
                self.get_logger().info(f"status-> {str(status)}")
                self.get_logger().info(f"res-> {str(res)}")

                # Normalize status
                try:
                    status = status.value
                except Exception:
                    try:
                        status = int(status)
                    except Exception:
                        pass

                self.subtask_manager.hri.add_command_history(command, res, status)

        except Exception as e:
            self.get_logger().warning(
                f"Error occurred while executing command ({str(command)}): {str(e)}"
            )

        self.timeout(3)

    def _handle_misplaced_object(self):
        """Handle misplaced object detection and placement"""
        if (
            not self.found_misplaced_objects
            or self.problems_solved["misplaced_objects"] >= MAX_OBJECTS_PLACED
        ):
            # No more objects or limit reached
            if self.solve_immediately and self.interrupted_exploration_location:
                # Return to interrupted exploration
                self.get_logger().info(
                    f"Returning to interrupted exploration at {self.interrupted_exploration_location}"
                )
                if self.interrupted_exploration_subarea:
                    self.navigate_to(
                        self.interrupted_exploration_location,
                        self.interrupted_exploration_subarea,
                        False,
                    )
                    self.interrupted_exploration_subarea = None
                else:
                    self.navigate_to(self.interrupted_exploration_location, "safe_place", False)
                self.interrupted_exploration_location = None  # Clear interruption state
                self.current_state = self.States.EXPLORATION
            else:
                self.current_state = self.States.WAIT_FOR_COMMAND
            return

        self.sort_misplaced_objects_by_distance()

        # Get first misplaced object
        obj_info = self.found_misplaced_objects.popleft()

        self.subtask_manager.hri.say(
            f"I have detected a misplaced object: {obj_info['object']}. I will now pick it up and take it to the correct location."
        )

        # Navigate to object location
        self.navigate_to(obj_info["current_location"], obj_info["current_sublocation"], False)

        # Pick up the object
        self.subtask_manager.manipulation.pick_object(obj_info["object"])
        self.subtask_manager.hri.say("I have picked up the misplaced object.")

        # Navigate to correct location
        self.navigate_to(obj_info["expected_location"], obj_info["expected_sublocation"], False)

        # Place the object
        self.subtask_manager.hri.say(f"I will now place the {obj_info['object']}.")
        self.subtask_manager.manipulation.place()

        self.problems_solved["misplaced_objects"] += 1

        # Continue with next object or return to exploration
        if self.solve_immediately and self.interrupted_exploration_location:
            # Return to interrupted exploration
            self.get_logger().info(
                f"Object placed. Returning to exploration at {self.interrupted_exploration_location}"
            )
            if self.interrupted_exploration_subarea:
                self.navigate_to(
                    self.interrupted_exploration_location,
                    self.interrupted_exploration_subarea,
                    False,
                )
                self.interrupted_exploration_subarea = None
            else:
                self.navigate_to(self.interrupted_exploration_location, "safe_place", False)
            self.interrupted_exploration_location = None  # Clear interruption state
            self.current_state = self.States.EXPLORATION
        elif (
            self.found_misplaced_objects
            and self.problems_solved["misplaced_objects"] < MAX_OBJECTS_PLACED
        ):
            # Stay in misplaced object handling state
            pass
        else:
            self.current_state = self.States.WAIT_FOR_COMMAND

    def run(self):
        if self.current_state == EGPSRTM.States.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.start_button_clicked = False
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")
            # Wait for the start button to be pressed

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
            self.navigate_to("start_area", "", False)

            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I will search for some tasks."
            )
            self.current_state = EGPSRTM.States.EXPLORATION

        elif self.current_state == EGPSRTM.States.EXPLORATION:
            if not self.exploration_complete:
                self.explore_environment()
            if self.found_trash:
                self.current_state = self.States.HANDLE_TRASH
            elif self.found_misplaced_objects:
                self.current_state = self.States.HANDLE_MISPLACED_OBJECT
            else:
                self.current_state = self.States.WAIT_FOR_COMMAND

        elif self.current_state == self.States.HANDLE_TRASH:
            self._handle_trash()

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


# ALTERNATIVE TO GET CURRENT LOCATION WITH NAVIGATION TASKS
# def get_current_location_from_nav(self):
#     """Get current robot location using NavigationTasks ReturnLocation service"""
#     try:
#         status, results = self.subtask_manager.nav.ReturnLocation_callback()

#         if status == Status.EXECUTION_SUCCESS and results is not None:
#             detected_location = results.location
#             nearest_sublocations = results.nearest_locations

#             # Get the closest sublocation
#             closest_sublocation = nearest_sublocations[0] if nearest_sublocations else "safe_place"

#             self.get_logger().info(
#                 f"Real position detected: {detected_location}/{closest_sublocation}"
#             )

#             return detected_location, closest_sublocation
#         else:
#             self.get_logger().warning("Failed to get location from NavigationTasks")

#     except Exception as e:
#         self.get_logger().error(f"Error getting location from NavigationTasks: {str(e)}")

#     # Fallback to stored location
#     return self.curr_location, self.curr_sublocation

# def update_current_location(self):
#     """Update current location using NavigationTasks"""
#     real_location, real_sublocation = self.get_current_location_from_nav()

#     # Update stored location
#     previous_location = f"{self.curr_location}/{self.curr_sublocation}"
#     self.curr_location = real_location
#     self.curr_sublocation = real_sublocation
#     current_location = f"{self.curr_location}/{self.curr_sublocation}"

#     if previous_location != current_location:
#         self.get_logger().info(f"Location updated: {previous_location} - {current_location}")
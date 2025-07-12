#!/usr/bin/env python3

"""
Task Manager for E-GPSR task of Robocup @Home 2025

Required nav locations:
- trashbin per room (e.g. kitchen_trashbin, living_room_trashbin)
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

import json
import math
from typing import List, Tuple
import heapq

import os
from ament_index_python.packages import get_package_share_directory

ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3


# NOTE: Maybe navigation could find a better approach to visit all areas
class ExplorationPlanner:
    def __init__(self, areas_json_path: str):
        """Initialize the exploration planner with areas data"""
        with open(areas_json_path, "r") as file:
            self.areas = json.load(file)

        # Extract valid exploration areas (exclude start_area and entrance)
        self.exploration_areas = {
            name: data
            for name, data in self.areas.items()
            if name not in ["start_area", "entrance"]
            and "safe_place" in data
            and data["safe_place"]
        }

        self.distances = {}
        self._calculate_distances()

    def _calculate_distances(self):
        """Calculate Euclidean distances between all areas"""
        area_names = list(self.exploration_areas.keys())

        for i, area1 in enumerate(area_names):
            for j, area2 in enumerate(area_names):
                if i != j:
                    dist = self._euclidean_distance(area1, area2)
                    self.distances[(area1, area2)] = dist
                else:
                    self.distances[(area1, area2)] = 0

    def _euclidean_distance(self, area1: str, area2: str) -> float:
        """Calculate Euclidean distance between two areas"""
        pos1 = self.exploration_areas[area1]["safe_place"]
        pos2 = self.exploration_areas[area2]["safe_place"]

        # Extract x, y coordinates
        x1, y1 = pos1[0], pos1[1]
        x2, y2 = pos2[0], pos2[1]

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def dijkstra_shortest_path(self, start: str, end: str) -> Tuple[List[str], float]:
        """Find shortest path between two areas using Dijkstra's algorithm"""
        if start not in self.exploration_areas or end not in self.exploration_areas:
            return [], float("inf")

        # Priority queue: (distance, node)
        pq = [(0, start)]
        distances = {area: float("inf") for area in self.exploration_areas}
        distances[start] = 0
        previous = {}
        visited = set()

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current in visited:
                continue

            visited.add(current)

            if current == end:
                break

            for neighbor in self.exploration_areas:
                if neighbor not in visited:
                    distance = current_dist + self.distances[(current, neighbor)]

                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous[neighbor] = current
                        heapq.heappush(pq, (distance, neighbor))

        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        return path, distances[end]

    def plan_exploration_order(self, start_area: str = "start_area") -> List[str]:
        """Plan optimal exploration order using nearest neighbor heuristic"""
        if start_area not in self.areas:
            start_area = "start_area"

        # Get starting position
        start_pos = self.areas[start_area]["safe_place"]

        # Find nearest exploration area to start
        unvisited = set(self.exploration_areas.keys())
        current_pos = start_pos
        exploration_order = []

        # Find closest area to start position
        min_dist = float("inf")
        next_area = None

        for area in unvisited:
            area_pos = self.exploration_areas[area]["safe_place"]
            dist = math.sqrt(
                (area_pos[0] - current_pos[0]) ** 2 + (area_pos[1] - current_pos[1]) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                next_area = area

        if next_area:
            exploration_order.append(next_area)
            unvisited.remove(next_area)
            current_area = next_area

        # Continue with nearest neighbor
        while unvisited:
            min_dist = float("inf")
            next_area = None

            for area in unvisited:
                dist = self.distances[(current_area, area)]
                if dist < min_dist:
                    min_dist = dist
                    next_area = area

            if next_area:
                exploration_order.append(next_area)
                unvisited.remove(next_area)
                current_area = next_area

        return exploration_order

    def get_total_exploration_distance(self, order: List[str]) -> float:
        """Calculate total distance for exploration order"""
        if not order:
            return 0

        total_dist = 0

        # Distance from start to first area
        start_pos = self.areas["start_area"]["safe_place"]
        first_pos = self.exploration_areas[order[0]]["safe_place"]
        total_dist += math.sqrt(
            (first_pos[0] - start_pos[0]) ** 2 + (first_pos[1] - start_pos[1]) ** 2
        )

        # Distance between consecutive areas
        for i in range(len(order) - 1):
            total_dist += self.distances[(order[i], order[i + 1])]

        return total_dist


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
        START = 0
        WAITING_FOR_COMMAND = 1
        EXECUTING_COMMAND = 2
        FINISHED_COMMAND = 3
        EXPLORATION = 4
        HANDLING_TRASH = 5
        NAVIGATE_TO_TRASH_BIN = 6
        DROP_TRASH = 7
        PICK_MISPLACED_OBJECT = 8
        NAVIGATE_TO_CORRECT_LOCATION = 9
        PLACE_MISPLACED_OBJECT = 10
        DONE = 11

    def __init__(self):
        """Initialize the node"""
        super().__init__("egpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.EGPSR, mock_areas=[""])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = EGPSRTM.States.START
        self.running_task = True
        self.current_attempt = 0
        self.executed_commands = 0
        self.commands = []
        self.exploration_locations = []
        self.current_exploration_index = 0
        self.found_misplaced_objects = []
        self.found_trash = []
        self.people_to_ask_command = []
        self.exploration_complete = False
        self.trash_detected_flag = False
        self.trash_bin_location = {}
        self.current_misplaced_object_index = 0

        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")

        self.exploration_planner = ExplorationPlanner(file_path)

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
            areas = json.load(file)

        # Check which area the robot is in
        for area in areas:
            self.trash_bin_location = areas.get(area, {}).get("trashbin", [])

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
            if self.trash_bin_location[location] == []:
                if self.current_exploration_index + 1 < len(self.exploration_locations):
                    trash_loc = self.exploration_locations[self.current_exploration_index + 1]
                else:
                    trash_loc = self.exploration_locations[self.current_exploration_index - 1]
            else:
                trash_loc = location

            # Get objects in the current view
            s, detected_objects = self.subtask_manager.vision.detect_objects()
            # NOTE: uncomment labels once hri is implemented.
            # labels = self.subtask_manager.vision.get_labels(detected_objects)

            # Check if object is trash
            if not self.trash_detected_flag:
                status, trash_detected = self.subtask_manager.vision.detect_trash()
                if status == Status.EXECUTION_SUCCESS and trash_detected:
                    self.trash_detected_flag = True
                    self.get_logger().info(f"Trash detected in {location}")
                    # Transition to handling trash state when trash is detected
                    self.current_state = EGPSRTM.States.HANDLING_TRASH

            if self.current_state == EGPSRTM.States.HANDLING_TRASH:
                self.subtask_manager.hri.say("I have detected trash. I will now receive the trash.")
                # Move to a pose suitable for receiving trash
                # NOTE: consider a possible self.subtask_manager.manipulation.move_joint_positions(named_position="trash_receiving_pose", velocity=0.5, degrees=True)

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
                    "I have received the trash. I will now pick it up and take it to the trash bin."
                )
                self.current_state = EGPSRTM.States.NAVIGATE_TO_TRASH_BIN

            if self.current_state == EGPSRTM.States.NAVIGATE_TO_TRASH_BIN:
                if trash_loc:
                    self.subtask_manager.hri.say("I will now navigate to the trashbin.")
                    self.navigate_to(trash_loc, "trashbin", False)
                    self.current_state = EGPSRTM.States.DROP_TRASH

            if self.current_state == EGPSRTM.States.DROP_TRASH:
                self.subtask_manager.hri.say("I will now drop the trash in the trashbin.")
                self.subtask_manager.manipulation.open_gripper()
                self.subtask_manager.hri.say(
                    "I have dropped the trash in the trashbin. I will now return to my previous position."
                )
                self.navigate_to(location, "", False)
                self.current_state = EGPSRTM.States.EXPLORATION

            if s == Status.EXECUTION_SUCCESS:
                misplaced_object, expected_location, expected_sublocation = None, None, None
                pass
                # Check if object is misplaced
                # TODO: HRI verification of misplaced objects
                # misplaced_object, expected_loc, expected_subloc = self.subtask_manager.hri.find_misplaced_object(labels)
                if misplaced_object:
                    self.get_logger().info(
                        f"Found misplaced object: {misplaced_object} in {location}. Expected location: {expected_location}, sublocation: {expected_sublocation}"
                    )
                    self.subtask_manager.hri.say(
                        f"I have detected a misplaced object: {misplaced_object}. I will now pick it up and take it to the correct location."
                    )
                    self.current_state = EGPSRTM.States.PICK_MISPLACED_OBJECT
                    self.subtask_manager.manipulation.pick_object(misplaced_object)
                    self.subtask_manager.hri.say(
                        "I have picked up the misplaced_object. I will now navigate to the correct location."
                    )
                    self.navigate_to(expected_location, expected_sublocation, False)
                    self.current_state = EGPSRTM.States.PLACE_MISPLACED_OBJECT
                    self.subtask_manager.hri.say(f"I will now place the {misplaced_object}.")
                    self.subtask_manager.manipulation.place()
                    self.subtask_manager.hri.say("I will now return to my original location.")
                    self.navigate_to(location, "", False)
                    self.current_state = EGPSRTM.States.EXPLORATION

            # Check for people with raised arms
            status, cur_pose = self.subtask_manager.vision.find_person_info_client(
                "pose"
            )  # check if lower or upper case i.e. POSE

            if status == Status.EXECUTION_SUCCESS:
                if (
                    cur_pose is not None
                    and cur_pose == "raising_right_arm"
                    or cur_pose == "raising_left_arm"
                ):
                    self.people_to_ask_command.append(
                        {"location": location, "timestamp": time.time()}
                    )
                    self.get_logger().info(f"Found person with raised arm in {location}")

        except Exception as e:
            self.get_logger().error(f"Error during location analysis: {str(e)}")

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

    def run(self):
        """State machine"""
        if self.current_state == EGPSRTM.States.START:
            self.navigate_to("start_area", "", False)
            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I can help you with some tasks."
            )
            self.current_state = EGPSRTM.States.EXPLORATION

        elif self.current_state == EGPSRTM.States.EXPLORATION:
            if not self.exploration_complete:
                self.explore_environment()
            else:
                self.current_state = EGPSRTM.States.WAITING_FOR_COMMAND

        elif self.current_state == EGPSRTM.States.WAITING_FOR_COMMAND:
            if self.executed_commands >= MAX_COMMANDS:
                self.current_state = EGPSRTM.States.DONE
                return

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
            else:
                self.subtask_manager.hri.say(
                    "I am planning how to perform your command, please wait a moment", wait=False
                )
                s, self.commands = self.subtask_manager.hri.command_interpreter(user_command)

                self.get_logger().info(
                    f"Interpreted command: {user_command} -> {str(self.commands)}"
                )
                self.subtask_manager.hri.say("I will now execute your command")
                self.current_state = EGPSRTM.States.EXECUTING_COMMAND
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

                        try:
                            status = status.value
                        except Exception:
                            try:
                                status = int(status)
                            except Exception:
                                pass

                        self.subtask_manager.hri.add_command_history(
                            command,
                            res,
                            status,
                        )
                except Exception as e:
                    self.get_logger().warning(
                        f"Error occured while executing command ({str(command)}): " + str(e)
                    )
            self.timeout(3)

        elif self.current_state == EGPSRTM.States.FINISHED_COMMAND:
            self.subtask_manager.hri.say(
                "I have finished executing your command. I will return to the start position to await for new commands.",
                wait=False,
            )
            self.executed_commands += 1
            self.current_state = EGPSRTM.States.WAITING_FOR_COMMAND
            if self.people_to_ask_command:
                self.subtask_manager.manipulation.move_joint_positions(
                    named_position="front_stare", velocity=0.5, degrees=True
                )
                # If there are people to ask for commands, navigate to the first person
                location = self.people_to_ask_command.pop(0)["location"]
                self.subtask_manager.hri.say(
                    f"I will now navigate to {location} to ask for commands."
                )
                self.navigate_to(location, "", False)
        elif self.current_state == EGPSRTM.States.DONE:
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

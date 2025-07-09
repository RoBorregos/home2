#!/usr/bin/env python3

"""
Task Manager for Give me a Hand Robocup @Home 2025
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


class HandTM(Node):
    class TaskStates:
        START = "START"
        NAVIGATE_TO_ROOM = "NAVIGATE_TO_ROOM"
        MAP_ROOM = "MAP_ROOM"
        FIND_OPERATOR = "FIND_OPERATOR"
        PICK_OBJECT = "PICK_OBJECT"
        ASK_OBJECT = "ASK_OBJECT"
        PLACE_OBJECT = "PLACE_OBJECT"
        RETURN_TO_OPERATOR = "RETURN_TO_OPERATOR"
        END = "END"

    def __init__(self):
        """Initialize the node"""
        super().__init__("hand_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.HAND, mock_areas=["navigation", "vision", "manipulation"]
        )
        self.selected_room = "living_room"
        self.look_degrees = [-45, 0, 45]
        self.handed_objects = 0
        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")
        with open(file_path, "r") as file:
            self.areas = json.load(file)

        Logger.info(self, "HandTaskManager has started.")

    def navigate_to(self, location: str, sublocation: str = "", say=True):
        """Navigate to the location"""
        Logger.info(self, f"Moving to {location} {sublocation}")
        if say:
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.hri.say(f"I'll go to {location} {sublocation}.", wait=False)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self, future)

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def run(self):
        """State machine"""

        if self.current_state == HandTM.TaskStates.START:
            Logger.state(self, "Starting task")
            # self.navigate_to("entrance")
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.current_state = HandTM.TaskStates.NAVIGATE_TO_ROOM

        if self.current_state == HandTM.TaskStates.NAVIGATE_TO_ROOM:
            Logger.state(self, "Going to target room")
            self.navigate_to(self.selected_room)
            self.subtask_manager.hri.say("I've arrived to the target room.", wait=False)
            self.current_state = HandTM.TaskStates.MAP_ROOM

        if self.current_state == HandTM.TaskStates.MAP_ROOM:
            Logger.state(self, "Mapping room guest")

            for sublocation in self.areas[self.selected_room]:
                self.navigate_to(self.selected_room, sublocation)
                # TODO: detect objects and store them somewhere
                # Also, possible take a picture of all the tables and save them

            self.current_state = HandTM.TaskStates.FIND_OPERATOR

        if self.current_state == HandTM.TaskStates.FIND_OPERATOR:
            # Approach: take a look at several angles, and search for the operator (raising hand or people)

            # If there is a raising hand, approach the person and assume it is the operator
            # Else, approach each person in the room and ask if they are the operator

            Logger.state(self, "Finding operator")
            self.navigate_to(self.selected_room)
            self.subtask_manager.hri.say("I'll look for calling operators")
            self.subtask_manager.manipulation.move_to_position("front_stare")

            operator_found = False

            people = []

            for degree in self.look_degrees:
                self.subtask_manager.manipulation.pan_to(degree)

                # if person detected:
                # save_location

                # if person_detected and hand_up
                # operator_found = True
                # approach_person
                # return

            for person in people:
                if operator_found:
                    break
                # Approach person
                # Ask if they are the operator
                # if operator:
                # operator_found = True

            self.current_state = HandTM.TaskStates.PICK_OBJECT

        if self.current_state == HandTM.TaskStates.PICK_OBJECT:
            Logger.state(self, "Grabbing object from user")

            self.current_state = HandTM.TaskStates.ASK_OBJECT

        if self.current_state == HandTM.TaskStates.ASK_OBJECT:
            Logger.state(self, "Asking where to place object")
            self.current_state = HandTM.TaskStates.PLACE_OBJECT

        if self.current_state == HandTM.TaskStates.PLACE_OBJECT:
            Logger.state(self, "Placing an object")
            self.navigate_to("kitchen", "beverages")
            self.current_state = HandTM.TaskStates.RETURN_TO_OPERATOR

        if self.current_state == HandTM.TaskStates.RETURN_TO_OPERATOR:
            Logger.state(self, "Returning to operator")
            if self.handed_objects < 5:
                self.current_state = HandTM.TaskStates.PICK_OBJECT
            else:
                self.current_state = HandTM.TaskStates.END

        if self.current_state == HandTM.TaskStates.END:
            Logger.state(self, "Task finished")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = HandTM()

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

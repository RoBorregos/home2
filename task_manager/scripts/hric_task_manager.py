#!/usr/bin/env python3
# TODO: Test task manager with FRIDA

"""
Task Manager for Human Robot Interaction Challenge task of Robocup @Home 2026
"""

import time
from datetime import datetime

import rclpy
from frida_constants.vision_constants import FACE_RECOGNITION_IMAGE, IMAGE_TOPIC_HRIC
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
FIRST_GUEST_IDX = 0
SECOND_GUEST_IDX = 1


class Guest:
    """Class to manage the guest information"""

    def __init__(self, name: str = None, drink: str = None):
        """Initialize the class"""
        self.name = name
        self.drink = drink
        self.description = ""

    def __str__(self):
        """Return the string representation of the class"""
        return f"Name: {self.name}, Drink: {self.drink}"


class HRIC_TM(Node):
    """Human Robot Interaction Challenge Task Manager"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        WAIT_FOR_GUEST = "WAIT_FOR_GUEST"
        GREETING = "GREETING"
        SAVE_FACE = "SAVE_FACE"
        TAKE_BAG = "TAKE_BAG"
        NAVIGATE_TO_LIVING_ROOM = "NAVIGATE_TO_LIVING_ROOM"
        FIND_SEAT = "FIND_SEAT"
        INTRODUCTION = "INTRODUCTION"
        NAVIGATE_TO_ENTRANCE = "NAVIGATE_TO_ENTRANCE"
        TAKE_BAG = "TAKE_BAG"
        END = "END"
        DEBUG = "DEBUG"

    def __init__(self):
        """Initialize the node"""
        super().__init__("hric_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.HRIC, mock_areas=[])

        # ACTION REQUIRED: Adjust the following variables according to the actual context
        self.seat_angles = [0, -90]
        self.check_angles = [0, -10, 10]

        self.guests = [Guest() for _ in range(2)]
        self.current_guest_idx = 0
        self.current_attempts = 0
        self.running_task = True
        self.message = ""

        # State timing variables
        self.state_start_time = None
        self.state_times = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        self.current_state = HRIC_TM.TaskStates.WAIT_FOR_BUTTON
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "HRICTaskManager has started.")

    def get_current_guest(self) -> Guest:
        return self.guests[self.current_guest_idx]

    def _track_state_change(self, new_state: str):
        """Track state changes and time spent in each state"""
        current_time = datetime.now()

        # If we have a previous state, calculate time spent
        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent

            Logger.info(self, f"State '{self.previous_state}' took {time_spent:.2f} seconds")

        # Update for new state
        self.previous_state = new_state
        self.state_start_time = current_time

        # Log cumulative times
        if self.state_times:
            total_time = sum(self.state_times.values())
            Logger.info(self, f"Total time elapsed: {total_time:.2f} seconds")
            Logger.info(self, f"State breakdown: {self.state_times}")

        Logger.state(self, self.current_state)

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        self.subtask_manager.nav.resume_nav()
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(
                f"I'll guide you to the {location}. Take a step back and please follow me.",
                wait=False,
            )
        result = Status.EXECUTION_ERROR
        retry = 0
        while result == Status.EXECUTION_ERROR and retry < ATTEMPT_LIMIT:
            future = self.subtask_manager.nav.move_to_location(location, sublocation)
            if "navigation" not in self.subtask_manager.get_mocked_areas():
                rclpy.spin_until_future_complete(self, future)
                result = future.result()

            retry += 1
        self.subtask_manager.nav.pause_nav()

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def set_description(self, status, description: str):
        self.get_current_guest().description = description

    def run(self):
        """Finite State Machine"""

        if self.current_state == HRIC_TM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")

            # Wait for the start button to be pressed
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(
                self, "Start button pressed, Human Robot Interaction Challenge task will begin now"
            )
            self.current_state = HRIC_TM.TaskStates.START

        if self.current_state == HRIC_TM.TaskStates.START:
            self._track_state_change(HRIC_TM.TaskStates.START)
            self.navigate_to("entrance", say=False)
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.current_state = HRIC_TM.TaskStates.WAIT_FOR_GUEST

        if self.current_state == HRIC_TM.TaskStates.WAIT_FOR_GUEST:
            self._track_state_change(HRIC_TM.TaskStates.WAIT_FOR_GUEST)
            self.subtask_manager.manipulation.move_to_position("front_stare")
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_HRIC)
            result = self.subtask_manager.vision.detect_person(timeout=10)

            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.vision.follow_by_name("area")
                self.subtask_manager.manipulation.follow_face(True)
                self.current_state = HRIC_TM.TaskStates.GREETING
            else:
                self.subtask_manager.hri.say("I am waiting for a guest.")

        if self.current_state == HRIC_TM.TaskStates.GREETING:
            self._track_state_change(HRIC_TM.TaskStates.GREETING)
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            status, name = self.subtask_manager.hri.ask_and_confirm(
                question="What is your name?",
                query="name",
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_current_guest().name = name
            else:
                self.get_current_guest().name = f"Guest {self.current_guest_idx + 1}"

            status, drink = self.subtask_manager.hri.ask_and_confirm(
                question="What is your favorite drink?",
                query="drink",
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_current_guest().drink = drink
            else:
                self.get_current_guest().drink = "Unknown"

            Logger.info(
                self,
                f"Guest name: {self.get_current_guest().name}, Guest drink: {self.get_current_guest().drink}",
            )
            self.subtask_manager.hri.say(f"Nice to meet you, {self.get_current_guest().name}.")
            self.current_attempts = 0
            self.current_state = HRIC_TM.TaskStates.SAVE_FACE

        if self.current_state == HRIC_TM.TaskStates.SAVE_FACE:
            self._track_state_change(HRIC_TM.TaskStates.SAVE_FACE)
            self.subtask_manager.hri.say(
                "Please stand in front of me so I can save your face."
                if self.current_attempts == 0
                else "Please get closer to me and look at my camera so I can save your face."
            )
            result = self.subtask_manager.vision.save_face_name(self.get_current_guest().name)

            if result == Status.EXECUTION_SUCCESS or self.current_attempts >= ATTEMPT_LIMIT:
                self.subtask_manager.vision.describe_person(self.set_description)
                self.subtask_manager.hri.say("I have saved your face.")
                self.current_attempts = 0
                if self.current_guest_idx == FIRST_GUEST_IDX:
                    self.current_state = HRIC_TM.TaskStates.NAVIGATE_TO_LIVING_ROOM
                else:
                    self.current_state = HRIC_TM.TaskStates.TAKE_BAG
            else:
                self.current_attempts += 1
                Logger.error(self, "Error saving face")

        if self.current_state == HRIC_TM.TaskStates.TAKE_BAG:
            self._track_state_change(HRIC_TM.TaskStates.TAKE_BAG)
            self.subtask_manager.hri.say(
                # TODO: uncomment and delete next line
                # "I see you brought a bag for the host. Please let me take care of it for you.",
                "Sorry, I currently can't take care of your bag.",
                wait=False,
            )
            # TODO: Implement bag taking functionality here
            guest_1 = self.guests[FIRST_GUEST_IDX]
            self.subtask_manager.hri.say(
                f"Please follow me to the living room. By the way, another guest named {guest_1.name} is already in the living room. {guest_1.description}.",
                wait=False,
            )
            self.current_state = HRIC_TM.TaskStates.NAVIGATE_TO_LIVING_ROOM

        if self.current_state == HRIC_TM.TaskStates.NAVIGATE_TO_LIVING_ROOM:
            self._track_state_change(HRIC_TM.TaskStates.NAVIGATE_TO_LIVING_ROOM)
            self.navigate_to("living_room", "couches", say=False)
            self.current_state = HRIC_TM.TaskStates.FIND_SEAT

        if self.current_state == HRIC_TM.TaskStates.FIND_SEAT:
            self._track_state_change(HRIC_TM.TaskStates.FIND_SEAT)
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_HRIC)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_low_stare", velocity=0.5, degrees=True
            )
            angle = 0

            for seat_angle in self.seat_angles:
                self.subtask_manager.manipulation.pan_to(seat_angle)
                self.timeout(1)
                status, angle = self.subtask_manager.vision.find_seat()
                if status == Status.EXECUTION_SUCCESS:
                    break

            self.subtask_manager.manipulation.pan_to(angle)
            self.subtask_manager.manipulation.point(10)
            self.subtask_manager.hri.say("Please take a seat where my arm points at.")
            if self.current_guest_idx == FIRST_GUEST_IDX:
                self.current_state = HRIC_TM.TaskStates.NAVIGATE_TO_ENTRANCE
            else:
                self.current_state = HRIC_TM.TaskStates.INTRODUCTION

        if self.current_state == HRIC_TM.TaskStates.INTRODUCTION:
            self._track_state_change(HRIC_TM.TaskStates.INTRODUCTION)
            guest_1 = self.guests[FIRST_GUEST_IDX]
            guest_2 = self.get_current_guest()
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            self.subtask_manager.manipulation.move_to_position("front_low_stare")
            self.subtask_manager.vision.follow_by_name(guest_1.name)
            self.subtask_manager.hri.say(
                f"Thanks for taking a seat {guest_2.name}. Allow me to introduce you to {guest_1.name}, they're favorite drink is {guest_1.drink}.",
                wait=True,
            )
            person_found = False

            for seat_angle in self.seat_angles:
                if person_found:
                    break

                self.subtask_manager.manipulation.pan_to(seat_angle)
                self.current_attempts = 0

                while self.current_attempts < ATTEMPT_LIMIT:
                    result = self.subtask_manager.vision.isPerson(guest_2.name)
                    if result:
                        person_found = True
                        break

                    self.timeout(1)
                    self.current_attempts += 1

            self.subtask_manager.manipulation.follow_face(True)
            self.subtask_manager.hri.say(
                f"Hello {guest_2.name}. This is {guest_1.name} and they're favorite drink is {guest_1.drink}"
            )
            self.subtask_manager.manipulation.follow_face(False)

            self.current_state = HRIC_TM.TaskStates.TAKE_BAG

        if self.current_state == HRIC_TM.TaskStates.NAVIGATE_TO_ENTRANCE:
            self._track_state_change(HRIC_TM.TaskStates.NAVIGATE_TO_ENTRANCE)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.current_guest_idx = SECOND_GUEST_IDX
            self.navigate_to("entrance", say=False)
            self.current_state = HRIC_TM.TaskStates.WAIT_FOR_GUEST

        if self.current_state == HRIC_TM.TaskStates.TAKE_BAG:
            self._track_state_change(HRIC_TM.TaskStates.TAKE_BAG)
            # TODO: Follow the host and leave the bag
            self.current_state = HRIC_TM.TaskStates.END

        if self.current_state == HRIC_TM.TaskStates.END:
            Logger.state(self, "Ending task")
            self._track_state_change(HRIC_TM.TaskStates.END)

            # Generate final timing report
            total_task_time = (datetime.now() - self.total_start_time).total_seconds()
            Logger.info(self, "=== FINAL TIMING REPORT ===")
            Logger.info(self, f"Total task time: {total_task_time:.2f} seconds")

            # Sort states by time spent (descending)
            sorted_states = sorted(self.state_times.items(), key=lambda x: x[1], reverse=True)

            for state, time_spent in sorted_states:
                percentage = (time_spent / total_task_time) * 100
                Logger.info(self, f"{state}: {time_spent:.2f}s ({percentage:.1f}%)")

            Logger.info(self, "=== END TIMING REPORT ===")

            self.subtask_manager.hri.say("I have finished my task, I will rest now.")
            self.subtask_manager.manipulation.follow_face(False)
            self.running_task = False

        if self.current_state == HRIC_TM.TaskStates.DEBUG:
            Logger.state(self, "Debugging task")
            self._track_state_change(HRIC_TM.TaskStates.DEBUG)
            self.subtask_manager.hri.say("Debugging task.")
            self.current_state = HRIC_TM.TaskStates.END


def main(args=None):
    rclpy.init(args=args)
    node = HRIC_TM()

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

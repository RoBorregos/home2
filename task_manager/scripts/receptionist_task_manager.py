#!/usr/bin/env python3

"""
Task Manager for Receptionist task of Robocup @Home 2025
"""

import rclpy

from rclpy.node import Node
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task
from utils.status import Status
import time


from frida_constants.vision_constants import (
    IMAGE_TOPIC_RECEPTIONIST,
    FACE_RECOGNITION_IMAGE,
)

ATTEMPT_LIMIT = 3
START = "START"


class Guest:
    """Class to manage the guest information"""

    def __init__(self, name: str = None, drink: str = None, interest: str = None):
        """Initialize the class"""
        self.name = name
        self.drink = drink
        self.interest = interest
        self.description = ""

    def __str__(self):
        """Return the string representation of the class"""
        return f"Name: {self.name}, Drink: {self.drink}, Interest: {self.interest}"


class ReceptionistTM(Node):
    """Class to manage the receptionist task"""

    TASK_STATES = {
        "START": 0,
        "WAIT_FOR_GUEST": 1,
        "GREETING": 2,
        "SAVE_FACE": 3,
        "NAVIGATE_TO_BEVERAGES": 4,
        "ASK_FOR_DRINK": 5,
        "DRINK_AVAILABLE": 6,
        "ASK_FOR_INTEREST": 7,
        "NAVIGATE_TO_LEAVING_ROOM": 8,
        "FIND_SEAT": 9,
        "INTRODUCTION": 10,
        "NAVIGATE_TO_ENTRANCE": 11,
        "END": 12,
        "DEBUG": 13,
        "DESCRIBE": 14,
    }

    def __init__(self):
        """Initialize the node"""
        super().__init__("receptionist_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.RECEPTIONIST, mock_areas=["navigation"]
        )
        self.current_state = ReceptionistTM.TASK_STATES[START]
        self.current_guest = 1
        self.seat_angles = [0, 90]
        self.check_angles = [-10, 10, 10]

        self.guests = [Guest() for _ in range(3)]
        self.guests[0] = Guest("ale", "Juice", "Football")

        self.current_attempts = 0
        self.running_task = True
        self.message = ""

        Logger.info(self, "ReceptionistTaskManager has started.")

    def get_guest(self) -> Guest:
        """Get the current guest"""
        return self.guests[self.current_guest]

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.manipulation.move_to_position("nav_pose")
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self, future)

    def confirm(self, statement: str) -> bool:
        """Confirm the name is correct"""
        self.subtask_manager.hri.say(f"I heard {statement}, is that correct?")
        response = self.subtask_manager.hri.hear()
        return self.subtask_manager.hri.is_positive(response)

    def hear_word(self, word: str) -> bool:
        """Check if the word is heard"""
        status, statement = self.subtask_manager.hri.hear()
        status, name = self.subtask_manager.hri.extract_data(word, statement)
        if name is None:
            name = statement
        return name

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def set_description(self, status, description: str):
        self.get_guest().description = description

    def run(self):
        """State machine"""

        if self.current_state == ReceptionistTM.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.subtask_manager.manipulation.follow_face(False)
            self.navigate_to("entrance", say=False)
            self.subtask_manager.hri.say("I am ready to start my task.", wait=False)
            self.current_state = ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]

        if self.current_state == ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]:
            Logger.state(self, "Waiting for guest")

            self.subtask_manager.manipulation.move_to_position("front_stare")
            self.subtask_manager.hri.say(
                "I am ready to receive guests, please open the door.", wait=True
            )

            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_RECEPTIONIST)
            result = self.subtask_manager.vision.detect_person(timeout=10)
            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.vision.follow_by_name("area")
                self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say("Hello, I am FRIDA, your receptionist today.")
                self.current_state = ReceptionistTM.TASK_STATES["GREETING"]
            else:
                self.subtask_manager.hri.say("I am waiting for a guest.")

        if self.current_state == ReceptionistTM.TASK_STATES["GREETING"]:
            Logger.state(self, "Greeting guest")
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            status, name = self.subtask_manager.hri.ask_and_confirm(
                question="What is your name?", query="name", use_hotwords=False
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().name = name
            else:
                self.get_guest().name = f"Guest {self.current_guest}"

            Logger.info(self, f"Guest name: {self.get_guest().name}")
            self.subtask_manager.hri.say(f"Nice to meet you, {self.get_guest().name}.")
            self.current_attempts = 0
            self.current_state = ReceptionistTM.TASK_STATES["SAVE_FACE"]

        if self.current_state == ReceptionistTM.TASK_STATES["SAVE_FACE"]:
            Logger.state(self, "Saving face")
            self.subtask_manager.hri.say("I will save your face now. Please stand in front of me")
            result = self.subtask_manager.vision.save_face_name(self.get_guest().name)
            self.timeout(2)

            if result == Status.EXECUTION_SUCCESS or self.current_attempts >= ATTEMPT_LIMIT:
                self.subtask_manager.vision.describe_person(self.set_description)
                self.subtask_manager.hri.say("I have saved your face.")
                self.current_attempts = 0
                self.current_state = ReceptionistTM.TASK_STATES["ASK_FOR_INTEREST"]
            else:
                self.current_attempts += 1
                self.subtask_manager.hri.say("Please get closer to me and look at my camera.")
                Logger.error(self, "Error saving face")

        if self.current_state == ReceptionistTM.TASK_STATES["ASK_FOR_INTEREST"]:
            Logger.state(self, "Asking for interest")

            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
            self.subtask_manager.manipulation.follow_face(True)

            status, interest = self.subtask_manager.hri.ask_and_confirm(
                question="What is your main interest?", query="interest", use_hotwords=False
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().interest = interest
            else:
                self.get_guest().interest = "Nothing"

            Logger.info(self, f"Interest: {self.get_guest().interest}")

            self.subtask_manager.hri.say(
                f"Thank you for sharing your interest in {self.get_guest().interest}."
            )

            if self.current_guest == 1:
                self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_BEVERAGES"]
            else:
                self.current_state = ReceptionistTM.TASK_STATES["DESCRIBE"]

        if self.current_state == ReceptionistTM.TASK_STATES["DESCRIBE"]:
            guest1 = self.guests[1]
            self.subtask_manager.hri.say(
                f"By the way, {guest1.name} is already in the living room. {guest1.description}",
                wait=False,
            )

            status, common_message_guest1 = self.subtask_manager.hri.common_interest(
                self.get_guest().name, self.get_guest().interest, guest1.name, guest1.interest
            )

            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"{common_message_guest1}")
            else:
                host = self.guests[0]
                status, common_message_host = self.subtask_manager.hri.common_interest(
                    self.get_guest().name, self.get_guest().interest, host.name, host.interest
                )
                self.subtask_manager.hri.say(
                    f"{host.name} is also in the living room. {common_message_host}"
                )

            self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_BEVERAGES"]

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_BEVERAGES"]:
            Logger.state(self, "Navigating to beverages")
            self.navigate_to("kitchen", "beverages")
            self.current_state = ReceptionistTM.TASK_STATES["ASK_FOR_DRINK"]

        if self.current_state == ReceptionistTM.TASK_STATES["ASK_FOR_DRINK"]:
            Logger.state(self, "Asking for drink")
            self.subtask_manager.manipulation.follow_face(True)

            status, drink = self.subtask_manager.hri.ask_and_confirm(
                question="What is your favorite drink?", query="drink", use_hotwords=False
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().drink = drink
            else:
                self.get_guest().drink = "Water"

            Logger.info(self, f"Guest drink: {self.get_guest().drink}")

            self.subtask_manager.hri.say(
                f"Great! I will check if we have {self.get_guest().drink}."
            )
            self.current_state = ReceptionistTM.TASK_STATES["DRINK_AVAILABLE"]

        if self.current_state == ReceptionistTM.TASK_STATES["DRINK_AVAILABLE"]:
            Logger.state(self, "Checking drink availability")
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("table_stare")
            # self.subtask_manager.manipulation.move_joint_positions(
            #     named_position="table_stare", velocity=0.5, degrees=True
            # )
            status, position = self.subtask_manager.vision.find_drink(
                self.get_guest().drink, timeout=40
            )

            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    f"There is {self.get_guest().drink} at the table in the {position}."
                )
            else:
                self.subtask_manager.hri.say(f"Sorry, we do not have {self.get_guest().drink}.")
            self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_LEAVING_ROOM"]

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_LEAVING_ROOM"]:
            Logger.state(self, "Navigating to leaving room")
            self.navigate_to("living_room", "couches")
            self.current_state = ReceptionistTM.TASK_STATES["FIND_SEAT"]

        if self.current_state == ReceptionistTM.TASK_STATES["FIND_SEAT"]:
            Logger.state(self, "Finding seat")
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_RECEPTIONIST)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_low_stare", velocity=0.5, degrees=True
            )

            for seat_angle in self.seat_angles:
                joint_positions = self.subtask_manager.manipulation.get_joint_positions(
                    degrees=True
                )
                joint_positions["joint1"] = joint_positions["joint1"] - seat_angle
                self.subtask_manager.manipulation.move_joint_positions(
                    joint_positions=joint_positions, velocity=0.5, degrees=True
                )
                self.timeout(1)
                status, angle = self.subtask_manager.vision.find_seat()
                if status == Status.EXECUTION_SUCCESS:
                    break

            self.subtask_manager.hri.say("Please take a seat where my arm points at.", wait=False)
            self.subtask_manager.manipulation.pan_to(angle)
            self.timeout(2)
            self.current_state = ReceptionistTM.TASK_STATES["INTRODUCTION"]

        if self.current_state == ReceptionistTM.TASK_STATES["INTRODUCTION"]:
            Logger.state(self, "Introducing guest")
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            self.subtask_manager.manipulation.move_to_position("front_low_stare")
            self.subtask_manager.manipulation.follow_face(True)

            if self.current_guest == 1:
                host = self.guests[0]
                self.current_attempts = 0
                while self.current_attempts < ATTEMPT_LIMIT:
                    result = self.subtask_manager.vision.follow_by_name(host.name)
                    if result == Status.EXECUTION_SUCCESS:
                        break
                    self.subtask_manager.manipulation.pan_to(
                        self.check_angles[self.current_attempts]
                    )
                    self.current_attempts += 1

                self.subtask_manager.hri.say(f"Hello {host.name}. This is {self.get_guest().name}")
            else:
                guest1 = self.guests[1]
                self.current_attempts = 0
                while self.current_attempts < ATTEMPT_LIMIT:
                    result = self.subtask_manager.vision.follow_by_name(guest1.name)
                    if result == Status.EXECUTION_SUCCESS:
                        break
                    self.subtask_manager.manipulation.pan_to(
                        self.check_angles[self.current_attempts]
                    )
                    self.current_attempts += 1
                self.subtask_manager.hri.say(
                    f"Hello {guest1.name}. This is {self.get_guest().name} and they like {self.get_guest().drink}"
                )

            self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_ENTRANCE"]

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_ENTRANCE"]:
            Logger.state(self, "Navigating to entrance")
            self.current_guest += 1
            if self.current_guest == 3:
                self.current_state = ReceptionistTM.TASK_STATES["END"]
            else:
                self.navigate_to("entrance", say=False)
                self.current_state = ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]

        if self.current_state == ReceptionistTM.TASK_STATES["END"]:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("I have finished my task, I will rest now.")
            self.subtask_manager.manipulation.follow_face(False)
            self.running_task = False

        if self.current_state == ReceptionistTM.TASK_STATES["DEBUG"]:
            Logger.state(self, "Debugging task")
            self.subtask_manager.hri.say("Debugging task.")
            self.current_state = ReceptionistTM.TASK_STATES["END"]


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = ReceptionistTM()

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

#!/usr/bin/env python3

"""
Task Manager for Receptionist task of Robocup @Home 2025
"""

import rclpy

from rclpy.node import Node
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task
from utils.status import Status
from frida_constants.xarm_configurations import XARM_CONFIGURATIONS

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

        self.guests = [Guest() for _ in range(3)]
        self.guests[0] = Guest("John", "Beer", "Football")

        self.current_attempts = 0
        self.running_task = True

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
            # self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
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

    # TODO (@alecoeto): detect guest
    # TODO (@joce): improve prompt for person description

    def run(self):
        """State machine"""

        if self.current_state == ReceptionistTM.TASK_STATES["DEBUG"]:
            Logger.state(self, "Debugging task")
            self.subtask_manager.hri.say("Debugging task.")
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

        if self.current_state == ReceptionistTM.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.navigate_to("entrance", say=False)
            self.subtask_manager.hri.say("I am ready to start my task.")
            self.current_state = ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]

        if self.current_state == ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]:
            Logger.state(self, "Waiting for guest")

            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

            self.subtask_manager.hri.say(
                "I am ready to receive guests, please open the door.", wait=True
            )

            result = self.subtask_manager.vision.detect_person(timeout=10)
            if result == Status.EXECUTION_SUCCESS:
                # self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say("Hello, I am FRIDA, your receptionist today.")
                self.current_state = ReceptionistTM.TASK_STATES["GREETING"]
            else:
                self.subtask_manager.hri.say("I am waiting for a guest.")

        if self.current_state == ReceptionistTM.TASK_STATES["GREETING"]:
            Logger.state(self, "Greeting guest")

            status, name = self.subtask_manager.hri.ask_and_confirm(
                question="What is your name?", query="name", use_hotwords=False
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().name = name
            else:
                self.get_guest().name = f"Guest {self.current_guest}"

            Logger.info(self, f"Guest name: {self.get_guest().name}")
            # if self.current_attempts >= ATTEMPT_LIMIT:
            #     self.get_guest().name = f"Guest {self.current_guest}"
            # else:
            #     self.subtask_manager.hri.say("What is your name?", wait=True)
            #     self.get_guest().name = self.hear_word("name")
            #     Logger.info(self, f"Heard name: {self.get_guest().name}")
            #     self.current_attempts += 1

            # if self.get_guest().name is not None:
            # if self.get_guest().name == f"Guest {self.current_guest}" or self.confirm(
            #     self.get_guest().name
            # ):
            self.subtask_manager.hri.say(f"Nice to meet you, {self.get_guest().name}.")
            self.current_attempts = 0
            self.current_state = ReceptionistTM.TASK_STATES["SAVE_FACE"]

        if self.current_state == ReceptionistTM.TASK_STATES["SAVE_FACE"]:
            Logger.state(self, "Saving face")
            result = self.subtask_manager.vision.save_face_name(self.get_guest().name)
            self.get_guest().description = self.subtask_manager.vision.describe_person()

            if result == Status.EXECUTION_SUCCESS or self.current_attempts >= ATTEMPT_LIMIT:
                self.subtask_manager.hri.say("I have saved your face.")
                self.current_attempts = 0
                self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_BEVERAGES"]
            else:
                self.current_attempts += 1
                Logger.error(self, "Error saving face")

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_BEVERAGES"]:
            Logger.state(self, "Navigating to beverages")
            self.navigate_to("kitchen", "beverages")
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
            self.current_state = ReceptionistTM.TASK_STATES["ASK_FOR_DRINK"]

        if self.current_state == ReceptionistTM.TASK_STATES["ASK_FOR_DRINK"]:
            Logger.state(self, "Asking for drink")
            # self.subtask_manager.manipulation.follow_face(True)

            status, drink = self.subtask_manager.hri.ask_and_confirm(
                question="What is your favorite drink?", query="drink", use_hotwords=False
            )

            # if self.current_attempts >= ATTEMPT_LIMIT:
            #     self.get_guest().drink = "Water"
            # else:
            #     self.subtask_manager.hri.say("What is your favorite drink?")
            #     self.get_guest().drink = self.hear_word("drink")
            #     self.current_attempts += 1

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
            # self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            # self.subtask_manager.manipulation.move_to_position("table")
            status, position = self.subtask_manager.vision.find_drink(
                self.get_guest().drink, timeout=40
            )
            # self.subtask_manager.manipulation.move_to_position("gaze")
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    f"There is {self.get_guest().drink} at the table in the {position}."
                )
            else:
                self.subtask_manager.hri.say(f"Sorry, we do not have {self.get_guest().drink}.")
            self.current_state = ReceptionistTM.TASK_STATES["ASK_FOR_INTEREST"]

        if self.current_state == ReceptionistTM.TASK_STATES["ASK_FOR_INTEREST"]:
            Logger.state(self, "Asking for interest")

            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

            status, interest = self.subtask_manager.hri.ask_and_confirm(
                question="What is your main interest?", query="interest", use_hotwords=False
            )

            # if self.current_attempts >= ATTEMPT_LIMIT:
            #     self.get_guest().interest = "Nothing"
            # else:
            #     # self.subtask_manager.manipulation.follow_face(True)
            #     self.subtask_manager.hri.say("What is your main interest?")
            #     self.get_guest().interest = self.hear_word("interest")
            #     self.current_attempts += 1

            # if self.get_guest().interest is not None:
            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().interest = interest
            else:
                self.get_guest().interest = "Nothing"

            Logger.info(self, f"Interest: {self.get_guest().interest}")
            # if self.confirm(self.get_guest().interest):
            #     self.current_attempts = 0
            self.subtask_manager.hri.say(
                f"Thank you for sharing your interest in {self.get_guest().interest}."
            )
            self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_LEAVING_ROOM"]

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_LEAVING_ROOM"]:
            Logger.state(self, "Navigating to leaving room")
            self.navigate_to("living_room", "couches")
            self.current_state = ReceptionistTM.TASK_STATES["FIND_SEAT"]

        if self.current_state == ReceptionistTM.TASK_STATES["FIND_SEAT"]:
            Logger.state(self, "Finding seat")
            # self.subtask_manager.manipulation.move_to_position("find_seat")
            target = 0
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
            for seat_angle in self.seat_angles:
                joint_positions = self.subtask_manager.manipulation.get_joint_positions(
                    degrees=True
                )
                joint_positions["joint1"] = joint_positions["joint1"] - seat_angle
                self.subtask_manager.manipulation.move_joint_positions(
                    joint_positions=joint_positions, velocity=0.5, degrees=True
                )
                # self.subtask_manager.manipulation.pan_to(seat_angle)
                status, angle = self.subtask_manager.vision.find_seat()
                if status == Status.EXECUTION_SUCCESS:
                    target = angle
                    break

            self.subtask_manager.hri.say("Please take a seat where my arm points at.")
            joint_positions = self.subtask_manager.manipulation.get_joint_positions(degrees=True)
            joint_positions["joint1"] = joint_positions["joint1"] - target
            self.subtask_manager.manipulation.move_joint_positions(
                joint_positions=joint_positions, velocity=0.5, degrees=True
            )

            # self.subtask_manager.vision.detect_guest(self.get_guest().name, timeout=5)
            self.current_state = ReceptionistTM.TASK_STATES["INTRODUCTION"]

        if self.current_state == ReceptionistTM.TASK_STATES["INTRODUCTION"]:
            Logger.state(self, "Introducing guest")
            # self.subtask_manager.manipulation.move_to_position("gaze")
            # self.subtask_manager.manipulation.follow_face(True)

            for guest in self.guests:
                if guest.name is None or guest == self.get_guest():
                    continue
                self.subtask_manager.vision.follow_by_name(guest.name)
                self.subtask_manager.hri.say(
                    f"Hello {guest.name}. This is {self.get_guest().name}. {self.get_guest().description} and they like {self.get_guest().drink}."
                )

                status, common_message = self.subtask_manager.hri.common_interest(
                    self.get_guest().name, self.get_guest().interest, guest.name, guest.interest
                )
                self.subtask_manager.hri.say(common_message)

            self.current_state = ReceptionistTM.TASK_STATES["NAVIGATE_TO_ENTRANCE"]

        if self.current_state == ReceptionistTM.TASK_STATES["NAVIGATE_TO_ENTRANCE"]:
            Logger.state(self, "Navigating to entrance")
            self.navigate_to("entrance")
            self.current_guest += 1
            if self.current_guest == 3:
                self.current_state = ReceptionistTM.TASK_STATES["END"]
            else:
                self.current_state = ReceptionistTM.TASK_STATES["WAIT_FOR_GUEST"]

        if self.current_state == ReceptionistTM.TASK_STATES["END"]:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("I have finished my task, I will rest now.")
            self.running_task = False


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

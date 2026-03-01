#!/usr/bin/env python3

"""
Task Manager for Receptionist task of Robocup @Home 2025
"""

import time
from datetime import datetime

import rclpy
from frida_constants.vision_constants import FACE_RECOGNITION_IMAGE, IMAGE_TOPIC_RECEPTIONIST
from rclpy.node import Node
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3


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


def mock_common_interest(person1, interest1, person2, interest2, remove_thinking=True):
    """Pick a random word from keywords"""

    return Status.MOCKED, f"{person1} and {person2} both like {interest1}"


class ReceptionistTM(Node):
    """Class to manage the receptionist task"""

    class TaskStates:
        """Class to manage the task states"""

        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        WAIT_FOR_GUEST = "WAIT_FOR_GUEST"
        GREETING = "GREETING"
        SAVE_FACE = "SAVE_FACE"
        NAVIGATE_TO_BEVERAGES = "NAVIGATE_TO_BEVERAGES"
        ASK_FOR_DRINK = "ASK_FOR_DRINK"
        DRINK_AVAILABLE = "DRINK_AVAILABLE"
        ASK_FOR_INTEREST = "ASK_FOR_INTEREST"
        NAVIGATE_TO_LEAVING_ROOM = "NAVIGATE_TO_LEAVING_ROOM"
        FIND_SEAT = "FIND_SEAT"
        INTRODUCTION = "INTRODUCTION"
        NAVIGATE_TO_ENTRANCE = "NAVIGATE_TO_ENTRANCE"
        END = "END"
        DEBUG = "DEBUG"
        DESCRIBE = "DESCRIBE"

    def __init__(self):
        """Initialize the node"""
        super().__init__("receptionist_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.RECEPTIONIST, mock_areas=[])
        # self.seat_angles = [0, -90, 180]
        self.seat_angles = [0, -90]
        self.check_angles = [0, -10, 10]

        self.guests = [Guest() for _ in range(3)]
        self.guests[0] = Guest("Ale", "Juice", "Football")
        self.current_guest = 1

        self.current_attempts = 0
        self.running_task = True
        self.message = ""

        # State timing variables
        self.state_start_time = None
        self.state_times = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        self.current_state = ReceptionistTM.TaskStates.WAIT_FOR_BUTTON
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "ReceptionistTaskManager has started.")

    def get_guest(self) -> Guest:
        """Get the current guest"""
        return self.guests[self.current_guest]

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
        self.get_guest().description = description

    def run(self):
        """State machine"""

        if self.current_state == ReceptionistTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.")
            # Wait for the start button to be pressed
            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            self.current_state = ReceptionistTM.TaskStates.START

        if self.current_state == ReceptionistTM.TaskStates.START:
            Logger.state(self, "Starting task")
            self._track_state_change(ReceptionistTM.TaskStates.START)
            # self.subtask_manager.manipulation.follow_face(False)
            # self.subtask_manager.manipulation.move_to_position("nav_pose")
            # import sys
            # sys.exit()
            self.navigate_to("entrance", say=False)
            self.subtask_manager.hri.say("I am ready.", wait=False)
            self.current_state = ReceptionistTM.TaskStates.WAIT_FOR_GUEST

        if self.current_state == ReceptionistTM.TaskStates.WAIT_FOR_GUEST:
            Logger.state(self, "Waiting for guest")
            self._track_state_change(ReceptionistTM.TaskStates.WAIT_FOR_GUEST)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("front_stare")
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_RECEPTIONIST)
            result = self.subtask_manager.vision.detect_person(timeout=10)
            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.vision.follow_by_name("area")
                self.subtask_manager.manipulation.follow_face(True)
                self.current_state = ReceptionistTM.TaskStates.GREETING
            else:
                self.subtask_manager.hri.say("I am waiting for a guest.")

        if self.current_state == ReceptionistTM.TaskStates.GREETING:
            Logger.state(self, "Greeting guest")
            self._track_state_change(ReceptionistTM.TaskStates.GREETING)
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            status, name = self.subtask_manager.hri.ask_and_confirm(
                question="What is your name?",
                query="name",
                use_hotwords=False,
                hotwords="Here are some available names: "
                + " ".join(self.subtask_manager.hri.names),
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().name = name
            else:
                self.get_guest().name = f"Guest {self.current_guest}"

            Logger.info(self, f"Guest name: {self.get_guest().name}")
            self.subtask_manager.hri.say(f"Nice to meet you, {self.get_guest().name}.")
            self.current_attempts = 0
            self.current_state = ReceptionistTM.TaskStates.SAVE_FACE

        if self.current_state == ReceptionistTM.TaskStates.SAVE_FACE:
            Logger.state(self, "Saving face")
            self._track_state_change(ReceptionistTM.TaskStates.SAVE_FACE)
            self.subtask_manager.hri.say("I'll save your face. Please stand in front of me")
            result = self.subtask_manager.vision.save_face_name(self.get_guest().name)
            # self.timeout(2)

            if result == Status.EXECUTION_SUCCESS or self.current_attempts >= ATTEMPT_LIMIT:
                self.subtask_manager.vision.describe_person(self.set_description)
                self.subtask_manager.hri.say("I have saved your face.")
                self.current_attempts = 0
                self.current_state = ReceptionistTM.TaskStates.ASK_FOR_INTEREST
            else:
                self.current_attempts += 1
                self.subtask_manager.hri.say("Please get closer to me and look at my camera.")
                Logger.error(self, "Error saving face")

        if self.current_state == ReceptionistTM.TaskStates.ASK_FOR_INTEREST:
            Logger.state(self, "Asking for interest")
            # self._track_state_change(ReceptionistTM.TaskStates.ASK_FOR_INTEREST)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
            self.subtask_manager.manipulation.follow_face(True)

            status, interest = self.subtask_manager.hri.ask_and_confirm(
                question="What is your main interest?", query="LLM_interest", use_hotwords=False
            )

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().interest = interest
            else:
                self.get_guest().interest = "None"

            Logger.info(self, f"Interest: {self.get_guest().interest}")

            self.subtask_manager.hri.say(
                f"Thank you for sharing your interest in {self.get_guest().interest}."
            )

            if self.current_guest == 1:
                self.current_state = ReceptionistTM.TaskStates.NAVIGATE_TO_BEVERAGES
            else:
                self.current_state = ReceptionistTM.TaskStates.DESCRIBE

        if self.current_state == ReceptionistTM.TaskStates.DESCRIBE:
            self._track_state_change(ReceptionistTM.TaskStates.DESCRIBE)
            guest1 = self.guests[1]
            host = self.guests[0]

            common_message_guest1_future = mock_common_interest(
                self.get_guest().name,
                self.get_guest().interest,
                guest1.name,
                guest1.interest,
            )

            common_message_host_future = mock_common_interest(
                self.get_guest().name,
                self.get_guest().interest,
                host.name,
                host.interest,
            )

            self.subtask_manager.hri.say(
                f"By the way, {guest1.name} is already in the living room. {guest1.description}",
                wait=True,
            )

            rclpy.spin_until_future_complete(self, common_message_guest1_future, timeout_sec=15)
            status, common_message_guest1 = common_message_guest1_future.result()

            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"{common_message_guest1}", wait=False)
            else:
                self.subtask_manager.hri.say(
                    f"{host.name} is also in the living room.",
                    wait=True,
                )
                rclpy.spin_until_future_complete(self, common_message_host_future, timeout_sec=15)
                s, common_message_host = common_message_host_future.result()
                self.subtask_manager.hri.say(
                    f"{common_message_host}",
                    wait=False,
                )

            self.current_state = ReceptionistTM.TaskStates.NAVIGATE_TO_BEVERAGES

        if self.current_state == ReceptionistTM.TaskStates.NAVIGATE_TO_BEVERAGES:
            Logger.state(self, "Navigating to beverages")
            self._track_state_change(ReceptionistTM.TaskStates.NAVIGATE_TO_BEVERAGES)
            self.navigate_to("kitchen", "beverages")
            self.current_state = ReceptionistTM.TaskStates.ASK_FOR_DRINK

        if self.current_state == ReceptionistTM.TaskStates.ASK_FOR_DRINK:
            Logger.state(self, "Asking for drink")
            self._track_state_change(ReceptionistTM.TaskStates.ASK_FOR_DRINK)
            # self.subtask_manager.manipulation.follow_face(False)
            # self.subtask_manager.hri.say("Please stand to my right")
            # self.subtask_manager.manipulation.move_to_position("front_stare")
            # self.subtask_manager.manipulation.pan_to(90)
            # self.subtask_manager.manipulation.follow_face(True)

            status, drink = self.subtask_manager.hri.ask_and_confirm(
                question="What is your favorite drink?",
                query="LLM_drink",
                use_hotwords=False,
                hotwords="Some regional drinks are Kuat, kuat it pronounced similar to 4, but don't mistake it, understand kuat.",
                # options=[
                #  self.s.objects_data["categories"]["drink"]
                #  + ["4", "What", "Quatt", "quattre"]
                # ],
                # remap={"4": "kuat", "quattre": "kuat", "Quatt": "kuat", "What": "kuat"},
            )

            # self.subtask_manager.manipulation.follow_face(False)

            if status == Status.EXECUTION_SUCCESS:
                self.get_guest().drink = drink
            else:
                self.get_guest().drink = "none"
                self.current_state = ReceptionistTM.TaskStates.NAVIGATE_TO_LEAVING_ROOM
                return

            Logger.info(self, f"Guest drink: {self.get_guest().drink}")

            self.subtask_manager.hri.say(f"I will check if we have {self.get_guest().drink}.")
            self.current_state = ReceptionistTM.TaskStates.DRINK_AVAILABLE

        if self.current_state == ReceptionistTM.TaskStates.DRINK_AVAILABLE:
            Logger.state(self, "Checking drink availability")
            self._track_state_change(ReceptionistTM.TaskStates.DRINK_AVAILABLE)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("table_stare")
            # self.guests[1].drink = "juice"

            for i in range(5):
                s, detections = self.subtask_manager.vision.detect_objects()
                if len(detections) > 0:
                    break
                self.timeout(1)
            # print("detections:", detections)

            if len(detections) == 0:
                s = Status.TARGET_NOT_FOUND

            if s == Status.EXECUTION_SUCCESS:
                Logger.info(self, "Detected drinks with detector: " + str(detections))
                labels = self.subtask_manager.vision.get_labels(detections)
                status, drink_match = self.subtask_manager.hri.find_closest(
                    labels, self.get_guest().drink, threshold=0.4
                )
                detected_drink = drink_match.results
                if len(detected_drink) > 0:
                    detected_drink = detected_drink[0]
                    s, position = self.subtask_manager.vision.get_drink_position(
                        detections, detected_drink
                    )
                    self.subtask_manager.hri.say(
                        # f"There is {self.get_guest().drink} at the table is {position}."
                        f"There is {self.get_guest().drink} at the table. It is {position}."
                    )
                else:
                    s = Status.TARGET_NOT_FOUND
            if s != Status.EXECUTION_SUCCESS:
                Logger.info(self, "Detecting drinks with moondream")
                # Moondream backup
                status, position = self.subtask_manager.vision.find_drink(
                    self.get_guest().drink, timeout=40
                )
                if status == Status.EXECUTION_SUCCESS and position != "not found":
                    self.subtask_manager.hri.say(
                        f"There is {self.get_guest().drink} at the table is {position}."
                    )
                else:
                    self.subtask_manager.hri.say(f"Sorry, we do not have {self.get_guest().drink}.")
            self.current_state = ReceptionistTM.TaskStates.NAVIGATE_TO_LEAVING_ROOM

        if self.current_state == ReceptionistTM.TaskStates.NAVIGATE_TO_LEAVING_ROOM:
            Logger.state(self, "Navigating to leaving room")
            self._track_state_change(ReceptionistTM.TaskStates.NAVIGATE_TO_LEAVING_ROOM)
            self.navigate_to("living_room", "couches")
            self.current_state = ReceptionistTM.TaskStates.FIND_SEAT

        if self.current_state == ReceptionistTM.TaskStates.FIND_SEAT:
            Logger.state(self, "Finding seat")
            self._track_state_change(ReceptionistTM.TaskStates.FIND_SEAT)
            self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_RECEPTIONIST)
            self.subtask_manager.manipulation.follow_face(False)
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

            self.subtask_manager.hri.say("Please take a seat where my arm points at.", wait=False)
            self.subtask_manager.manipulation.pan_to(angle)
            self.subtask_manager.manipulation.point(10)
            self.timeout(4)
            self.current_state = ReceptionistTM.TaskStates.INTRODUCTION

        if self.current_state == ReceptionistTM.TaskStates.INTRODUCTION:
            Logger.state(self, "Introducing guest")
            self._track_state_change(ReceptionistTM.TaskStates.INTRODUCTION)
            self.subtask_manager.hri.publish_display_topic(FACE_RECOGNITION_IMAGE)
            # self.subtask_manager.manipulation.follow_face(True)

            # self.subtask_manager.manipulation.follow_face(False)
            host = self.guests[0]

            if self.current_guest == 1:
                # self.subtask_manager.manipulation.move_to_position("front_low_stare")
                # person_found = False
                # for seat_angle in self.seat_angles:
                #     if person_found:
                #         break

                #     self.subtask_manager.manipulation.pan_to(seat_angle)

                #     self.current_attempts = 0

                #     while self.current_attempts < ATTEMPT_LIMIT:
                #         # Comment if using an already saved host
                #         self.subtask_manager.vision.follow_by_name("Unknown")
                #         result = self.subtask_manager.vision.isPerson("Unknown")

                #         # Uncomment if using an already saved host
                #         # self.subtask_manager.vision.follow_by_name(host.name)
                #         # result = self.subtask_manager.vision.isPerson(host.name)

                #         if result:
                #             # Comment if using an already saved host
                #             self.subtask_manager.vision.save_face_name(host.name)
                #             person_found = True
                #             break
                #         self.timeout(1)
                #         # self.subtask_manager.manipulation.pan_to(
                #         #     self.check_angles[self.current_attempts]
                #         # )
                #         self.current_attempts += 1
                # self.subtask_manager.manipulation.follow_face(True)
                # self.subtask_manager.hri.say(f"Hello {host.name}. This is {self.get_guest().name}")
                # self.timeout(3)
                # # self.subtask_manager.manipulation.follow_face(False)
                self.current_state = ReceptionistTM.TaskStates.NAVIGATE_TO_ENTRANCE

            else:
                guest1 = self.guests[1]
                self.timeout(3)
                self.subtask_manager.hri.say(
                    f"Thanks for taking a seat {self.get_guest().name}. I will now introduce you to {guest1.name}.",
                    wait=True,
                )

                # self.timeout(2)
                # self.subtask_manager.manipulation.follow_face(False)
                self.subtask_manager.manipulation.move_to_position("front_low_stare")
                self.subtask_manager.vision.follow_by_name(guest1.name)

                self.subtask_manager.vision.follow_by_name(guest1.name)
                person_found = False

                for seat_angle in self.seat_angles:
                    if person_found:
                        break

                    self.subtask_manager.manipulation.pan_to(seat_angle)
                    self.current_attempts = 0

                    while self.current_attempts < ATTEMPT_LIMIT:
                        result = self.subtask_manager.vision.isPerson(guest1.name)
                        if result:
                            person_found = True
                            break

                        self.timeout(1)
                        self.current_attempts += 1

                self.subtask_manager.manipulation.follow_face(True)
                self.subtask_manager.hri.say(
                    f"Hello {guest1.name}. This is {self.get_guest().name} and they like {self.get_guest().drink}"
                )
                self.timeout(3)
                self.subtask_manager.manipulation.follow_face(False)

                self.current_state = ReceptionistTM.TaskStates.END

        if self.current_state == ReceptionistTM.TaskStates.NAVIGATE_TO_ENTRANCE:
            Logger.state(self, "Navigating to entrance")
            self._track_state_change(ReceptionistTM.TaskStates.NAVIGATE_TO_ENTRANCE)
            # self.subtask_manager.manipulation.follow_face(False)
            self.timeout(1)
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self.current_guest += 1
            if self.current_guest == 3:
                self.current_state = ReceptionistTM.TaskStates.END
            else:
                self.navigate_to("entrance", say=False)
                self.current_state = ReceptionistTM.TaskStates.WAIT_FOR_GUEST

        if self.current_state == ReceptionistTM.TaskStates.END:
            Logger.state(self, "Ending task")
            self._track_state_change(ReceptionistTM.TaskStates.END)

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

        if self.current_state == ReceptionistTM.TaskStates.DEBUG:
            Logger.state(self, "Debugging task")
            self._track_state_change(ReceptionistTM.TaskStates.DEBUG)
            self.subtask_manager.hri.say("Debugging task.")
            self.current_state = ReceptionistTM.TaskStates.END


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

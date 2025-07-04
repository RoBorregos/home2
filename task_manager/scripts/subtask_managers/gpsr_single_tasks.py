import time

import rclpy
from frida_constants.hri_constants import GPSR_COMMANDS
from utils.baml_client.types import (
    AnswerQuestion,
    GetVisualInfo,
    GoTo,
    PickObject,
    PlaceObject,
    SayWithContext,
)
from utils.status import Status

from subtask_managers.generic_tasks import GenericTask

RETRIES = 3


def search_command(command, objects: list[object]):
    for object in objects:
        if hasattr(object, command):
            method = getattr(object, command)
            if callable(method):
                return method
    return None


class GPSRSingleTask(GenericTask):
    """Class to manage the GPSR task"""

    def __init__(self, subtask_manager):
        """Initialize the class"""
        super().__init__(subtask_manager)

    ## Nav
    def go_to(self, command: GoTo):
        """
        Navigate to a given location.

        Args:
            complement (str): The location to go to.
            characteristic (str, optional): Always empty.

        Purpose:
            - Go to a location.

        Preconditions:
            - The robot knows the location

        Behavior:
            - Move towards the specified location, while avoiding obstacles.

        Postconditions:
            - The robot is in the specified location
        """

        if isinstance(command, dict):
            command = GoTo(**command)

        self.subtask_manager.hri.say(f"I will go to {command.location_to_go}.", wait=False)
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        location = self.subtask_manager.hri.query_location(command.location_to_go)
        area = self.subtask_manager.hri.get_area(location)
        if isinstance(area, list):
            area = area[0]

        subarea = self.subtask_manager.hri.get_subarea(location)
        if isinstance(subarea, list):
            if len(subarea) == 0:
                subarea = ""
            else:
                subarea = subarea[0]

        self.subtask_manager.hri.node.get_logger().info(f"Moving to {subarea} in {area}")
        self.subtask_manager.hri.node.get_logger().info("Subarea")
        self.subtask_manager.hri.node.get_logger().info(subarea)

        future = self.subtask_manager.nav.move_to_location(area, subarea)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

        return Status.EXECUTION_SUCCESS, "arrived to:" + command.location_to_go

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

    ## Manipulation
    def pick_object(self, command: PickObject):
        """
        Picks an object from a designated picking spot.

        Args:
            complement (str): Description of the object to pick.
            characteristic (str, optional): Always empty.

        Purpose:
            To enable the robot to pick up an object.

        Preconditions:
            - The robot must be at the picking spot.
            - The object must be present on the picking spot.

        Behavior:
            - Executes the necessary manipulation to pick the object from the picking spot.

        Postconditions:
            - The robot holds the object.

        Pseudocode:
            - pick_object(complement)
        """
        if isinstance(command, dict):
            command = PickObject(**command)

        # self.subtask_manager.hri.say(f"I will try to pick {command.object_to_pick}.")

        # def check_found_object():
        #     check_retries = 0

        #     while check_retries < RETRIES:
        #         self.timeout(1.5)
        #         s, detections = self.subtask_manager.vision.detect_objects()
        #         if s == Status.EXECUTION_SUCCESS:
        #             s, object_to_pick = self.subtask_manager.hri.find_closest_raw(
        #                 self.subtask_manager.vision.get_labels(detections), command.object_to_pick
        #             )

        #             print("object_to_pick", object_to_pick)

        #             if object_to_pick[0]["distance"][0] < 0.5:
        #                 return Status.EXECUTION_SUCCESS
        #         check_retries += 1
        #     return Status.TARGET_NOT_FOUND

        # self.subtask_manager.hri.node.get_logger().info("computing loc")

        # loc_retires = 0
        # while loc_retires < RETRIES:
        #     status, results = self.subtask_manager.nav.ReturnLocation_callback()

        #     if status != Status.EXECUTION_SUCCESS:
        #         self.timeout(1.0)
        #     loc_retires += 1

        # self.subtask_manager.hri.node.get_logger().info("loc computed")

        # if status != Status.EXECUTION_SUCCESS:
        #     self.subtask_manager.hri.say(
        #         "I couldn't distinguish picking areas. Please help me pick the object."
        #     )
        #     return self.deus_pick(command)

        # area = results.location
        # locations = results.nearest_locations

        # self.subtask_manager.hri.node.get_logger().info("area: " + str(area))
        # self.subtask_manager.hri.node.get_logger().info("locations: " + str(locations))

        # found_object = False
        # for location in locations:
        #     self.subtask_manager.hri.node.get_logger().info("iterating: " + str(location))
        #     if found_object:
        #         break
        #     if location == "safe_place":
        #         continue

        #     self.navigate_to(area, location, False)
        #     self.subtask_manager.hri.node.get_logger().info("navigate to done")
        #     status = check_found_object()
        #     if status == Status.EXECUTION_SUCCESS:
        #         found_object = True
        #         self.subtask_manager.hri.say(
        #             f"I found the {command.object_to_pick} in the {location}. I will pick it up."
        #         )
        #     else:
        #         self.subtask_manager.hri.say(
        #             f"I couldn't find the {command.object_to_pick} in the {location}."
        #         )

        # if not found_object:
        #     self.subtask_manager.hri.say(
        #         f"I couldn't find the {command.object_to_pick} in any picking area. Please help me pick it up."
        #     )
        #     return self.deus_pick(command)

        self.subtask_manager.hri.say(f"I will pick the {command.object_to_pick}.", wait=False)
        current_try = 0

        while True:
            s, detections = self.subtask_manager.vision.detect_objects()
            current_try += 1

            if current_try >= RETRIES:
                self.subtask_manager.hri.say(
                    f"I could not see the object. Please hand me the {command.object_to_pick}."
                )
                return self.deus_pick(command)

            if len(detections) == 0:
                self.subtask_manager.hri.say("I didn't find any object. I will try again.")
                continue
            if s == Status.EXECUTION_SUCCESS:
                break

        labels = self.subtask_manager.vision.get_labels(detections)
        s, object_to_pick = self.subtask_manager.hri.find_closest(labels, command.object_to_pick)
        if isinstance(object_to_pick, list):
            object_to_pick = object_to_pick[0]
        current_try = 0
        while True:
            if current_try >= RETRIES:
                return self.deus_pick(command)
            s = self.subtask_manager.manipulation.pick_object(object_to_pick), ""
            if s == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    f"I have picked the {command.object_to_pick}.", wait=True
                )
                return s, f"picked {command.object_to_pick}"
            current_try += 1

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def deus_pick(self, command: PickObject):
        deus_machina_retries = 0
        self.subtask_manager.hri.say(
            f"I couldn't make a pick on my own. I will require some help picking the {command.object_to_pick}!"
        )
        while True:
            if deus_machina_retries >= RETRIES:
                self.subtask_manager.hri.say(
                    "I couldn't hear your confirmation, I will abort picking the object."
                )
                return Status.TARGET_NOT_FOUND, ""
            s, res = self.subtask_manager.hri.confirm(
                f"Have you placed the {command.object_to_pick} on my gripper?", use_hotwords=False
            )
            if res == "yes":
                self.subtask_manager.hri.say("Thank you. I will close my gripper")
                return self.subtask_manager.manipulation.close_gripper(), ""
            else:
                deus_machina_retries += 1

    ## Manipulation
    def place_object(self, command: PlaceObject):
        """
        Places an object in the available location.

        Args:
            complement (str, optional): Always empty.
            characteristic (str, optional): Always empty.

        Purpose:
            - To place an object

        Preconditions:
            - The robot must have an object.
            - The robot must be in front of the placing location.

        Behaviour:
            - Attempts to place the object in the specified location if possible.

        Postconditions:
            - The robot is in front of the placing location without the object.

        Pseudocode:
            place()
        """

        place_retries = 0
        self.subtask_manager.hri.say("I will place the object.", wait=False)

        while place_retries < RETRIES:
            s = self.subtask_manager.manipulation.place()
            if s == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have placed the object.")
                return Status.EXECUTION_SUCCESS, "placed"
            place_retries += 1

        self.subtask_manager.hri.say(
            "I couldn't place the object. Please help me place the object. When you confirm you have grabbed the object, I will open my gripper, so that you can place it."
        )

        deus_place_retries = 0
        while True:
            if deus_place_retries >= RETRIES:
                self.subtask_manager.hri.say(
                    "I couldn't hear your confirmation, I will abort placing the object."
                )
                return Status.EXECUTION_ERROR, ""
            s, res = self.subtask_manager.hri.confirm(
                "Have you picked the object in my gripper?", use_hotwords=False
            )
            if res == "yes":
                self.subtask_manager.hri.say("Thank you. I will open my gripper")
                return (
                    self.subtask_manager.manipulation.open_gripper(),
                    "placed with deus ex machina",
                )
            else:
                deus_place_retries += 1

    ## HRI
    def say_with_context(self, command: SayWithContext):
        """
        Say something grounded on the information known to the robot, which can include the results of
        previous executions, robot information, and general knowledge information.

        Args:
            complement (str): The original command.
            characteristic (str): the information to fetch, which can be a previous command or information to answer.

        Purpose:
            - To generate and vocalize a response based on the robot's available information, which may
            include execution results, robot-specific data, or general knowledge.

        Preconditions:
            - The necessary information must be available in the robot's database.

        Behavior:
            - If a command name is specified, fetch the execution results of the last command.
            - Use the provided complement to elaborate a response.
            - Generate the response using a language model and the fetched information.
            - Say the generated response.

        Postconditions:
            None.

        Pseudocode:
            say(llm_response(complement, fetch_info(characteristic)))
        """

        if isinstance(command, dict):
            command = SayWithContext(**command)

        context = command.previous_command_info[0]

        if context == "introduction":
            self.subtask_manager.hri.say("Hello, I am Frida. Nice to meet you.")
            return Status.EXECUTION_SUCCESS, "success"

        if context in GPSR_COMMANDS:
            history = self.subtask_manager.hri.query_command_history(
                command.previous_command_info[0]
            )
            # command_type = self.subtask_manager.hri.get_command(history)
            result = self.subtask_manager.hri.get_result(history)
            status = self.subtask_manager.hri.get_status(history)
            s, answer = self.subtask_manager.hri.answer_with_context(
                command.user_instruction,
                f"Result of executing {context}: {result}. STATUS: {status}",
            )
            self.subtask_manager.hri.say(answer, wait=True)
            return Status.EXECUTION_SUCCESS, "success"
        else:
            s, response, score = self.subtask_manager.hri.answer_question(
                command.previous_command_info[0]
            )

            self.subtask_manager.hri.say(response, wait=True)
            return Status.EXECUTION_SUCCESS, "success"

    ## HRI
    # Removed from the command dataset
    def say(self, complement: str, characteristic=""):
        """
        Makes the robot say the provided text to the user.

        Args:
            complement (str): The text that the robot will say to the user.
            characteristic (str, optional): Always empty.

        Purpose:
            - Say the text to the user

        Preconditions:
            - None

        Behavior:
            - Says the text and blocks execution until the text is finished.

        Postconditions:
            - None

        Pseudocode:
            - say(text)
        """
        return self.subtask_manager.hri.say(complement, wait=True), ""

    ## HRI
    def answer_question(self, command: AnswerQuestion):
        """
        Answers a user's question by asking, confirming, and responding.

        Args:
            complement (str, optional): Always empty.
            characteristic (str, optional): Always empty.

        Purpose:
            To answer the user's question by following a structured process.

        Preconditions:
            The robot must be in front of the user.

        Behavior:
            - Ask a question.
            - Confirm the question.
            - Provide an answer to the question.

        Postconditions:
            None.

        Pseudocode:
            while get_question():
                if confirm_question():
                    contextual_say('Please answer my question', question)
        """

        def confirm_question(interpreted_text, target_info):
            return f"Is your question: {target_info}?"

        status, question = self.subtask_manager.hri.ask_and_confirm(
            "Please ask your question",
            "LLM_question",
            context="The user was asked to say a question. We want to infer his question from the response",
            confirm_question=confirm_question,
            use_hotwords=False,
            retries=3,
            min_wait_between_retries=5.0,
            skip_extract_data=True,
        )

        if status != Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say("I am sorry, I could not understand your question.")
            return Status.TARGET_NOT_FOUND, ""

        question = SayWithContext(
            action="say_with_context",
            user_instruction=f"Please answer my question: {question}",
            previous_command_info=[question],
        )
        return self.say_with_context(question)

    ## Vision
    def get_visual_info(self, command: GetVisualInfo):
        """
        Retrieves visual information about an object based on the specified complement
        and optional characteristic.

        Args:
            complement (str): A descriptor indicating the type of object to analyze.
                Expected values include "biggest object", "largest object",
                "smallest object", "heaviest object", "lightest object",
                or "thinnest object".
            characteristic (str, optional): A filter specifying the object category
                to consider (e.g., "object category"). If empty, there is no filter.

        Purpose:
            To obtain specific data about an object in an image.

        Preconditions:
            The camera must be positioned appropriately to capture the required frame.

        Behavior:
            Analyzes the image to extract the requested data, applying the
            characteristic filter if provided.

        Postconditions:
            The robot saves the specified information for further use.
        """
        if isinstance(command, dict):
            command = GetVisualInfo(**command)

        return self.subtask_manager.vision.visual_info(command.measure, command.object_category)

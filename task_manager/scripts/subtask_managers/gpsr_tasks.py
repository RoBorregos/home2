import importlib.resources as pkg_resources
import json

import frida_constants
import rclpy
from frida_constants.vision_enums import DetectBy, Gestures, Poses, is_value_in_enum

from subtask_managers.generic_tasks import GenericTask
from task_manager.scripts.utils.status import Status


class GPSRTask(GenericTask):
    """Class to manage the GPS task"""

    def __init__(self, subtask_manager):
        """Initialize the class"""
        super().__init__(subtask_manager)
        self.pan_angles = [-45, 0, 45]
        with pkg_resources.files(frida_constants).joinpath("areas.json").open("r") as f:
            self.locations = json.load(f)

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self, future)

    ## HRI, Manipulation
    def give(self, complement="", characteristic=""):
        """
        Handles the action of giving an object to a person.

        Args:
            complement (str, optional): Always empty.
            characteristic (str, optional): Always empty.

        Purpose:
            - Give an object to a person.

        Preconditions:
            - The robot must have possession of an object.
            - The robot must be positioned in front of a person.

        Behavior:
            - Moves the robot's arm to the 'giving' position.
            - Waits for confirmation that the person has grabbed the object.
            - Releases the object by opening the gripper.
            - Moves the robot's arm back to the 'standard' position.

        Postconditions:
            - The robot is in front of a person without holding an object.
        """
        self.subtask_manager.hri.say(
            "I will give you the object. Once you have picked the object, I will open my gripper",
            wait=False,
        )
        self.subtask_manager.manipulation.move_to_position(named_position="receive_object")

        while True:
            s, res = self.subtask_manager.hri.confirm("Have you grabbed the object?")
            if res == "yes":
                break
            else:
                self.subtask_manager.hri.say(
                    "I will give you the object. Once you have picked the object, I will open my gripper.",
                    wait=False,
                )

        self.subtask_manager.hri.say("I will now release the object.", wait=True)
        self.subtask_manager.manipulation.open_gripper()

        return Status.EXECUTION_SUCCESS, "object given"

    ## HRI, Nav
    def follow_person_until(self, complement: str, characteristic: str):
        """
        Follow a person until a specific condition is triggered.

        Args:
            complement (str): The condition to stop following. If it is a location,
                the robot will follow the person until it arrives at the location.
                If it is 'canceled', the robot will follow the person until the
                person cancels the follow.
            characteristic (str): name of the person to follow or empty if unavilable. Can be used to improve interaction.

        Preconditions:
            - The robot is in front of the person it will follow.

        Behavior:
            - If the complement is a location, the robot follows the person until
              it reaches the specified location.
            - If the complement is 'canceled', the robot follows the person until
              the person cancels the follow.

        Postconditions:
            - The robot is in the room where the user canceled the operation or in
              the target room.
            - Store command execution status in the database.


        Pseudocode:
            while not_in_room() or not_canceled():
                follow_person()

        """

        if characteristic == "":
            self.subtask_manager.hri.say(
                "I will now follow you.",
            )
            # status = self.subtask_manager.vision.track_person(True)
        else:
            self.subtask_manager.hri.say(
                f"I will now follow you, {characteristic}.",
            )
            # status = self.subtask_manager.vision.follow_by_name(characteristic)
            self.subtask_manager.nav.follow_person()

        # TODO (@nav, hri): fix conditions to stop
        if complement == "canceled":
            while self.subtask_manager.hri.hear() != "cancel":
                pass

        else:
            while self.subtask_manager.nav.get_location() != complement:
                pass

        self.subtask_manager.vision.track_person(False)
        self.subtask_manager.vision.follow_by_name("area")
        self.subtask_manager.nav.activate_follow(False)

        pass

    ## HRI, Nav
    def guide_to(self, complement="", characteristic=""):
        """
        Guides a person to a specified target location.

        Args:
            complement (str): "person" or a name. If a name is provided, it can be used to improve interaction.
            characteristic (str, optional): location or room to guide the person to.

        Purpose:
            - Guide a person to a target location.

        Preconditions:
            - The robot must be positioned in front of the person.
            - If a name is specified, the robot already knows the name and face of the person.

        Behavior:
            - The robot expresses its intent to guide the person.
            - The robot moves to the specified location.
            - Optionally, the robot confirms that the person is following it.

        Postconditions:
            - Both the robot and the guided person are at the target location.
            - Store command execution status in the database.

        Pseudocode:
            - say(intent)
            - go(characteristic)

        """

        self.navigate_to(characteristic)

        if complement == "person":
            self.subtask_manager.hri.say(f"We have arrived to {characteristic}!", wait=True)
        else:
            self.subtask_manager.hri.say(f"We have arrived, {complement}!", wait=True)

    ## HRI, Vision
    def find_person_info(self, complement="", characteristic=""):
        """
        Get specific information about a person.

        Args:
            complement (str): The type of information to retrieve. If "gesture" or "posture", the robot computes the information visually. If "name", the robot fetches the name from known names or interacts with the person if the name is unknown.
            characteristic (str): Always empty string.

        Preconditions:
            - The robot must be in front of the person.

        Behaviour:
            - If the complement is "gesture" or "posture", the robot computes the information visually.
            - If the complement is "name", the robot fetches the name from previously known names or interacts with the person if the name is not known.

        Postconditions:
            - The robot saves the specified information about the person in the database.

        Pseudocode:
            if complement == 'gesture' or complement == 'posture':
                return gest_posture_analyzer()
            else:
                return get_person_name()
        """

        if complement == "gesture":
            complement = DetectBy.GESTURES.value
        elif complement == "posture":
            complement = DetectBy.POSES.value

        if complement != "name":
            return self.subtask_manager.vision.find_person_info(complement)

        s, res = self.subtask_manager.vision.get_person_name()
        if s == Status.EXECUTION_SUCCESS:
            return (Status.EXECUTION_SUCCESS,)

        else:
            self.subtask_manager.hri.say(
                "Hi, I'm Frida.",
            )
            s, name = self.subtask_manager.hri.ask_and_confirm(
                question="Can you please tell me your name?",
                query="name",
                use_hotwords=False,
                context="The user was asked to say their name. We want to infer his name from the response",
            )
            if s == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    "Please stand in front .",
                )
                self.subtask_manager.vision.save_face_name(res)
                return Status.EXECUTION_SUCCESS, res
            else:
                return Status.EXECUTION_ERROR, "name not found"

    ## Nav, Vision
    def find_object(self, complement: str, characteristic: str):
        """
        Finds an object in a specified location and approaches it for picking.

        Args:
            complement (str): Specifies additional context for the search.
                If it is a room, the robot will search all placements in the room.
            characteristic (str): Specifies the object to find.

        Purpose:
            - Locate an object in a given place and approach it to a position suitable for picking.

        Preconditions:
            - Assumes the robot is already at a location.

        Behavior:
            - If the location is a placement location, the robot will search for the object only in that placement.
            - If the complement specifies a room, the robot will navigate to all placements in the room and search for the object.
            - Once the object is found, the robot will approach it to a position where it can pick the object.

        Postconditions:
            - The robot will be positioned at a location where it can pick the object.

        Pseudocode:
            For each location in specified_room:
                Navigate to the location.
                If the object is detected in the location:
                    Approach the object to a position suitable for picking.
        """

        for location in self.locations[complement]:
            self.navigate_to(complement, location, False)

            result_status = self.subtask_manager.vision.find_object(characteristic)
            if result_status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I found the {characteristic}.")
                return Status.EXECUTION_SUCCESS, "object found"

        return Status.TARGET_NOT_FOUND, "object not found"

    ## Manipulation, Vision
    def count(self, complement="", characteristic=""):
        """
        Count the number of specific targets based on the given parameters.

        Args:
            complement (str): Specifies the context for counting.
                Options include placement location or a room.
            characteristic (str): Specifies the type of target to count.
                Options include objects, "person with " + cloth + color, person with some posture,
                or person with some gesture.

        Purpose:
            - Compute the amount of a specific target to later communicate it to a person.

        Preconditions:
            - Assumes the robot is at a location.

        Behavior:
            - If a placement location is given, count the number of the specified item.
            - If a room is given, gaze at several non-overlapping points and return the
              sum of the target across the several pictures.

        Postconditions:
            - The count result is saved in the database.

        Pseudocode:
            For each degree in [-45, 0, 45]:
                Gaze at the specified degree.
                Add the count of the target in the frame to the total count.
            Store the total count.
        """

        # TODO (@nav): go to a location given only one value

        # TODO (@hri):
        # TODO: get category and value that matches vision_enums
        # Ex: "poses", "standing" , "clothes", "red t-shirt"

        possibilities = (
            [v.value for v in DetectBy.GESTURES] + [v.value for v in DetectBy.POSES] + ["clothes"]
        )

        status, value = self.subtask_manager.hri.find_closest(possibilities, characteristic)

        self.subtask_manager.manipulation.move_to_position("front_stare")

        counter = 0
        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)
            if is_value_in_enum(value, Gestures):
                status, count = self.subtask_manager.vision.count_by_gesture(value)
            elif is_value_in_enum(value, Poses):
                status, count = self.subtask_manager.vision.count_by_pose(value)
            else:
                color = self.subtask_manager.hri.extract_data("color", value)
                cloth = self.subtask_manager.hri.extract_data("cloth", value)
                status, count = self.subtask_manager.vision.count_by_color(color, cloth)

            if status == Status.EXECUTION_SUCCESS:
                counter += count
                self.subtask_manager.hri.say(f"I have counted {count} {characteristic}.")
            elif status == Status.TARGET_NOT_FOUND:
                self.subtask_manager.hri.say(
                    f"I did't find any {characteristic}.",
                )

        self.subtask_manager.hri.say(
            f"I have counted {counter} {characteristic} in the room.",
        )
        return Status.EXECUTION_SUCCESS, "counted " + str(counter) + " " + characteristic

    ## Manipulation, Nav, Vision
    def find_person(self, complement="", characteristic=""):
        """
        Finds and approaches a person for further interaction.

        Args:
            complement (str): Additional information to help identify the person, it can be a gesture, pose, a color + cloth, or empty.
                              If not specified, the robot will attempt to find any person.
            characteristic (str): A string giving details of the characteristic, can be "clothes", "gesture", "posture" or "".

        Purpose:
            - Find a person and approach it for further interaction. Further interaction may be: following the person,
              guiding the person, interacting with the person, describing the person, or give an object to a person.

        Preconditions:
            - Assumes the robot is already at a specific location.

        Behavior:
            - The robot scans its surroundings by gazing at different angles (e.g., -45, 0, 45 degrees).
            - If a person matching the specified complement is found, the robot approaches them.
            - If no such person is found, a fallback mechanism (`deus_machina`) is invoked.

        Postconditions:
            The robot successfully approaches the identified person, ready for further interaction.

        Pseudocode:
            found = False
            for degree in [-45, 0, 45]:
                gaze(degree)
                if person(complement, characteristic):
                    approach_person(complement, characteristic)
                    found = True
            if not found:
                deus_machina()
        """

        for location in self.locations[complement]:
            self.navigate_to(complement, location, False)

            self.subtask_manager.manipulation.move_to_position("front_stare")
            for degree in self.pan_angles:
                self.subtask_manager.manipulation.pan_to(degree)
                if complement == "":
                    result = self.subtask_manager.vision.track_person(track=True)
                else:
                    result = self.subtask_manager.vision.track_person_by(
                        by=complement, value=characteristic, track=True
                    )
                if result == Status.EXECUTION_SUCCESS:
                    return Status.EXECUTION_SUCCESS, "person found"

        return Status.TARGET_NOT_FOUND, "person not found"

    ## HRI, Manipulation, Nav, Vision
    def find_person_by_name(self, complement="", characteristic=""):
        """
        Find a person by their name and save the information for further interactions. Further interaction may include: following the person,
        guiding the person, interacting with the person, describing the person, give an object to a person.

        Args:
            complement (str): The name of the person to search for.
            characteristic (str, optional): Always an empty string.

        Behavior:
            - The robot searches for a person by scanning the area at different angles.
            - For each detected person, it checks if the person is already known.
            - If the person is unknown, the robot asks for their name and saves it.
            - The robot approaches the person with the specified name once found.
            - If the maximum number of retries is exceeded, a fallback mechanism (`deus_machina`) is triggered.

        Preconditions:
            - The robot must be at a location where it can search for people.

        Postconditions:
            - The robot approaches the person with the specified name.
            - The robot saves information about all the people it encounters.
        """

        for location in self.locations[complement]:
            self.navigate_to(complement, location, False)

            self.subtask_manager.manipulation.move_to_position("front_stare")
            for degree in self.pan_angles:
                self.subtask_manager.manipulation.pan_to(degree)
                status, name = self.subtask_manager.vision.get_person_name()

                if status == Status.TARGET_NOT_FOUND:
                    continue

                if name == "Unknown":
                    self.subtask_manager.hri.say(
                        "Hi, I'm Frida. Can you please tell me your name?",
                    )
                    status, new_name = self.subtask_manager.hri.ask_and_confirm(
                        question="Can you please tell me your name?",
                        query="name",
                        use_hotwords=False,
                    )
                    self.subtask_manager.vision.save_face_name(new_name)
                    name = new_name

                if name == complement:
                    return Status.EXECUTION_SUCCESS, f"found {name}"
        return Status.TARGET_NOT_FOUND, "person not found"

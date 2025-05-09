import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from frida_constants.vision_enums import DetectBy, Gestures, Poses, is_value_in_enum
from utils.baml_client.types import (
    Count,
    FindPerson,
    FindPersonByName,
    FollowPersonUntil,
    GetPersonInfo,
    GiveObject,
    GuidePersonTo,
)
from utils.status import Status

from subtask_managers.generic_tasks import GenericTask


class GPSRTask(GenericTask):
    """Class to manage the GPSR task"""

    def __init__(self, subtask_manager):
        """Initialize the class"""
        super().__init__(subtask_manager)
        # Angles are relative to current position
        self.pan_angles = [-45, 45, 45]
        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")
        with open(file_path, "r") as file:
            self.locations = json.load(file)

        self.color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray", "green"]
        self.clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket", "jeans"]

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
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

    ## HRI, Manipulation
    def give_object(self, command: GiveObject):
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
            s, res = self.subtask_manager.hri.confirm(
                "Have you grabbed the object?", use_hotwords=False
            )
            if res == "yes":
                break
            else:
                self.subtask_manager.hri.say(
                    "I will give you the object. Once you have picked the object, I will open my gripper.",
                    wait=True,
                )

        self.subtask_manager.hri.say("I will now release the object.", wait=True)
        self.subtask_manager.manipulation.open_gripper()

        return Status.EXECUTION_SUCCESS, "object given"

    ## HRI, Nav
    def follow_person_until(self, command: FollowPersonUntil):
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
        if isinstance(command, dict):
            command = FollowPersonUntil(**command)

        # TODO: fix this, now follow person until only has destination because
        # it can only be triggered after a find_person action, my suggestion for
        # all is to read the hri subtask manager log to find
        # if characteristic == "":
        #   self.subtask_manager.hri.say(
        #       "I will now follow you.",
        #   )
        # status = self.subtask_manager.vision.track_person(True)
        # else:
        #   self.subtask_manager.hri.say(
        #       f"I will now follow you, {characteristic}.",
        #   )
        # status = self.subtask_manager.vision.follow_by_name(characteristic)
        #   self.subtask_manager.nav.follow_person()

        # TODO (@nav, hri): fix conditions to stop

        loc = command.destination

        if command.destination == "cancelled":
            self.subtask_manager.hri.say(
                "I'm sorry, I can't follow you. Please tell me where to go"
            )
            status, loc = self.subtask_manager.hri.ask_and_confirm(
                question="Please tell me where to go.",
                query="location",
                use_hotwords=False,
                context="The user was asked to say the location. We want to infer the location from the response",
            )

        else:
            self.subtask_manager.hri.say(
                f"I'm sorry, I can't follow you, but I'll go to the {command.destination}",
            )

        # infer location from the response
        # go to

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        location = self.subtask_manager.hri.query_location(loc)
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

        # xd
        # if command.destination == "cancelled":
        #     while self.subtask_manager.hri.hear() != "cancel":
        #         pass

        # else:
        #     while self.subtask_manager.nav.get_location() != command.destination:
        #         pass

        # self.subtask_manager.vision.track_person(False)
        # self.subtask_manager.vision.follow_by_name("area")
        # self.subtask_manager.nav.activate_follow(False)

        # pass

    ## HRI, Nav
    def guide_person_to(self, command: GuidePersonTo):
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
        if isinstance(command, dict):
            command = GuidePersonTo(**command)

        location = self.subtask_manager.hri.query_location(command.destination_room)
        area = self.subtask_manager.hri.get_area(location)
        self.subtask_manager.hri.node.get_logger().info(f"Initial area: {area}.")

        if isinstance(area, list):
            area = area[0]

        subarea = self.subtask_manager.hri.get_subarea(location)

        if isinstance(subarea, list):
            if len(subarea) == 0:
                subarea = ""
            else:
                subarea = subarea[0]

        self.navigate_to(area, subarea)

        self.subtask_manager.hri.say(f"We have arrived to {command.destination_room}!", wait=True)

    ## HRI, Vision
    def get_person_info(self, command: GetPersonInfo):
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

        if isinstance(command, dict):
            command = GetPersonInfo(**command)

        print("RECEIVED COMMAND", command.info_type)
        if command.info_type == "gesture":
            command.info_type = DetectBy.GESTURES.value
        elif command.info_type == "pose":
            command.info_type = DetectBy.POSES.value

        self.subtask_manager.manipulation.move_to_position("front_stare")

        self.subtask_manager.hri.say(f"I will search for the {command.info_type} of a person.")

        if command.info_type != "name":
            person_retries = 0
            while person_retries < 2:
                self.timeout(1)
                print("AAAA", command.info_type)
                s, res = self.subtask_manager.vision.find_person_info(command.info_type)

                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        f"The person is {res}.",
                    )
                    return s, res
                person_retries += 1

            # Look for the person again but at a lower degree
            self.subtask_manager.manipulation.check_lower(30)
            person_retries = 0

            while person_retries < 2:
                self.timeout(1)
                s, res = self.subtask_manager.vision.find_person_info(command.info_type)

                if s == Status.EXECUTION_SUCCESS:
                    self.subtask_manager.hri.say(
                        f"The person is {res}.",
                    )
                    return s, res
                person_retries += 1

            self.subtask_manager.hri.say(
                f"I couldn't find the person's {command.info_type}.",
            )
            return Status.TARGET_NOT_FOUND, "Unkown"

        else:
            # get name
            self.subtask_manager.hri.say(
                "I will check if I know your name.",
            )
            current_attempt = 0
            while current_attempt < 3:
                current_attempt += 1
                s, res = self.subtask_manager.vision.get_person_name()
                print(f"Person name: {res}")
                if s == Status.EXECUTION_SUCCESS and res != "Unknown":
                    self.subtask_manager.hri.say(f"Hi {res}, nice to meet you again!")
                    return Status.EXECUTION_SUCCESS, res
            else:
                self.subtask_manager.hri.say(
                    "Hi, I'm Frida. I don't think I know you yet.",
                )
                s, response = self.subtask_manager.hri.ask_and_confirm(
                    question="Can you please tell me your name?",
                    query="name",
                    use_hotwords=False,
                    context="The user was asked to say their name. We want to infer his name from the response",
                )
                if s == Status.EXECUTION_SUCCESS:
                    save_name_retries = 0
                    while save_name_retries < 3:
                        self.subtask_manager.hri.say(
                            "Please stand in front of me so I can save your face.",
                        )

                        if (
                            self.subtask_manager.vision.save_face_name(response)
                            == Status.EXECUTION_SUCCESS
                        ):
                            self.subtask_manager.hri.say(
                                f"Nice to meet you, {response}. I have saved your name.",
                            )
                            return Status.EXECUTION_SUCCESS, response

                        save_name_retries += 1

                    self.subtask_manager.hri.say(
                        "Sorry, I couldn't save your name.",
                    )
                    return Status.TARGET_NOT_FOUND, response
                else:
                    self.subtask_manager.hri.say(
                        "I couldn't undestand your name",
                    )
                    return Status.EXECUTION_ERROR, "name not found"

    ## Nav, Vision
    # TODO: We removed this in command dataset
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

        # for location in self.locations[complement]:
        #     self.navigate_to(complement, location, False)

        result_status = self.subtask_manager.vision.find_object(characteristic)
        if result_status == Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say(f"I found the {characteristic}.")
            return Status.EXECUTION_SUCCESS, "object found"

        return Status.TARGET_NOT_FOUND, "object not found"

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def count_objects(self, object_name: str):
        self.subtask_manager.manipulation.move_to_position("table_stare")
        self.subtask_manager.hri.say(
            f"I am going to count {object_name}.",
        )
        status, result = self.subtask_manager.vision.count_objects(object_name)
        if status == Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say(
                f"I have counted {result} {object_name}.",
            )
        elif status == Status.TARGET_NOT_FOUND:
            self.subtask_manager.hri.say(
                f"I didn't find any {object_name}.",
            )

    ## Manipulation, Vision
    def count(self, command: Count):
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

        if isinstance(command, dict):
            command = Count(**command)

        possibilities = [v.value for v in Gestures] + [v.value for v in Poses] + ["clothes"]

        status, value = self.subtask_manager.hri.find_closest(
            possibilities, command.target_to_count
        )
        value = value[0]

        print(f"Value: {value}", command.target_to_count)

        if (
            "person" not in command.target_to_count.lower()
            and "people" not in command.target_to_count.lower()
        ):
            self.count_objects(command.target_to_count)
            return Status.EXECUTION_SUCCESS, "counted objects"

        self.subtask_manager.manipulation.move_to_position("front_stare")

        counter = 0

        self.subtask_manager.hri.say(
            f"I am going to count the {value}.",
        )

        cache_color = None
        cache_cloth = None

        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)

            if is_value_in_enum(value, Gestures):
                status, count = self.subtask_manager.vision.count_by_gesture(value)
            elif is_value_in_enum(value, Poses):
                status, count = self.subtask_manager.vision.count_by_pose(value)
            else:
                if cache_color is None or cache_cloth is None:
                    s, color = self.subtask_manager.hri.find_closest(
                        self.color_list, command.target_to_count
                    )
                    cache_color = color[0]
                    s, cloth = self.subtask_manager.hri.find_closest(
                        self.clothe_list, command.target_to_count
                    )
                    cache_cloth = cloth[0]

                color = cache_color
                cloth = cache_cloth
                # Say actual color that its counting
                characteristic = f"{color} {cloth}s"
                self.subtask_manager.hri.say(
                    f"I am going to count the {characteristic}.",
                    wait=False,
                )
                status, count = self.subtask_manager.vision.count_by_color(color, cloth)

            if status == Status.EXECUTION_SUCCESS:
                counter += count
                self.subtask_manager.hri.say(f"I have counted {count} {command.target_to_count}.")

            elif status == Status.TARGET_NOT_FOUND:
                self.subtask_manager.hri.say(
                    f"I didn't find any {command.target_to_count}.",
                )

        self.subtask_manager.hri.say(
            f"I have counted {counter} {command.target_to_count} in the room.",
        )
        return Status.EXECUTION_SUCCESS, "counted " + str(counter) + " " + command.target_to_count

    ## Manipulation, Nav, Vision
    def find_person_og(self, command: FindPerson):
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

        if isinstance(command, dict):
            command = FindPerson(**command)

        self.subtask_manager.manipulation.move_to_position("front_stare")
        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)
            if command.attribute_value == "":
                result = self.subtask_manager.vision.track_person(track=True)
            else:
                result = self.subtask_manager.vision.track_person_by(
                    by=command.attribute_value, value="", track=True
                )
                # TODO: We removed the second param from the command
            # TODO (@nav): approach the person
            if result == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(
                    f"I found a person with {command.attribute_value}. Please get closer to me.",
                )
                return Status.EXECUTION_SUCCESS, "person found"

        self.subtask_manager.hri.say(
            f"I didn't find a person with {command.attribute_value}.",
        )
        return Status.TARGET_NOT_FOUND, "person not found"

    def find_person(self, command: FindPersonByName):
        if isinstance(command, dict):
            command = Count(**command)

        possibilities = [v.value for v in Gestures] + [v.value for v in Poses] + ["clothes"]

        status, value = self.subtask_manager.hri.find_closest(
            possibilities, command.target_to_count
        )
        value = value[0]

        self.subtask_manager.manipulation.move_to_position("front_stare")

        self.subtask_manager.hri.say(
            f"Searching for {value}.",
        )

        cache_color = None
        cache_cloth = None

        for degree in self.pan_angles:
            self.subtask_manager.manipulation.pan_to(degree)

            if command.target_to_count == "":
                status, count = self.subtask_manager.vision.count_by_pose(Poses.STANDING.value)
            elif is_value_in_enum(value, Gestures):
                status, count = self.subtask_manager.vision.count_by_gesture(value)
            elif is_value_in_enum(value, Poses):
                status, count = self.subtask_manager.vision.count_by_pose(value)
            else:
                if cache_color is None or cache_cloth is None:
                    s, color = self.subtask_manager.hri.find_closest(
                        self.color_list, command.target_to_count
                    )
                    cache_color = color[0]
                    s, cloth = self.subtask_manager.hri.find_closest(
                        self.clothe_list, command.target_to_count
                    )
                    cache_cloth = cloth[0]

                color = cache_color
                cloth = cache_cloth
                # Say actual color that its counting
                characteristic = f"{color} {cloth}s"
                self.subtask_manager.hri.say(
                    f"I am going to count the {characteristic}.",
                    wait=False,
                )
                status, count = self.subtask_manager.vision.count_by_color(color, cloth)

            if status == Status.EXECUTION_SUCCESS and count > 0:
                self.subtask_manager.hri.say(
                    f"I found a person with {command.target_to_count}. Please get approach me.",
                )
                break

            elif status == Status.TARGET_NOT_FOUND:
                self.subtask_manager.hri.say(
                    f"I didn't find any person with {command.target_to_count}.",
                )

        return Status.EXECUTION_SUCCESS, "found" + command.target_to_count

    ## HRI, Manipulation, Nav, Vision
    def find_person_by_name(self, command: FindPersonByName):
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
        if isinstance(command, dict):
            command = FindPersonByName(**command)

        # self.subtask_manager.manipulation.move_to_position("front_stare")
        for degree in self.pan_angles:
            # self.subtask_manager.manipulation.pan_to(degree)
            while True:
                self.subtask_manager.hri.say(
                    "Please stand in front of me.",
                )

                status, name = self.subtask_manager.vision.get_person_name()
                if status == Status.EXECUTION_SUCCESS:
                    break

            if status == Status.TARGET_NOT_FOUND:
                continue

            # TODO: (@nav): approach the person
            self.subtask_manager.hri.node.get_logger().info(f"Found {name}.")
            if name == "Unknown":
                self.subtask_manager.hri.say("Hi, I'm Frida.")
                status, new_name = self.subtask_manager.hri.ask_and_confirm(
                    question="Can you please tell me your name?",
                    query="name",
                    use_hotwords=False,
                )
                self.subtask_manager.vision.save_face_name(new_name)
                name = new_name

            if name == command.name:
                self.subtask_manager.hri.say("Nice to meet you, " + name + ".")
                return Status.EXECUTION_SUCCESS, f"found {name}"
            else:
                self.subtask_manager.hri.say(
                    "Hi, " + name + ", but I am looking for " + command.name + "."
                )
                self.subtask_manager.vision.save_face_name(name)
        return Status.TARGET_NOT_FOUND, "person not found"

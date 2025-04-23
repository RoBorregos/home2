from subtask_managers.generic_tasks import GenericTask
from task_manager.scripts.utils.status import Status


class GPSRTask(GenericTask):
    """Class to manage the GPS task"""

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
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="receive_object", velocity=0.5, degrees=True
        )

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

        pass

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
        pass

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
        pass

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

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="front_stare", velocity=0.5, degrees=True
        )
        for degree in [-45, 0, 45]:
            # self.subtask_manager.manipulation.set_angle(degree)
            pass

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
        self.subtask_manager.manipulation.move_joint_positions(
            named_position="front_stare", velocity=0.5, degrees=True
        )
        for degree in [-45, 0, 45]:
            # self.subtask_manager.manipulation.set_angle(degree)
            pass

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
        pass

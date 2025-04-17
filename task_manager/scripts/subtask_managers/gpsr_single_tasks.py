from subtask_managers.generic_tasks import GenericTask


class GPSRSingleTask(GenericTask):
    """Class to manage the GPS task"""

    ## Nav
    def go(self, complement="", characteristic=""):
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
        self.subtask_manager.hri.say(f"I will go to {complement}.", wait=False)
        self.subtask_manager.nav.move_to_location(complement)

    ## Manipulation
    def pick(self, complement: str, characteristic=""):
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
        self.subtask_manager.hri.say(f"I will pick the {complement}.", wait=False)

        pass

    ## Manipulation
    def place(self, complement="", characteristic=""):
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
        self.subtask_manager.hri.say("I will place the object.", wait=False)
        pass

    ## HRI
    def contextual_say(self, complement: str, characteristic: str):
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
        pass

    ## HRI
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
        self.subtask_manager.hri.say(complement, wait=True)

    ## HRI
    def ask_answer_question(self, complement="", characteristic=""):
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

        pass

    ## Vision
    def visual_info(self, complement: str, characteristic=""):
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

        # TODO: @lleo-13 -> implement saving information to db
        # TODO: @Oscar-gg -> integrate db interactions
        pass

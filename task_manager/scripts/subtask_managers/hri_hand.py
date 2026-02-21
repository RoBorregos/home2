#!/usr/bin/env python3

from utils.status import Status

# from subtask_managers.hri_tasks import HRITasks


class HRIHand:
    """Class to implement hri routines needed for give a hand"""

    # def __init__(self, hri_manager: HRITasks):
    def __init__(self, hri_manager):
        """Initialize the class"""

        self.hri_manager = hri_manager

    def furniture_or_object(self):
        """Determine if the object where the user wants to place something is a furniture or an object"""
        status, response = self.hri_manager.ask_and_confirm(
            "Are you referring to a piece of furniture or an object as the placement location?",
            "furniture or object",
            context="The user should specify whether they want to place the item near furniture (like a table, chair, sofa) or near an object (like a cup, book, phone).",
            use_hotwords=False,
            retries=3,
            min_wait_between_retries=3,
            skip_extract_data=False,
            skip_confirmation=True,
        )

        if status == Status.EXECUTION_SUCCESS:
            # Use find_closest to determine if it's furniture or object
            closest_status, closest_match, _ = self.hri_manager.find_closest(
                ["furniture", "object"], response
            )
            return closest_status, closest_match[0] if closest_match else "object"

        return status, "furniture"  # Default to furniture if unclear

    def determine_furniture(self, room: str = ""):
        """Determine which piece of furniture the user is referring to"""
        question = f"Which piece of furniture in the {room} would you like me to place the object near? Please describe it."

        status, response = self.hri_manager.ask_and_confirm(
            question,
            "furniture type",
            context="The user should specify a piece of furniture like table, chair, sofa, bed, desk, shelf, etc.",
            use_hotwords=False,
            retries=3,
            min_wait_between_retries=3,
            skip_extract_data=False,
            skip_confirmation=True,
        )

        if status == Status.EXECUTION_SUCCESS:
            # Query the location database to find the closest furniture match
            # Include room context in the query if provided
            search_query = f"{response} in {room}" if room else response
            location_results = self.hri_manager.query_location(search_query, top_k=5)

            # Filter results by room if room is specified
            if room and location_results:
                filtered_results = []
                for result in location_results:
                    # Check if the location contains the room name or is associated with it
                    location_name = result.area.lower()
                    location_context = result.context
                    room_lower = room.lower()

                    if room_lower in location_name or room_lower in location_context:
                        filtered_results.append(result)

                # If we have filtered results, use them; otherwise fall back to all results
                location_results = filtered_results if filtered_results else location_results[:3]
            else:
                location_results = location_results[:3] if location_results else []

            if location_results:
                # Confirm with the user
                confirm_question = f"Should I place it near the {location_results[0].subarea}?"

                confirm_status, confirmation = self.hri_manager.confirm(
                    confirm_question,
                    use_hotwords=True,
                    retries=2,
                )

                if confirmation == "yes":
                    return status, location_results[0].subarea
                elif len(location_results) > 1:
                    # Try second best match
                    second_furniture = location_results[1].subarea
                    confirm_question = f"How about near the {second_furniture}?"

                    confirm_status, confirmation = self.hri_manager.confirm(
                        confirm_question,
                        use_hotwords=True,
                        retries=2,
                    )
                    if confirmation == "yes":
                        return status, second_furniture

            # If no good match found, use the raw response
            return status, response

        return status, ""

    def determine_object(self):
        """Determine which object the user is referring to for placement"""
        question = "Which object would you like me to place the item near? Please describe it."

        status, response = self.hri_manager.ask_and_confirm(
            question,
            "object name",
            context="The user should specify an object like a cup, book, phone, pen, etc.",
            use_hotwords=False,
            retries=3,
            min_wait_between_retries=3,
            skip_extract_data=False,
            skip_confirmation=True,
        )

        if status == Status.EXECUTION_SUCCESS:
            # Use the PostgreSQL adapter to find the closest matching hand items
            # Include room context in the search if provided
            closest_by_name, closest_by_description = self.hri_manager.pg.get_hand_items(response)

            if closest_by_description:
                # Confirm with the user using the description
                object_name = closest_by_description[0].name
                confirm_question = f"Should I place it near the {object_name}?"
                confirm_status, confirmation = self.hri_manager.confirm(
                    confirm_question,
                    use_hotwords=True,
                    retries=2,
                )

                if confirmation == "yes":
                    return status, object_name
                elif closest_by_name:
                    # Try the name-based match
                    name_object = closest_by_name[0].name
                    confirm_question = f"How about near the {name_object}?"
                    confirm_status, confirmation = self.hri_manager.confirm(
                        confirm_question,
                        use_hotwords=True,
                        retries=2,
                    )
                    if confirmation == "yes":
                        return status, name_object

            # If no good match found, use the raw response
            return status, response

        return status, ""

    def get_place_orientation(self):
        """Get the orientation/direction where the user wants to place the object"""
        status, response = self.hri_manager.ask_and_confirm(
            "Where do you want me to place it with respect to the location? Please say left, right, front, back, top, or nearby.",
            "LLM_orientation",
            context="The user should specify a spatial orientation like left, right, front, back, top, or nearby.",
            use_hotwords=False,
            retries=3,
            min_wait_between_retries=3,
            skip_extract_data=True,
            skip_confirmation=True,
        )

        if status == Status.EXECUTION_SUCCESS:
            # Find the closest matching orientation
            orientation_options = ["left", "right", "front", "back", "top", "nearby"]
            closest_status, closest_match, _ = self.hri_manager.find_closest(
                orientation_options, response
            )

            if closest_status == Status.EXECUTION_SUCCESS and closest_match:
                orientation = closest_match[0]

                # Confirm with the user
                prefix = "on the " if orientation != "nearby" else ""
                confirm_status, confirmation = self.hri_manager.confirm(
                    f"Should I place it {prefix}{orientation}?", use_hotwords=True, retries=2
                )

                if confirmation == "yes":
                    return status, orientation
                else:
                    # Try the second best match if available
                    if len(closest_match) > 1:
                        second_orientation = closest_match[1]
                        prefix = "on the " if second_orientation != "nearby" else ""
                        confirm_status, confirmation = self.hri_manager.confirm(
                            f"How about {prefix}{second_orientation}?", use_hotwords=True, retries=2
                        )
                        if confirmation == "yes":
                            return status, second_orientation

            # If no good match or confirmation failed, use "nearby" as default
            return status, "nearby"

        return status, "nearby"  # Default to nearby if unclear

    def get_complete_placement_info(self, room: str = ""):
        """
        Get complete placement information including location and orientation

        Args:
            room (str): The room where the placement should occur. Used to filter furniture and objects.

        Returns:
            tuple: (status, location, orientation) where:
                - status: Status enum indicating success or failure
                - location: The determined location (furniture or object name)
                - orientation: The spatial orientation (left, right, front, back, top, nearby)
        """

        self.hri_manager.say("I need to understand where you want me to place the object.")

        # Step 1: Determine if it's furniture or object
        furn_obj_status, target_type = self.furniture_or_object()
        if furn_obj_status != Status.EXECUTION_SUCCESS:
            self.hri_manager.say(
                "I couldn't understand the target type. Defaulting to furniture placement."
            )
            target_type = "furniture"

        # Step 2: Determine the specific location
        if target_type == "furniture":
            location_status, location = self.determine_furniture(room)
        else:
            location_status, location = self.determine_object(room)

        if location_status != Status.EXECUTION_SUCCESS:
            self.hri_manager.say("I couldn't determine the specific location.")
            return location_status, "", ""

        # Step 3: Get the orientation
        orientation_status, orientation = self.get_place_orientation()
        if orientation_status != Status.EXECUTION_SUCCESS:
            self.hri_manager.say("I couldn't determine the orientation. Using 'nearby' as default.")
            orientation = "nearby"

        # Confirm the complete placement
        prefix = "on the " if orientation != "nearby" else ""
        final_confirmation_status, confirmation = self.hri_manager.confirm(
            f"So I should place the object {prefix}{orientation} the {location}. Is that correct?",
            use_hotwords=True,
            retries=2,
        )

        if confirmation == "yes":
            self.hri_manager.say(f"Perfect! I'll place it {prefix}{orientation} the {location}.")
            return Status.EXECUTION_SUCCESS, location, orientation
        else:
            self.hri_manager.say("Let me try to understand the placement again.")
            # Recursive call to try again (with a maximum of one retry to avoid infinite loops)
            if not hasattr(self, "_retry_count"):
                self._retry_count = 0

            if self._retry_count < 1:
                self._retry_count += 1
                result = self.get_complete_placement_info(room)
                self._retry_count = 0  # Reset counter
                return result
            else:
                self.hri_manager.say(
                    "I'm having trouble understanding. Using default placement nearby the object."
                )
                return Status.EXECUTION_ERROR, location or "object", "nearby"

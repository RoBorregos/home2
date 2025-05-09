def get_gpsr_comands(command_type: str, structured_cmd=True):
    """
    Function to get GPSR commands based on the command type.
    :param command_type: The type of command to retrieve.
    :return: A list of GPSR commands matching the command type.
    """

    if command_type == "custom":
        return {"commands": custom_command}

    for command in GPSR_COMMANDS:
        if command["cmd_type"] == command_type:
            if structured_cmd:
                return {"commands": command["structured_cmd"]}
            return command
    return None

    # "visual_info",
    # "guide_to",
    # "find_object",
    # "find_person",
    # "find_person_by_name",
    # # "follow_person_until",
    # "count",
    # # "contextual_say",


custom_command = [
    {"action": "pick_object", "object_to_pick": "zucaritas"},
    # {"action": "answer_question"},
    #  get_person_info
    #  {"action": "get_person_info", "info_type": "name"},
    # {"action": "get_person_info", "info_type": "name"},
    # {"action": "get_person_info", "info_type": "pose"},
    # {"action": "get_person_info", "info_type": "gesture"},
    # Counting
    # {"action": "count", "target_to_count": "people pointing right"},
    # {"action": "count", "target_to_count": "drinks"},
    # {"action": "count", "target_to_count": "toys"},
    # {"action": "count", "target_to_count": "sitting persons"},
    # {"action": "pick_object", "object_to_pick": "lime"},
    # {"action": "count", "target_to_count": "sitting persons"},
    # {
    #             "action": "say_with_context",
    #             "user_instruction": "tell me how many sitting persons are in the living room",
    #             "previous_command_info": ["count"],
    #         },
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "say what day is tomorrow to the person pointing to the left in the office",
    #     "previous_command_info": ["what day is tomorrow"],
    # },
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "salute the person wearing a blue shirt in the office and say your teams country",
    #     "previous_command_info": ["your teams country"],
    # },
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "meet Morgan in the bedroom and tell something about yourself",
    #     "previous_command_info": ["something about yourself"],
    # },
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "say the time to the person raising their right arm in the bathroom",
    #     "previous_command_info": ["the time"],
    # },
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "look for a person pointing to the left in the bedroom and tell the day of the month",
    #     "previous_command_info": ["the day of the month"],
    # },
    # {"action": "give_object"},
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "say what day is tomorrow to the person pointing to the left in the office",
    #     "previous_command_info": ["what day is tomorrow"],
    # },
    # {
    #     "action": "count",
    #     "target_to_count": "waving",
    # },
    # {
    #     "action": "find_person",
    #     "attribute_value": "waving person",
    # }
    # {
    #     "action": "say_with_context",
    #     "user_instruction": "say what day is tomorrow to the person pointing to the left in the office",
    #     "previous_command_info": "what day is tomorrow",
    # },
    # Manipulation
    # Vision
    # {"action": "get_person_info", "info_type": "pose"},
    # {"action": "get_person_info", "info_type": "pose"},
    # {"action": "get_person_info", "info_type": "pose"},
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "waving",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "raise right arm",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "raise left arm",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "pointing left",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "pointing right",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "waving",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "seating",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "standing",
    # },
    # {
    #     "action": "count",
    #     "complement": "sota corner",
    #     "characteristic": "lying",
    # },
    # Nav
    # HRI
]

GPSR_COMMANDS = [
    {
        "cmd_type": "goToLoc",
        "string_cmd": "go to the sofa then find a cleanser and take it and deliver it to me",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "sofa"},
            {"action": "pick_object", "object_to_pick": "cleanser"},
            {"action": "give_object"},
        ],
    },
    {
        "cmd_type": "takeObjFromPlcmt",
        "string_cmd": "get a strawberry from the kitchen table and bring it to the standing person in the office",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "kitchen table"},
            {"action": "pick_object", "object_to_pick": "strawberry"},
            {"action": "go_to", "location_to_go": "office"},
            {"action": "find_person", "attribute_value": "standing person"},
            {"action": "give_object"},
        ],
    },
    {
        "cmd_type": "findPrsInRoom",
        "string_cmd": "locate a lying person in the office and guide them to the dishwasher",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "office"},
            {"action": "find_person", "attribute_value": "lying person"},
            {"action": "guide_person_to", "destination_room": "dishwasher"},
        ],
    },
    {
        "cmd_type": "findObjInRoom",
        "string_cmd": "locate a sugar in the living room then take it and place it on the desk",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "living room"},
            {"action": "pick_object", "object_to_pick": "sugar"},
            {"action": "go_to", "location_to_go": "desk"},
            {"action": "place_object"},
        ],
    },
    {
        "cmd_type": "meetPrsAtBeac",
        "string_cmd": "meet Jane in the bathroom and follow them",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "bathroom"},
            {"action": "find_person_by_name", "name": "Jane"},
            {"action": "follow_person_until", "destination": "canceled"},
        ],
    },
    {
        "cmd_type": "countObjOnPlcmt",
        "string_cmd": "tell me how many drinks there are on the tv stand",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "tv stand"},
            {"action": "count", "target_to_count": "drinks"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me how many drinks there are on the tv stand",
                "previous_command_info": "count",
            },
        ],
    },
    {
        "cmd_type": "countPrsInRoom",
        "string_cmd": "tell me how many standing persons are in the living room",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "living room"},
            {"action": "count", "target_to_count": "standing persons"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me how many standing persons are in the living room",
                "previous_command_info": "count",
            },
        ],
    },
    {
        "cmd_type": "tellPrsInfoInLoc",
        "string_cmd": "tell me the gesture of the person at the waste basket",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "waste basket"},
            {"action": "find_person", "attribute_value": ""},
            {"action": "get_person_info", "info_type": "gesture"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me the gesture of the person at the waste basket",
                "previous_command_info": "get_person_info",
            },
        ],
    },
    {
        "cmd_type": "tellObjPropOnPlcmt",
        "string_cmd": "tell me what is the thinnest object on the bed",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "bed"},
            {"action": "get_visual_info", "measure": "thinnest", "object_category": "object"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me what is the thinnest object on the bed",
                "previous_command_info": "get_visual_info",
            },
        ],
    },
    {
        "cmd_type": "talkInfoToGestPrsInRoom",
        "string_cmd": "say what day is tomorrow to the person pointing to the left in the office",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "office"},
            {"action": "find_person", "attribute_value": "person pointing to the left"},
            {
                "action": "say_with_context",
                "user_instruction": "say what day is tomorrow to the person pointing to the left in the office",
                "previous_command_info": "what day is tomorrow",
            },
        ],
    },
    {
        "cmd_type": "answerToGestPrsInRoom",
        "string_cmd": "answer the quiz of the person raising their left arm in the living room",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "living room"},
            {"action": "find_person", "attribute_value": "person raising their left arm"},
            {"action": "answer_question"},
        ],
    },
    {
        "cmd_type": "followNameFromBeacToRoom",
        "string_cmd": "follow Paris from the sink to the office",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "sink"},
            {"action": "find_person_by_name", "name": "Paris"},
            {"action": "follow_person_until", "destination": "office"},
        ],
    },
    {
        "cmd_type": "guideNameFromBeacToBeac",
        "string_cmd": "take Simone from the sofa to the office",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "sofa"},
            {"action": "find_person_by_name", "name": "Simone"},
            {"action": "guide_person_to", "destination_room": "office"},
        ],
    },
    {
        "cmd_type": "guidePrsFromBeacToBeac",
        "string_cmd": "lead the lying person from the desk to the exit",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "desk"},
            {"action": "find_person", "attribute_value": "lying person"},
            {"action": "guide_person_to", "destination_room": "exit"},
        ],
    },
    {
        "cmd_type": "guideClothPrsFromBeacToBeac",
        "string_cmd": "lead the person wearing a white coat from the sofa to the living room",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "sofa"},
            {"action": "find_person", "attribute_value": "white coat"},
            {"action": "guide_person_to", "destination_room": "living room"},
        ],
    },
    {
        "cmd_type": "bringMeObjFromPlcmt",
        "string_cmd": "bring me a peach from the bed",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "bed"},
            {"action": "pick_object", "object_to_pick": "peach"},
            {"action": "go_to", "location_to_go": "start_location"},
            {"action": "give_object"},
        ],
    },
    {
        "cmd_type": "tellCatPropOnPlcmt",
        "string_cmd": "tell me what is the lightest dish on the side tables",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "side tables"},
            {"action": "get_visual_info", "measure": "lightest", "object_category": "dish"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me what is the lightest dish on the side tables",
                "previous_command_info": "get_visual_info",
            },
        ],
    },
    {
        "cmd_type": "greetClothDscInRm",
        "string_cmd": "greet the person wearing a black sweater in the bedroom and follow them to the shelf",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "bedroom"},
            {"action": "find_person", "attribute_value": "black sweater"},
            {
                "action": "say_with_context",
                "user_instruction": "greet the person wearing a black sweater in the bedroom and follow them to the shelf",
                "previous_command_info": "introduction",
            },
            {"action": "follow_person_until", "destination": "shelf"},
        ],
    },
    {
        "cmd_type": "greetNameInRm",
        "string_cmd": "salute Jane in the office and take them to the storage rack",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "office"},
            {"action": "find_person_by_name", "name": "Jane"},
            {
                "action": "say_with_context",
                "user_instruction": "salute Jane in the office and take them to the storage rack",
                "previous_command_info": "introduction",
            },
            {"action": "guide_person_to", "destination_room": "storage rack"},
        ],
    },
    {
        "cmd_type": "meetNameAtLocThenFindInRm",
        "string_cmd": "meet Robin at the waste basket then look for them in the bathroom",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "waste basket"},
            {"action": "find_person_by_name", "name": "Robin"},
            {"action": "go_to", "location_to_go": "bathroom"},
            {"action": "find_person_by_name", "name": "Robin"},
        ],
    },
    {
        "cmd_type": "countClothPrsInRoom",
        "string_cmd": "tell me how many people in the bathroom are wearing black sweaters",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "bathroom"},
            {"action": "count", "target_to_count": "people wearing black sweaters"},
            {"action": "go_to", "location_to_go": "start_location"},
            {
                "action": "say_with_context",
                "user_instruction": "tell me how many people in the bathroom are wearing black sweaters",
                "previous_command_info": "count",
            },
        ],
    },
    {
        "cmd_type": "tellPrsInfoAtLocToPrsAtLoc",
        "string_cmd": "tell the gesture of the person at the side tables to the person at the bookshelf",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "side tables"},
            {"action": "find_person", "attribute_value": ""},
            {"action": "get_person_info", "info_type": "gesture"},
            {"action": "go_to", "location_to_go": "bookshelf"},
            {"action": "find_person", "attribute_value": ""},
            {
                "action": "say_with_context",
                "user_instruction": "tell the gesture of the person at the side tables to the person at the bookshelf",
                "previous_command_info": "get_person_info",
            },
        ],
    },
    {
        "cmd_type": "followPrsAtLoc",
        "string_cmd": "follow the person pointing to the right at the refrigerator",
        "structured_cmd": [
            {"action": "go_to", "location_to_go": "refrigerator"},
            {"action": "find_person", "attribute_value": "person pointing to the right"},
            {"action": "follow_person_until", "destination": "canceled"},
        ],
    },
]


AVAILABLE_COMMANDS = {
    "give",
    "find_person_info",
    "go",
    "pick",
    "place",
    "say",
    "ask_answer_question",
    "visual_info",
    "guide_to",
    "find_object",
    "find_person",
    "find_person_by_name",
    # "follow_person_until",
    "count",
    # "contextual_say",
}

if __name__ == "__main__":
    command_statistics = []

    for command in GPSR_COMMANDS:
        cmd_type = command["cmd_type"]
        structured_cmd = command["structured_cmd"]

        implemented_count = 0
        total_count = len(structured_cmd)

        for sub_cmd in structured_cmd:
            if sub_cmd["action"] in AVAILABLE_COMMANDS:
                implemented_count += 1

        command_statistics.append(
            {
                "cmd_type": cmd_type,
                "implemented": implemented_count,
                "total": total_count,
                "percentage": (implemented_count / total_count) * 100 if total_count > 0 else 0,
            }
        )

    # Sort by percentage of implementation (ascending)
    command_statistics.sort(key=lambda x: -x["percentage"])

    # ANSI color codes
    RED = "\033[91m"
    ORANGE = "\033[93m"
    GREEN = "\033[92m"
    RESET = "\033[0m"

    print("\nCommand Implementation Status:\n")
    for stat in command_statistics:
        cmd_type = stat["cmd_type"]
        implemented = stat["implemented"]
        total = stat["total"]
        percentage = stat["percentage"]

        if implemented == total:
            color = GREEN
        elif percentage >= 50:
            color = ORANGE
        else:
            color = RED

        print(f"{color}{implemented}/{total} ({percentage:.1f}%) - {cmd_type}{RESET}")

    total_implemented = sum(stat["implemented"] for stat in command_statistics)
    total_commands = sum(stat["total"] for stat in command_statistics)
    total_percentage = (total_implemented / total_commands) * 100 if total_commands > 0 else 0

    print(
        f"\nOverall implementation: {total_implemented}/{total_commands} ({total_percentage:.1f}%)"
    )

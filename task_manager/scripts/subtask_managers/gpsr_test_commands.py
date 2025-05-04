def get_gpsr_comands(command_type: str, structured_cmd=True):
    """
    Function to get GPSR commands based on the command type.
    :param command_type: The type of command to retrieve.
    :return: A list of GPSR commands matching the command type.
    """
    for command in GPSR_COMMANDS:
        if command["cmd_type"] == command_type:
            if structured_cmd:
                return command["structured_cmd"]
            return command
    return None


GPSR_COMMANDS = [
    {
        "cmd_type": "goToLoc",
        "string_cmd": "go to the bathroom then locate a rubiks cube and take it and place it on the pantry",
        "structured_cmd": [
            {"action": "go", "complement": "bathroom", "characteristic": ""},
            {"action": "find_object", "complement": "bathroom", "characteristic": "rubiks cube"},
            {"action": "pick", "complement": "rubiks cube", "characteristic": ""},
            {"action": "go", "complement": "pantry", "characteristic": ""},
            {"action": "place", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "takeObjFromPlcmt",
        "string_cmd": "take a food from the refrigerator and give it to the lying person in the office",
        "structured_cmd": [
            {"action": "go", "complement": "refrigerator", "characteristic": ""},
            {"action": "find_object", "complement": "refrigerator", "characteristic": "food"},
            {"action": "pick", "complement": "food", "characteristic": ""},
            {"action": "go", "complement": "office", "characteristic": ""},
            {"action": "find_person", "complement": "lying person", "characteristic": ""},
            {"action": "give", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "findPrsInRoom",
        "string_cmd": "look for a person pointing to the left in the bedroom and guide them to the desk",
        "structured_cmd": [
            {"action": "go", "complement": "bedroom", "characteristic": ""},
            {
                "action": "find_person",
                "complement": "person pointing to the left",
                "characteristic": "",
            },
            {"action": "guide_to", "complement": "person", "characteristic": "desk"},
        ],
    },
    {
        "cmd_type": "findObjInRoom",
        "string_cmd": "locate a tomato soup in the kitchen then take it and deliver it to Axel in the living room",
        "structured_cmd": [
            {"action": "go", "complement": "kitchen", "characteristic": ""},
            {"action": "find_object", "complement": "kitchen", "characteristic": "tomato soup"},
            {"action": "pick", "complement": "tomato soup", "characteristic": ""},
            {"action": "go", "complement": "living room", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Axel", "characteristic": ""},
            {"action": "give", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "meetPrsAtBeac",
        "string_cmd": "meet Robin in the living room and follow them",
        "structured_cmd": [
            {"action": "go", "complement": "living room", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Robin", "characteristic": ""},
            {"action": "follow_person_until", "complement": "canceled", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "countObjOnPlcmt",
        "string_cmd": "tell me how many fruits there are on the kitchen table",
        "structured_cmd": [
            {"action": "go", "complement": "kitchen table", "characteristic": ""},
            {"action": "count", "complement": "kitchen table", "characteristic": "fruits"},
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me how many fruits there are on the kitchen table",
                "characteristic": "count",
            },
        ],
    },
    {
        "cmd_type": "countPrsInRoom",
        "string_cmd": "tell me how many persons raising their right arm are in the office",
        "structured_cmd": [
            {"action": "go", "complement": "office", "characteristic": ""},
            {
                "action": "count",
                "complement": "office",
                "characteristic": "person with persons raising their right arm",
            },
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me how many persons raising their right arm are in the office",
                "characteristic": "count",
            },
        ],
    },
    {
        "cmd_type": "tellPrsInfoInLoc",
        "string_cmd": "tell me the pose of the person at the potted plant",
        "structured_cmd": [
            {"action": "go", "complement": "potted plant", "characteristic": ""},
            {"action": "find_person", "complement": "", "characteristic": ""},
            {"action": "find_person_info", "complement": "pose", "characteristic": ""},
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me the pose of the person at the potted plant",
                "characteristic": "find_person_info",
            },
        ],
    },
    {
        "cmd_type": "tellObjPropOnPlcmt",
        "string_cmd": "tell me what is the lightest object on the desk",
        "structured_cmd": [
            {"action": "go", "complement": "desk", "characteristic": ""},
            {"action": "visual_info", "complement": "lightest", "characteristic": ""},
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me what is the lightest object on the desk",
                "characteristic": "visual_info",
            },
        ],
    },
    {
        "cmd_type": "talkInfoToGestPrsInRoom",
        "string_cmd": "say what day is today to the person raising their right arm in the bedroom",
        "structured_cmd": [
            {"action": "go", "complement": "bedroom", "characteristic": ""},
            {
                "action": "find_person",
                "complement": "person raising their right arm",
                "characteristic": "",
            },
            {
                "action": "contextual_say",
                "complement": "say what day is today to the person raising their right arm in the bedroom",
                "characteristic": "what day is today",
            },
        ],
    },
    {
        "cmd_type": "answerToGestPrsInRoom",
        "string_cmd": "answer the quiz of the person raising their right arm in the bedroom",
        "structured_cmd": [
            {"action": "go", "complement": "bedroom", "characteristic": ""},
            {
                "action": "find_person",
                "complement": "person raising their right arm",
                "characteristic": "",
            },
            {"action": "ask_answer_question", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "followNameFromBeacToRoom",
        "string_cmd": "follow Jules from the side tables to the living room",
        "structured_cmd": [
            {"action": "go", "complement": "side tables", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Jules", "characteristic": ""},
            {
                "action": "follow_person_until",
                "complement": "living room",
                "characteristic": "Jules",
            },
        ],
    },
    {
        "cmd_type": "guideNameFromBeacToBeac",
        "string_cmd": "escort Simone from the coatrack to the office",
        "structured_cmd": [
            {"action": "go", "complement": "coatrack", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Simone", "characteristic": ""},
            {"action": "guide_to", "complement": "Simone", "characteristic": "office"},
        ],
    },
    {
        "cmd_type": "guidePrsFromBeacToBeac",
        "string_cmd": "escort the person pointing to the left from the bed to the storage rack",
        "structured_cmd": [
            {"action": "go", "complement": "bed", "characteristic": ""},
            {
                "action": "find_person",
                "complement": "person pointing to the left",
                "characteristic": "",
            },
            {"action": "guide_to", "complement": "person", "characteristic": "storage rack"},
        ],
    },
    {
        "cmd_type": "guideClothPrsFromBeacToBeac",
        "string_cmd": "take the person wearing a red jacket from the potted plant to the bedroom",
        "structured_cmd": [
            {"action": "go", "complement": "potted plant", "characteristic": ""},
            {"action": "find_person", "complement": "red jacket", "characteristic": "clothes"},
            {"action": "guide_to", "complement": "person", "characteristic": "bedroom"},
        ],
    },
    {
        "cmd_type": "bringMeObjFromPlcmt",
        "string_cmd": "bring me a strawberry from the bedside table",
        "structured_cmd": [
            {"action": "go", "complement": "bedside table", "characteristic": ""},
            {
                "action": "find_object",
                "complement": "bedside table",
                "characteristic": "strawberry",
            },
            {"action": "pick", "complement": "strawberry", "characteristic": ""},
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {"action": "give", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "tellCatPropOnPlcmt",
        "string_cmd": "tell me what is the largest toy on the tv stand",
        "structured_cmd": [
            {"action": "go", "complement": "tv stand", "characteristic": ""},
            {"action": "visual_info", "complement": "largest", "characteristic": "toy"},
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me what is the largest toy on the tv stand",
                "characteristic": "visual_info",
            },
        ],
    },
    {
        "cmd_type": "greetClothDscInRm",
        "string_cmd": "greet the person wearing a gray coat in the office and answer a question",
        "structured_cmd": [
            {"action": "go", "complement": "office", "characteristic": ""},
            {"action": "find_person", "complement": "gray coat", "characteristic": "clothes"},
            {
                "action": "contextual_say",
                "complement": "greet the person wearing a gray coat in the office and answer a question",
                "characteristic": "introduction",
            },
            {"action": "ask_answer_question", "complement": "", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "greetNameInRm",
        "string_cmd": "greet Axel in the kitchen and follow them to the dishwasher",
        "structured_cmd": [
            {"action": "go", "complement": "kitchen", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Axel", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "greet Axel in the kitchen and follow them to the dishwasher",
                "characteristic": "introduction",
            },
            {"action": "follow_person_until", "complement": "dishwasher", "characteristic": ""},
        ],
    },
    {
        "cmd_type": "meetNameAtLocThenFindInRm",
        "string_cmd": "meet Simone at the bookshelf then find them in the bedroom",
        "structured_cmd": [
            {"action": "go", "complement": "bookshelf", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Simone", "characteristic": ""},
            {
                "action": "say",
                "complement": "Hi Simone, I'll find you at bedroom",
                "characteristic": "",
            },
            {"action": "go", "complement": "bedroom", "characteristic": ""},
            {"action": "find_person_by_name", "complement": "Simone", "characteristic": ""},
            {
                "action": "say",
                "complement": "Hi Simone, nice to see you again!",
                "characteristic": "",
            },
        ],
    },
    {
        "cmd_type": "countClothPrsInRoom",
        "string_cmd": "tell me how many people in the bedroom are wearing red t shirts",
        "structured_cmd": [
            {"action": "go", "complement": "bedroom", "characteristic": ""},
            {
                "action": "count",
                "complement": "bedroom",
                "characteristic": "person with red t shirts",
            },
            {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me how many people in the bedroom are wearing red t shirts",
                "characteristic": "count",
            },
        ],
    },
    {
        "cmd_type": "tellPrsInfoAtLocToPrsAtLoc",
        "string_cmd": "tell the gesture of the person at the waste basket to the person at the tv stand",
        "structured_cmd": [
            {"action": "go", "complement": "waste basket", "characteristic": ""},
            {"action": "find_person", "complement": "", "characteristic": ""},
            {"action": "find_person_info", "complement": "gesture", "characteristic": ""},
            {"action": "go", "complement": "tv stand", "characteristic": ""},
            {"action": "find_person", "complement": "", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell the gesture of the person at the waste basket to the person at the tv stand",
                "characteristic": "find_person_info",
            },
        ],
    },
    {
        "cmd_type": "followPrsAtLoc",
        "string_cmd": "follow the standing person at the tv stand",
        "structured_cmd": [
            {"action": "go", "complement": "tv stand", "characteristic": ""},
            {"action": "find_person", "complement": "standing person", "characteristic": ""},
            {"action": "follow_person_until", "complement": "canceled", "characteristic": ""},
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

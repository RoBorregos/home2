import json
from nlp.assets.schemas import ExtractedData, IsAnswerPositive, RoomIdentification


def get_common_interests_dialog(
    person1Name, person2Name, person1Interests, person2Interests
):
    return {
        "messages": [
            {
                "role": "system",
                "content": "You will be presented with the interests of two people, your task is to get the common interests between them. Give a short answer with one common interest.",
            },
            {
                "role": "user",
                "content": f"{person1Name} likes {person1Interests} and {person2Name} likes {person2Interests}",
            },
        ]
    }


def get_extract_data_args(full_text, data_to_extract):
    return (
        [
            {
                "role": "system",
                "content": f"""You will be given a piece of text and a specific data item to extract. Your task is to return the requested information if it is explicitly present in the provided text.
### Extraction Rules:
- If the requested data is found within the text, return it as the output.
- If the data is not present, return an empty string (`""`).
- If `full_text` is missing or empty, return an empty string (`""`).  
- If `full_text` is a single word or a short phrase and corresponds to the data to extract, you can return the full_text.

### Examples:

**Input:**
<full_text>
    There is a cat in the house.
</full_text>
<extract_data>
    food
</extract_data>

**Output:**
{ExtractedData(data="").model_dump_json()}

**Input:**
<full_text>
    The restaurant serves delicious Italian food.
</full_text>
<extract_data>
    food
</extract_data>

**Output:**
{ExtractedData(data="Italian food").model_dump_json()} 

**Input:**
<full_text>
    My name is Juan and I like lemonade.
</full_text>
<extract_data>
    drink
</extract_data>

**Output:**
{ExtractedData(data="lemonade").model_dump_json()}

**Input:**
<full_text>
    Juan and I like to play basketball.
</full_text>
<extract_data>
    name
</extract_data>

**Output:**
{ExtractedData(data="Juan").model_dump_json()}

**Input:**
<full_text>
    Elis
</full_text>
<extract_data>
    name
</extract_data>

**Output:**
{ExtractedData(data="Elis").model_dump_json()}  

Ensure strict adherence to these rules. Do not infer or generate information beyond what is explicitly stated in the text.""",
            },
            {
                "role": "user",
                "content": "<full_text>"
                + str(full_text)
                + "</full_text>"
                + "\n"
                + "<extract_data>"
                + (data_to_extract)
                + "</extract_data>",
            },
        ],
        ExtractedData,
    )


def get_is_answer_positive_args(interpreted_text):
    return (
        [
            {
                "role": "system",
                "content": f"""You will be given a statement, and your task is to determine whether it is a **positive confirmation** or not. A **positive confirmation** is an explicit or implicit agreement, affirmation, or confirmation (e.g., 'yes', 'that's right', 'correct', 'absolutely'). A **negative response** includes disagreement, uncertainty, negation, or lack of understanding (e.g., 'no', 'I don’t know', 'wrong', 'not sure'). If the statement is ambiguous or unclear, assume it is negative.
                
### Guidelines:
- Only return a boolean value: **true** for positive confirmation, **false** otherwise.
- Ignore irrelevant sentiment (e.g., 'I am happy' is **not** a confirmation, so return false).
- Consider implicit confirmations like 'exactly' or 'of course' as positive.
- Treat uncertain responses like 'maybe' or 'I guess' as negative.
                
### Examples:
- **Input:** 'Yes'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'That's correct'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'I agree'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'Wrong'
{IsAnswerPositive(is_positive=False).model_dump_json()}
                
**Input:** 'I don’t know'
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Huh?'
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Maybe'
{IsAnswerPositive(is_positive=False).model_dump_json()}
""",
            },
            {
                "role": "user",
                "content": interpreted_text,
            },
        ],
        IsAnswerPositive,
    )


def get_room_identification_dialog(comment, areas_file="areas.json"):
    with open(areas_file, "r") as file:
        areas = json.load(file)

    rooms_and_sub_areas = []
    for room, sub_areas in areas.items():
        rooms_and_sub_areas.append(room.replace("_", " ").title())
        for sub_area in sub_areas.keys():
            rooms_and_sub_areas.append(
                f"{room.replace('_', ' ').title()} - {sub_area.replace('_', ' ').title()}"
            )

    return {
        "messages": [
            {
                "role": "system",
                "content": f"You will be given a comment or question. Your task is to identify which room or sub-area is being referred to. The possible options are: {', '.join(rooms_and_sub_areas)}. Provide the name of the room or sub-area as the answer.",
            },
            {
                "role": "user",
                "content": comment,
            },
        ],
        "schema": RoomIdentification,
    }

from nlp.assets.schemas import ExtractedData, IsAnswerNegative, IsAnswerPositive


def get_common_interests_dialog(
    person1Name, person2Name, person1Interests, person2Interests
):
    return {
        "messages": [
            {
                "role": "system",
                "content": f"You will be presented with the interests of two people, your task is to get the common interests between them. Give me an answer in the format similar to '{person1Name} and {person2Name}'s common interest is movies' or '{person1Name} and {person2Name} don't have a common interest between them' in case they don't share one",
            },
            {
                "role": "user",
                "content": f"{person1Name} likes {person1Interests} and {person2Name} likes {person2Interests}",
            },
        ]
    }


def get_extract_data_args(full_text, data_to_extract, context=None):
    user_content = f"<full_text>{full_text}</full_text>\n<extract_data>{data_to_extract}</extract_data>"

    if context and len(context) > 0:
        user_content += f"\n<explanation>{context}</explanation>"

    return (
        [
            {
                "role": "system",
                "content": f"""You will receive a text (`full_text`) and a specific target (`extract_data`). (Optional) Additional explanation (`explanation`) to help clarify ambiguous cases. Your task is to extract and return the closest relevant word or phrase that directly answers the target.

### Extraction Rules:
- Return the MOST RELEVANT WORD OR PHRASE that best corresponds to `extract_data`, considering its contextual meaning within the sentence.
- Do NOT return the target word (`extract_data`) itself unless it is the best available answer.
- If multiple possible matches exist, return the MOST CONTEXTUALLY RELEVANT one (e.g., a noun or phrase describing the requested information).
- If no relevant match is found, return an empty string (`""`).
- If `full_text` is missing, empty, or consists of only a single word or short phrase that directly corresponds to `extract_data`, return `full_text` as the result.
- If present, use the `explanation` to help clarify ambiguous cases.

### Examples:


#### Example 1:
**INPUT:**
<full_text>
    There is a cat in the house.
</full_text>
<extract_data>
    food
</extract_data>

**OUTPUT:**
{ExtractedData(data="").model_dump_json()}

#### Example 2:
**INPUT:**
<full_text>
    The restaurant serves delicious Italian food.
</full_text>
<extract_data>
    food
</extract_data>

**OUTPUT:**
{ExtractedData(data="Italian food").model_dump_json()} 

#### Example 3:
**INPUT:**
<full_text>
    My name is Juan and I like lemonade.
</full_text>
<extract_data>
    drink
</extract_data>

**OUTPUT:**
{ExtractedData(data="lemonade").model_dump_json()}

#### Example 4:
**INPUT:**
<full_text>
    Juan and I like to play basketball.
</full_text>
<extract_data>
    name
</extract_data>

**OUTPUT:**
{ExtractedData(data="Juan").model_dump_json()}

#### Example 5:
**INPUT:**
<full_text>
    Elis
</full_text>
<extract_data>
    name
</extract_data>

**OUTPUT:**
{ExtractedData(data="Elis").model_dump_json()}  

Ensure that the extracted data is always **the most contextually relevant** answer, not simply the target term itself.""",
            },
            {
                "role": "user",
                "content": user_content,
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
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}
                
**Input:** 'I don’t know'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Huh?'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Maybe'
**Output:**
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


def get_is_answer_negative_args(interpreted_text):
    return (
        [
            {
                "role": "system",
                "content": f"""You will receive a short statement. Your task is to determine if the statement is a **negative confirmation**.

A negative confirmation includes:
- explicit disagreement or rejection (e.g., "no", "that's wrong", "incorrect")

But does **not** include:
- expressions of uncertainty or lack of confidence (e.g., "maybe", "I'm not sure", "I don't think so", "I doubt it")
- explicit affirmation (e.g. "yes", "correct", "sure", "absolutely", "of course")

**Always respond with a single JSON object with one field `is_negative` (boolean).**
                
### Examples:
- **Input:** 'Yes'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'That's not correct'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'I don't agree'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'Wrong'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}
                
**Input:** 'I don't know'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'That's right'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'Maybe'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'No'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'Absolutely'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}
""",
            },
            {
                "role": "user",
                "content": interpreted_text,
            },
        ],
        IsAnswerNegative,
    )


def format_response(response):
    return [
        {
            "role": "system",
            "content": "You will be presented with an output. Your task is to format the response according to the given format.",
        },
        {
            "role": "user",
            "content": response,
        },
    ]


def get_command_interpreter_args(full_text):
    return [
        {
            "role": "user",
            "content": """
Respond in the following format:
<think>
...
</think>
<answer>
...
</answer>

For the answer it should be a json that can be parsed as a CommandListShape.

class CommandShape(BaseModel):
    action: str
    characteristic: str = ""
    complement: str = ""

class CommandListShape(BaseModel):
    commands: List[CommandShape]

These are the actions that you can use:

1. go (complement = location_to_go, characteristic = "")
2. pick(complement = "object to pick", characteristic = "")
3. place(complement = "", characteristic = "")
4. contextual_say(complement = complete_command, characteristic = previous_command | information_to_answer)
5. say(complement = text, characteristic = "")
6. ask_answer_question(complement = "", characteristic = "")
7. visual_info(complement = ("biggest" | "largest" | "smallest" | "heaviest" | "lightest" | "thinnest") + " object" | "describe the person name" | "describe the person posture", characteristic = object_category | "")
8. give(complement = "", characteristic = "")
9. follow_person_until(complement = room | location | 'canceled', characteristic = name | "")
10. guide_to(complement = "person" | name, characteristic = loc_room)
11. find_person_info(complement = "name" | "pose" | "gesture", characteristic = "")
12. find_object(complement = placement_location | room, characteristic = object_to_find)
13. count(complement = placement_location | room, characteristic = object | "person with " + cloth + color | "person with " + posture | "person with " + gesture)
14. find_person(complement = gesture | posture | cloth + color | "", characteristic = "clothes" | "gesture" | "posture" | "")
15. find_person_by_name(complement = name, characteristic = "")

For example, for the prompt "fetch a pringles from the kitchen table and deliver it to the person pointing to the left in the bedroom", your answer should be:

<think>
Okay, let me break down the user's task step by step. The user wants the robot to fetch a Pringles from the kitchen table and deliver it to a person in the bedroom who's pointing to the left.

First, the robot needs to navigate to the kitchen table. So the first action would be 'go' with the complement 'kitchen table'. Then, it needs to find the Pringles there. Since Pringles is a specific object, the action is 'find' with 'pringles' as the complement. Once located, the robot should grab it, so 'grab' and 'pringles' next.

Now, the robot needs to go to the bedroom. So another 'go' action with 'bedroom' as the complement. In the bedroom, the robot has to identify the person. The user mentioned the person is pointing to the left. Here, the robot should use person recognition with the property 'pointing to the left'. So 'find' action, complement 'person', and property 'pointing to the left'.

After locating the person, the robot should approach them. Then deliver the Pringles. So 'approach' and 'person', followed by 'give' and 'pringles'.

Wait, but in the previous examples, when delivering to a named person, they used 'find' with the person's name. Here, since the person isn't named but identified by a gesture, the complement remains 'person' with the property specifying the gesture. That makes sense. Also, the robot doesn't need to return to a previous location here because the task ends after delivering. So no 'remember location' steps unless necessary. In the example where they went to the bedroom, they didn't have to go back. So the steps would be: go to kitchen table, find pringles, grab, go to bedroom, find person (with property), approach, give. That should be it.
</think>
<answer>
{\"commands\":[{\"action\":\"go\",\"complement\":\"kitchen table\",\"characteristic\":\"\"},{\"action\":\"find_object\",\"complement\":\"kitchen table\",\"characteristic\":\"pringles\"},{\"action\":\"pick\",\"complement\":\"pringles\",\"characteristic\":\"\"},{\"action\":\"go\",\"complement\":\"bedroom\",\"characteristic\":\"\"},{\"action\":\"find_person\",\"complement\":\"person pointing to the left\",\"characteristic\":\"\"},{\"action\":\"give\",\"complement\":\"\",\"characteristic\":\"\"}]}
</answer>
            """,
        },
        {"role": "user", "content": full_text},
    ]

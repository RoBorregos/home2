from nlp.assets.schemas import (
    CategorizeShelvesResult,
    ExtractedData,
    IsAnswerNegative,
    IsAnswerPositive,
)


def get_common_interests_dialog(
    person1Name, person2Name, person1Interests, person2Interests
):
    return {
        "messages": [
            {
                "role": "system",
                "content": f"""You will be presented with the interests of two people, your task is to get the common interests between them.
Try to ALWAYS find a common interest, only rely in specifying that there is no common interest if there is absolutely no relation at all between both of their interests. For example, if {person1Name} likes 'burgers' and {person2Name} likes 'pizza', you can say that they share a common interest in 'food'.

Always provide an answer in the format: '{person1Name} and {person2Name}'s common interest is movies' if there is a common interest or '{person1Name} and {person2Name} don't have a common interest between them' in case they don't share one.
Do not add any other information or context to the answer, just the common interest or the lack of it.
""",
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


def get_categorize_shelves_args(shelves, table_objects):
    return (
        [
            {
                "role": "system",
                "content": """You are tasked with categorizing grocery objects onto shelves based on similarity.

Given:
- A set of existing shelves, where each shelf contains a list of objects.
- A list of table objects that need to be assigned to the correct shelf.

Instructions:
- Group each table object with the shelf whose existing objects are most similar.
- If a table object isn't similar to any of the objects in the shelves, you may assign it to the empty shelf.
- Provide, for each shelf:
  1. 'objects_to_add': a list of new objects (from the table) that should be placed on that shelf.
  2. 'classification_tag': a short descriptive name of the shelf's category (e.g., "dairy", "fruit", "snacks").
- For the empty shelf, you can assign any object from the table that doesn't fit into the other shelves.

Output format:
A dictionary where:
- The keys are shelf numbers (integers).
- The values are objects with two properties: 'objects_to_add' and 'classification_tag'.

Example:
Shelves:
{
  1: ["milk", "buttermilk"],
  2: [],
  3: ["apple", "banana"]
}

Table objects:
["butter", "orange", "cookies", "cheese", "watermelon", "pringles"]

Expected output:
"""
                + CategorizeShelvesResult(
                    shelves={
                        1: {
                            "objects_to_add": ["butter", "cheese"],
                            "classification_tag": "dairy",
                        },
                        2: {
                            "objects_to_add": ["cookies", "pringles"],
                            "classification_tag": "snacks",
                        },
                        3: {
                            "objects_to_add": ["orange", "watermelon"],
                            "classification_tag": "fruit",
                        },
                    }
                ).model_dump_json(),
            },
            {
                "role": "user",
                "content": f"Shelves: {shelves}, Table objects: {table_objects}",
            },
        ],
        CategorizeShelvesResult,
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
5. ask_answer_question(complement = "", characteristic = "")
6. visual_info(complement = ("biggest" | "largest" | "smallest" | "heaviest" | "lightest" | "thinnest"), characteristic = object_category | "object")
7. give(complement = "", characteristic = "")
8. follow_person_until(complement = room | location | 'canceled', characteristic = name | "")
9. guide_to(complement = "person" | name, characteristic = loc_room)
10. find_person_info(complement = "name" | "pose" | "gesture", characteristic = "")
11. find_object(complement = placement_location | room, characteristic = object_to_find)
12. count(complement = placement_location | room, characteristic = object | "person with " + cloth + color | "person with " + posture | "person with " + gesture)
13. find_person(complement = gesture | posture | cloth + color | "", characteristic = "clothes" | "gesture" | "posture" | "")
14. find_person_by_name(complement = name, characteristic = "")

For the contextual_say, the complement is used as a prompt for our LLM, and the characteristic specifies the context that should be fetched and passed to that LLM.
For find_person and find_person_info, the robot approaches the specified person, so you do not need to specify that extra action.

Do not assume that a complement or a characteristic can be omitted unless it is specified on the action by having it as "". It is better to have unnecessary extra information than missing needed information.

Another example, for the prompt "locate a standing person in the bedroom and follow them", your answer should be:

<think>
Okay, let me try to figure out how to approach this command. The user wants the robot to locate a standing person in the bedroom and then follow them.

First, the robot needs to go to the bedroom. So the first action is 'go' with complement 'bedroom'. Then, in the bedroom, the robot needs to find a person who is standing. The 'find_person' action is used here. The complement should be the posture, which is 'standing', and the characteristic would be 'posture' since that's the attribute we're searching by. Wait, looking back at the available actions, find_person has complement as the gesture, posture, cloth color, etc., and characteristic specifies "clothes", "gesture", "posture", etc. So maybe the complement is the value (like 'standing') and the characteristic is the type, which is 'posture'.

So the command would be action: find_person, complement: 'standing', characteristic: 'posture'. That makes sense because the robot is looking for a person with a specific posture.

Once the person is found, the robot needs to follow them. The 'follow_person_until' action is used here. The complement can be left as 'canceled' because the follow continues until stopped, and the characteristic would be empty unless we know the person's name. Since the user didn't mention a name, the characteristic is empty. Wait, no. The 'follow_person_until' action's complement can be a room, location, or 'canceled'. But here, we just want to follow the person regardless of where they go, until the command is canceled. So the complement would be 'canceled' to indicate that the following continues until instructed otherwise. The characteristic is the person's name, but since we don't have a name, it's left empty. So the action is follow_person_until with complement 'canceled' and characteristic empty.

Putting it all together: go to bedroom, find the person with posture standing, then follow them until canceled. The commands would be:

1. go to bedroom

2. find_person (complement: standing, characteristic: posture)

3. follow_person_until (complement: canceled, characteristic: "")
</think>
<answer> 
{"commands":[{"action":"go","complement":"bedroom","characteristic":""},{"action":"find_person","complement":"standing","characteristic":"posture"},{"action":"follow_person_until","complement":"canceled","characteristic":""}]}
</answer>

Another example, for the prompt "tell me how many dishes there are on the desk", your final answer should be this:

{"commands":[{"action":"go","complement":"desk","characteristic":""},{"action":"count","complement":"desk","characteristic":"dishes"},{"action":"go","complement":"start_location","characteristic":""},{"action":"contextual_say","complement":"tell me how many dishes there are on the desk","characteristic":"count"}]}
""",
        },
        {"role": "user", "content": full_text},
    ]


"""
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
"""

def get_answer_question_dialog(contexts, question):
    if contexts:
        context_text = "\n".join(contexts)
        user_content = f"{context_text}\n\n{question}"
    else:
        user_content = question

    return [
        {
            "role": "system",
            "content": (
                "You are a concise and knowledgeable assistant. "
                "Answer clearly and directly using only the provided information. "
                "Do not mention where the information came from. "
                "Avoid long explanations, speculation, or unnecessary details. "
                "Just provide the best possible answer."
            ),
        },
        {
            "role": "user",
            "content": user_content,
        },
    ]


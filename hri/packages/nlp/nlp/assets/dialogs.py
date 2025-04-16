from nlp.assets.schemas import ExtractedData, IsAnswerNegative, IsAnswerPositive


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
- disagreement or rejection (e.g., "no", "that's wrong")
- uncertainty or lack of understanding (e.g., "maybe", "I'm not sure", "I don't know")

A positive confirmation incluse:
- explicit affirmation (e.g. "yes", "correct", "sure", "absolutely")
- if the input confirms, agrees, or affirms something, it is **not** a negative confirmation.

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
                
**Input:** 'I don’t know'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'That's right'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'Maybe'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'Affirmative'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

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

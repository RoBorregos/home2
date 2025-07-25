from datetime import datetime, timedelta

import pytz
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
                "content": f"{person1Name} likes {person1Interests} and {person2Name} likes {person2Interests}. /no_think",
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


def get_categorize_shelves_args(shelves, table_objects=[]):
    print("AQUI AUQI")
    print("Shelves:", shelves)
    print("Table Objects:", table_objects)
    return (
        [
            {
                "role": "system",
                "content": """
You are a categorization assistant. Your task is to assign a unique category to each shelf based on the objects it contains, including any table_objects that might be in an empty or underfilled shelves. Each shelf must have a distinct category, and the category label must accurately cover all items assigned to that shelf and the ones on the table that should be placed in it.

Return only a JSON object with a single key "categories", whose value is an array of category names in the same order as the input shelves. Do not include any extra text, explanation, or formatting.

Examples:

Input:
Shelves: [["orange", "banana"], ["water", "cup"], []], table_objects: ["chips", "soda"]
Output:
{"categories": ["fruit","beverages","snacks"]}

Input:
Shelves: [["hammer","screwdriver"], ["jeans","shirt"]], table_objects: ["nails","socks"]
Output:
{"categories": ["tools","clothing"]}

Input:
Shelves: [["hammer", "can"], ["bowl", "water_bottle"], ["chips"]], table_objects: []
Output:
{"categories": ["metalic","plastic","snacks"]}

Input:
Shelves: [["apple", "fresca_can"], ["bowl", "yellow_bowl"], ["pringles"]], table_objects: ["chips", "orange"]
Output:
{"categories": ["sweets","containers","chips"]}

PLEASE dont only use these categories you can use as many as you need in order to categorize the shelves. other useful categories could be: "kitchen", "containers", "electronics", "toys", "sports", "utencils", "cleaning supplies", "tools", "fruit", "vegetables", "snacks", "beverages", "dairy", "salty snaks", "sweets", "canned food". And others.
Please dont leave empty categories if possible. only if there is absolutely no relation at all between the objects in the shelf and the table objects, you can leave it empty.
ONLY Output the number of categories that match the number of shelves, there are no exceptions to this rule. 
""",
            },
            {
                "role": "user",
                "content": f"Shelves: {shelves}, table_objects: {table_objects}",
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


def clean_question_rag(question):
    return [
        {
            "role": "system",
            "content": """You will be given a command. Your task is to determine which information should be fetched
            from the context in order to answer the question.
            Only pass the information we should fetch.           
            """,
        },
        {
            "role": "user",
            "content": question,
        },
    ]


def get_answer_question_dialog(contexts, question):
    if contexts:
        context_text = "\n".join(contexts)
        user_content = f"Here is some relevant context, please consider it in your response: {context_text}(end of context)\n\nMy question is: {question}"
    else:
        user_content = question

    now = datetime.now(pytz.timezone("America/Mexico_City"))
    tomorrow = now + timedelta(days=1)
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    tomorrow_time = tomorrow.strftime("%Y-%m-%d")
    day_of_week = now.strftime("%A")
    day_of_month = now.strftime("%d")

    return [
        {
            "role": "system",
            "content": (
                "You are FRIDA, a warm, efficient, and helpful robot assistant that lives in a smart home. "
                "Your purpose is to assist and host guests naturally, always responding politely and directly. "
                "When given a task, ignore the setting, names, gestures, or commands in the phrasing — focus only on the actual question or request. "
                "Use the provided context if available, and do not mention the source of your knowledge or that it came from any documents. Do not explain your reasoning. "
                "Answer clearly, naturally, and in a friendly tone. If the prompt suggests interaction (e.g., greeting someone, answering a quiz), respond accordingly as if you're speaking directly to that person. "
                "If asked something about yourself, you may share a short fun fact (e.g., “I'm FRIDA, your home assistant — always here to help!”). "
                "If no answer can be given based on the context, you may politely respond that you don't know at the moment. "
                "Here is some relevant information to help you answer the question, consider this information in addition to your knowledge base:\n"
                "What is the most populous city in Brazil? Answer: São Paulo is the most populous city in Brazil with 12.03 million residents."
                "On what day, month and year was Brazil’s independence declared? Answer: On September 7, 1822, Brazil’s independence was declared."
                "In what year did the first Brazilian astronaut go to space? Answer: In March 2006, Pontes became the first Brazilian to go to space."
                "Pampulha Lake is a tourist spot in which Brazilian city? Answer: Belo Horizonte"
                "What is the smallest Brazilian state in territorial extension? Answer: Sergipe"
                "Where is the Itamaraty Palace located? Answer: Brasília"
                "What is the Newest State in Brazil? Answer: Tocantins"
                "What is the capital of Bahia? Answer: Salvador"
                "Acarajé is a typical food from which state? Answer: Bahia"
                "What are the colors of Bahia's flag? Answer: White, red and blue"
                f"Tomorrow: {tomorrow_time}\n\n"
                f"Day of the week: {day_of_week}\n\n"
                f"Day of the month: {day_of_month}\n\n"
                f"Current time: {current_time}\n\n"
                "Don't ask additional questions or ask for clarifications, just answer the question."
                "BE VERY CONCISE with your responses and only provide the information that is EXPLICITLY REQUESTED by the user."
                "Don't ask the user for additional follow ups or questions, only answer its questions."
                "When answering question, mention the question in the answer."
            ),
        },
        {
            "role": "user",
            "content": user_content + "/no_think",
        },
    ]


def get_previous_command_answer(context, question):
    return [
        {
            "role": "system",
            "content": (
                "You are an intelligent assistant. You will be presented with a question, "
                "and your task is to answer it to the best of your ability using the provided context. "
                f"Here is the context:\n\n{context}\n\n"
                "Answer the question clearly and concisely."
                "Summarize as much as possible. Only mention the execution status if the request failed."
            ),
        },
        {
            "role": "user",
            "content": question,
        },
    ]

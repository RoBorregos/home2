from nlp.assets.schemas import ExtractedData


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

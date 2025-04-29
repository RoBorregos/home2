import sys

sys.path.append("..")

import json
import re
import time

from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_command_interpreter_args
from nlp.assets.schemas import CommandListShape
from openai import OpenAI
from openai._types import NOT_GIVEN
from tqdm import tqdm

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_embedding_similarity import JsonEmbeddingSimilarity


def format_json(data):
    try:
        return json.loads(data), True
    except (TypeError, ValueError):
        return json.loads(json.dumps(data)), False


def remove_extra_arguments(json_response):
    actions_no_complement = ["place", "ask_answer_question", "give"]
    actions_no_characteristic = ["go", "pick", "place", "say", "ask_answer_question", "give", "find_person_info", "find_person_by_name"]

    for command in json_response.get("commands", []):
        action = command.get("action", "")

        # Remove complement for specific actions
        if action in actions_no_complement:
            command["complement"] = ""

        # Remove characteristic for specific actions
        if action in actions_no_characteristic:
            command["characteristic"] = ""

    return json_response


def extract_answer(response):
    extracted = re.search("<think>(.+?)<\/think>\s*<answer>(.+?)<\/answer>", response, re.DOTALL)
    print(extracted)
    json_response, parsed = format_json(extracted.group(2))
    if extracted and parsed:
        json_response = remove_extra_arguments(json_response)
        return extracted.group(2), extracted.group(1)

    # Retry extraction by removing <think> tags
    extracted = re.search("<think>(.+?)<\/think>\s*(.+?)", response, re.DOTALL)
    print(extracted)

    json_response, parsed = format_json(extracted.group(2))
    if extracted and parsed:
        print(f"parsed by removing thinking {extracted.group(2)}")
        json_response = remove_extra_arguments(json_response)
        return extracted.group(2), extracted.group(1)

    # Retry by asking llm to format response
    structured = structured_response(response, CommandListShape)
    json_response, parsed = format_json(structured)
    if parsed:
        print(f"formatted by llm {structured}")
        json_response = remove_extra_arguments(json_response)
        return structured, None

    print("unable to parse")
    print(response)
    return response, None


# Sample function to test
def generate_response(full_text, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages = get_command_interpreter_args(full_text)

    # print(messages)
    start = time.time()
    response = client.beta.chat.completions.parse(model=MODEL, temperature=TEMPERATURE, messages=messages, response_format=NOT_GIVEN)
    end = time.time()
    print(f"Time waiting for response: {end - start}")

    content, thinking = extract_answer(response.choices[0].message.content)

    # if end - start > 60:
    #     print(f'Spent over one minute on generation, thinking: {thinking}')

    if two_steps:
        content = structured_response(content, CommandListShape)
        print("----------------------------------------------")
        print("Structured response:\n", content)
        return content, response.choices[0].message.content

    return content, response.choices[0].message.content


def structured_response(response, response_format):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    formatted_response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=format_response(response),
        response_format=response_format,
    )
    return formatted_response.choices[0].message.content


with open("data/command_interpreter.json") as f:
    command_dataset = json.load(f)

test_cases = [
    (
        command["string_cmd"],
        CommandListShape(
            commands=json.loads(command["structured_cmd"])["commands"],
        ).model_dump_json(),
    )
    for command in command_dataset
]


# Define test cases
test_cases = [LLMTestCase(input=test_case[0], expected_output=test_case[1], actual_output="") for test_case in test_cases]

THRESHOLD = 0.7
metric = JsonEmbeddingSimilarity(embeddings_keys=["complement", "characteristic"], threshold=THRESHOLD)
count = 0
for test_case in tqdm(test_cases, desc="Generating test case responses"):
    test_case.actual_output, full_response = generate_response(test_case.input, two_steps=False)
    a = metric.measure(test_case)
    if metric.measure(test_case) < THRESHOLD:
        count += 1
        print("\033[91mTest case failed:\033[0m")
        print("\033[94minput:\033[0m", test_case.input)
        print("\033[93mexpected_output:\033[0m", format_json(test_case.expected_output)[0])
        print("\033[93mactual_output:\033[0m", format_json(test_case.actual_output)[0])
        print("\033[93mfull_response:\033[0m", full_response)
        print("\n\n")

if count == 0:
    print(f"\n\033[92mAll test cases passed ({len(test_case)}/{len(test_case)})!\033[0m")
else:
    print(f"\n\033[91m{count} test cases failed out of {len(test_cases)}\033[0m")

import json
import os
import sys

sys.path.append("..")

import time

from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_categorize_shelves_args
from nlp.assets.schemas import CategorizeShelvesResult, Shelf
from openai import OpenAI
from openai._types import NOT_GIVEN

from frida_constants.hri_constants import MODEL
from config import API_KEY, BASE_URL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


def format_json(data):
    try:
        return json.loads(data)
    except (TypeError, ValueError):
        return json.loads(json.dumps(data))


def generate_response(shelves: dict[int, list[str]], table_objects: list[str], two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_categorize_shelves_args(shelves, table_objects)

    response = client.beta.chat.completions.parse(
        model=MODEL.GENERATE_RESPONSE.value,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=response_format if not two_steps else NOT_GIVEN,
    )

    if two_steps:
        return structured_response(response.choices[0].message.content, response_format)
    return response.choices[0].message.content


def structured_response(response, response_format):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    formatted_response = client.beta.chat.completions.parse(
        model=MODEL.STRUCTURED_RESPONSE.value,
        temperature=TEMPERATURE,
        messages=format_response(response),
        response_format=response_format,
    )
    return formatted_response.choices[0].message.content


# Load test cases from JSON file
test_cases_file = os.path.join(os.path.dirname(__file__), "data", "categorize_objects.json")
with open(test_cases_file, "r") as f:
    raw_test_cases = json.load(f)

# Define test cases

test_cases = []

for test_case in raw_test_cases:
    shelves = {}
    for k, value in test_case["expected_output"].items():
        shelves[int(k)] = Shelf(objects_to_add=value["objects_to_add"], classification_tag=value["classification_tag"])
    test_cases.append(LLMTestCase(input=(test_case["shelves"], test_case["table_objects"]), expected_output=CategorizeShelvesResult(shelves=shelves).model_dump_json(), actual_output=""))


metric = JsonInsensitiveValuesMatch(ignore_keys=["classification_tag"])
count = 0
print("\n\n")
for test_case in test_cases:
    start_time = time.time()
    test_case.actual_output = generate_response(test_case.input[0], test_case.input[1])
    end_time = time.time()
    duration = end_time - start_time
    if metric.measure(test_case) == 0:
        count += 1
        print("\033[91mTest case failed:\033[0m")
        print("\033[94minput:\033[0m", test_case.input)
        print("\033[93mexpected_output:\033[0m", format_json(test_case.expected_output))
        print("\033[93mactual_output:\033[0m", format_json(test_case.actual_output))
        print(f"\033[96mTime taken: {duration:.2f} seconds\033[0m")
        print("\n\n")
    else:
        # Optionally print success message with time
        print(f"\033[92mTest case passed in {duration:.2f} seconds\033[0m")
        # print("\033[94minput:\033[0m", test_case.input)
        # print("\033[93mexpected_output:\033[0m", format_json(test_case.expected_output))
        # print("\033[93mactual_output:\033[0m", format_json(test_case.actual_output))
        print("\n")


if count == 0:
    print(f"\n\033[92mAll test cases passed ({len(test_case)}/{len(test_case)})!\033[0m")
else:
    print(f"\n\033[91m{count} test cases failed out of {len(test_cases)}\033[0m")

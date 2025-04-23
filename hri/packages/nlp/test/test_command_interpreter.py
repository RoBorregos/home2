import sys

sys.path.append("..")

import json

# import pytest
# from deepeval import assert_test
# from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_command_interpreter_args
from nlp.assets.schemas import CommandListShape
from openai import OpenAI
from openai._types import NOT_GIVEN
from tqdm import tqdm

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


def format_json(data):
    try:
        return json.loads(data)
    except (TypeError, ValueError):
        return json.loads(json.dumps(data))


# Sample function to test
def generate_response(full_text, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages = get_command_interpreter_args(full_text)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=NOT_GIVEN if two_steps else CommandListShape,
    )

    content = response.choices[0].message.content

    if two_steps:
        content = structured_response(content, CommandListShape)
        return content

    return content


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
test_cases = [
    LLMTestCase(
        input=test_case[0],
        expected_output=test_case[1],
        actual_output=generate_response(test_case[0], two_steps=True),
    )
    for test_case in tqdm(test_cases, desc="Generating test case responses")
]

# dataset = EvaluationDataset(test_cases=test_cases)


# @pytest.mark.parametrize(
#     "test_case",
#     dataset,
# )
# def test_command_interpreter(test_case: LLMTestCase):
#     assert_test(test_case, [JsonInsensitiveValuesMatch()])


metric = JsonInsensitiveValuesMatch()
count = 0
print("\n\n")

for test_case in test_cases:
    if metric.measure(test_case) == 0:
        count += 1
        print("\033[91mTest case failed:\033[0m")
        print("\033[94minput:\033[0m", test_case.input)
        print("\033[93mexpected_output:\033[0m", format_json(test_case.expected_output))
        print("\033[93mactual_output:\033[0m", format_json(test_case.actual_output))
        print("\n\n")

if count == 0:
    print(f"\n\033[92mAll test cases passed ({len(test_case)}/{len(test_case)})!\033[0m")
else:
    print(f"\n\033[91m{count} test cases failed out of {len(test_cases)}\033[0m")

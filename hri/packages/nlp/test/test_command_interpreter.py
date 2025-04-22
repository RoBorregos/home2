import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_command_interpreter_args
from nlp.assets.schemas import CommandListShape
from openai import OpenAI
from openai._types import NOT_GIVEN
from tqdm import tqdm

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(full_text, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages = get_command_interpreter_args(full_text)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=CommandListShape if not two_steps else NOT_GIVEN,
    )

    content = response.choices[0].message.content

    if two_steps:
        content = structured_response(content, CommandListShape)
        print(content)
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


test_cases = [
    # Test possible receptionist dialogs
    (
        "navigate to the storage rack then find a cleaning supply and grasp it and bring it to the waving person in the office",
        CommandListShape(
            commands=[
                {"action": "go", "complement": "storage rack", "characteristic": ""},
                {"action": "find_object", "complement": "storage rack", "characteristic": "cleaning supply"},
                {"action": "pick", "complement": "cleaning supply", "characteristic": ""},
                {"action": "go", "complement": "office", "characteristic": ""},
                {"action": "find_person", "complement": "waving person", "characteristic": ""},
                {"action": "give", "complement": "", "characteristic": ""},
            ]
        ).model_dump_json(),
    ),
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


dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_command_interpreter(test_case: LLMTestCase):
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_room_identification_dialog
from nlp.assets.schemas import RoomIdentification
from openai import OpenAI

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(comment: str):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, _ = get_room_identification_dialog(comment)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages,
    )
    return response.choices[0].message.content


# Define test cases
test_cases = [
    ("Where is the sofa?", "Living room"),
    ("We eat dinner here.", "Dining room"),
    ("I cook meals here.", "Kitchen"),
    ("I sleep in this room.", "Bedroom"),
    ("This is where we enter the house.", "Entrance"),
]

test_cases = [
    LLMTestCase(
        input=test_case[0],
        expected_output=RoomIdentification(room=test_case[1]).model_dump_json(),
        actual_output=generate_response(test_case[0]),
    )
    for test_case in test_cases
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_room_identification(test_case: LLMTestCase):
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

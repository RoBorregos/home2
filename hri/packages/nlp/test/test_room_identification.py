import json
import shutil
import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_room_identification_dialog
from nlp.assets.schemas import RoomIdentification
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch
from openai import OpenAI
from config import API_KEY, BASE_URL, MODEL, TEMPERATURE


def generate_response(comment: str, areas_file="areas.json"):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    dialog = get_room_identification_dialog(comment, areas_file=areas_file)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=dialog["messages"],
    )
    return response.choices[0].message.content


# Create a copy of areas.json for consistent testing
shutil.copy("areas.json", "test_areas.json")

with open("test_areas.json", "r") as file:
    areas = json.load(file)

test_cases = []
for room, sub_areas in areas.items():
    test_cases.append(
        LLMTestCase(
            input=f"What is in the {room.replace('_', ' ')}?",
            expected_output=RoomIdentification(room=room.replace("_", " ").title()).model_dump_json(),
            actual_output=None,
        )
    )
    for sub_area in sub_areas.keys():
        test_cases.append(
            LLMTestCase(
                input=f"Where is the {sub_area.replace('_', ' ')}?",
                expected_output=RoomIdentification(room=f"{room.replace('_', ' ').title()} - {sub_area.replace('_', ' ').title()}").model_dump_json(),
                actual_output=None,
            )
        )

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize("test_case", dataset)
def test_room_identification(test_case: LLMTestCase):
    test_case.actual_output = generate_response(test_case.input, areas_file="test_areas.json")
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

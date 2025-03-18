import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_extract_data_args
from nlp.assets.schemas import ExtractedData
from openai import OpenAI
from openai._types import NOT_GIVEN

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(full_text, data_to_extract, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_extract_data_args(full_text, data_to_extract)

    response = client.beta.chat.completions.parse(
        model=MODEL,
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
        model=MODEL,
        temperature=TEMPERATURE,
        messages=format_response(response),
        response_format=response_format,
    )
    return formatted_response.choices[0].message.content


test_cases = [
    ("My name is Oscar and I like Fanta", "drink", ExtractedData(data="Fanta").model_dump_json()),
    ("My name is Oscar and I like Fanta", "name", ExtractedData(data="Oscar").model_dump_json()),
    ("", "drink", ExtractedData(data="").model_dump_json()),
    ("John enjoys eating pizza with his friends", "food", ExtractedData(data="pizza").model_dump_json()),
    ("John enjoys eating pizza with his friends", "name", ExtractedData(data="John").model_dump_json()),
    ("There is a dog in the backyard", "animal", ExtractedData(data="dog").model_dump_json()),
    ("She studies computer science at university", "major", ExtractedData(data="computer science").model_dump_json()),
    ("Maria's favorite fruit is mango", "fruit", ExtractedData(data="mango").model_dump_json()),
    ("The Eiffel Tower is located in Paris", "city", ExtractedData(data="Paris").model_dump_json()),
    ("We watched a great movie called Inception", "movie", ExtractedData(data="Inception").model_dump_json()),
    ("Carlos loves programming in Python", "language", ExtractedData(data="Python").model_dump_json()),
    ("The capital of France is Paris", "capital", ExtractedData(data="Paris").model_dump_json()),
    ("We traveled to Japan last summer", "country", ExtractedData(data="Japan").model_dump_json()),
    ("His favorite sport is basketball", "sport", ExtractedData(data="basketball").model_dump_json()),
    ("Sarah lives in New York City", "location", ExtractedData(data="New York City").model_dump_json()),
    ("My best friend’s name is Daniel", "friend", ExtractedData(data="Daniel").model_dump_json()),
    ("Lemonade", "drink", ExtractedData(data="Lemonade").model_dump_json()),
    ("Go to the kitchen", "place", ExtractedData(data="kitchen").model_dump_json()),
    ("Oscar", "name", ExtractedData(data="Oscar").model_dump_json()),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=test_case[0],
        expected_output=test_case[2],
        actual_output=generate_response(test_case[0], test_case[1]),
    )
    for test_case in test_cases
]


dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_data_extractor(test_case: LLMTestCase):
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

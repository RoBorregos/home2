import sys

sys.path.append("..")
sys.path.append("../../../../frida_constants")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import format_response, get_extract_data_args
from nlp.assets.schemas import ExtractedData
from openai import OpenAI
from openai._types import NOT_GIVEN
from frida_constants.hri_constants import MODEL
from config import API_KEY, BASE_URL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(full_text, data_to_extract, context, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_extract_data_args(full_text, data_to_extract, context)

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


RAW_TEST_CASES = [
    # Test possible receptionist dialogs
    ("My name is Oscar and I like Fanta", "drink", None, "Fanta"),
    ("My name is Oscar and I like Fanta", "name", None, "Oscar"),
    ("I like fanta but my favorite drink is Coke", "drink", None, "Coke"),
    ("My favorite drink is fanta but I also drink Coke", "drink", None, "fanta"),
    # Test with context
    ("I hate Lemonade. However, I like Fanta", "drink", "The user was asked 'which drink do you hate'. We want to infer his hated drink from response", "Lemonade"),
    ("I hate Lemonade. However, I like Fanta", "drink", "The user was asked 'what is your favorite drink'. We want to infer his favorite drink from response", "Fanta"),
    # Test extracting different information from the same sentence
    ("John enjoys eating pizza with his friends", "food", None, "pizza"),
    ("John enjoys eating pizza with his friends", "name", None, "John"),
    # Random Cases
    ("There is a dog in the backyard", "animal", None, "dog"),
    ("She studies computer science at university", "major", None, "computer science"),
    ("Maria's favorite fruit is mango", "fruit", None, "mango"),
    ("The Eiffel Tower is located in Paris", "city", None, "Paris"),
    ("We watched a great movie called Inception", "movie", None, "Inception"),
    ("Carlos loves programming in Python", "language", None, "Python"),
    ("The capital of France is Paris", "capital", None, "Paris"),
    ("We traveled to Japan last summer", "country", None, "Japan"),
    ("His favorite sport is basketball", "sport", None, "basketball"),
    ("Sarah lives in New York City", "location", None, "New York City"),
    ("My best friend's name is Daniel", "friend", None, "Daniel"),
    ("Go to the kitchen", "place", None, "kitchen"),
    # Test with few words
    ("Lemonade", "drink", None, "Lemonade"),
    ("Oscar", "name", None, "Oscar"),
    # Test empty
    ("", "drink", None, ""),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=text,
        expected_output=ExtractedData(data=expected).model_dump_json(),
        actual_output=None,
        additional_metadata={
            "data_to_extract": data_to_extract,
            "context": context,
        },
    )
    for text, data_to_extract, context, expected in RAW_TEST_CASES
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_data_extractor(test_case: LLMTestCase):
    meta = test_case.additional_metadata

    test_case.actual_output = generate_response(
        test_case.input,
        meta["data_to_extract"],
        meta["context"],
    )

    assert_test(test_case, [JsonInsensitiveValuesMatch()])

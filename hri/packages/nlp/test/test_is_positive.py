import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_is_answer_positive_args
from nlp.assets.schemas import IsAnswerPositive
from openai import OpenAI

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(interpreted_text: str):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_is_answer_positive_args(interpreted_text)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=response_format,
    )
    return response.choices[0].message.content


test_cases = [
    # Positive test cases
    ("Yes", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("That's right", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Correct", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Absolutely", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Of course", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Sure", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Exactly", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("I agree", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Indeed", IsAnswerPositive(is_positive=True).model_dump_json()),
    ("Affirmative", IsAnswerPositive(is_positive=True).model_dump_json()),
    # Negative test cases
    ("No", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("That's wrong", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("Incorrect", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("I don’t think so", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("Not at all", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("I don't know", IsAnswerPositive(is_positive=False).model_dump_json()),
    # Uncertain test cases
    ("Maybe", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("I'm not sure", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("Huh?", IsAnswerPositive(is_positive=False).model_dump_json()),
    ("What do you mean?", IsAnswerPositive(is_positive=False).model_dump_json()),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=test_case[0],
        expected_output=test_case[1],
        actual_output=generate_response(test_case[0]),
    )
    for test_case in test_cases
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_is_positive(test_case: LLMTestCase):
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

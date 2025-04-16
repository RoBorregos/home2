import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_is_answer_negative_args
from nlp.assets.schemas import IsAnswerNegative
from openai import OpenAI

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(interpreted_text: str):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_is_answer_negative_args(interpreted_text)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=response_format,
    )
    return response.choices[0].message.content


test_cases = [
    # Negative test cases
    ("No", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("That's wrong", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("Incorrect", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("Not at all", IsAnswerNegative(is_negative=True).model_dump_json()),
    # Positive test cases
    ("Yes", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("That's right", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Correct", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Absolutely", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Of course", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Sure", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Exactly", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("I agree", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Indeed", IsAnswerNegative(is_negative=False).model_dump_json()),
    ("Affirmative", IsAnswerNegative(is_negative=False).model_dump_json()),
    # Uncertain test cases should also be negative
    ("I don’t think so", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("I don't know", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("Maybe", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("I'm not sure", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("Huh?", IsAnswerNegative(is_negative=True).model_dump_json()),
    ("What do you mean?", IsAnswerNegative(is_negative=True).model_dump_json()),
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
def test_is_negative(test_case: LLMTestCase):
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

import sys

sys.path.append("..")
sys.path.append("../../../../frida_constants")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_is_answer_negative_args
from nlp.assets.schemas import IsAnswerNegative
from openai import OpenAI
from frida_constants.hri_constants import MODEL
from config import API_KEY, BASE_URL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(interpreted_text: str):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_is_answer_negative_args(interpreted_text)

    response = client.beta.chat.completions.parse(
        model=MODEL.GENERATE_RESPONSE.value,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=response_format,
    )
    return response.choices[0].message.content


RAW_TEST_CASES = [
    # Negative test cases
    ("No", True),
    ("That's wrong", True),
    ("Incorrect", True),
    ("Not at all", True),
    ("I don't think so", True),
    # Positive test cases
    ("Yes", False),
    ("That's right", False),
    ("Correct", False),
    ("Absolutely", False),
    ("Of course", False),
    ("Sure", False),
    ("Exactly", False),
    ("I agree", False),
    ("Indeed", False),
    ("Affirmative", False),
    # Uncertain test cases
    ("I don't know", False),
    ("Maybe", False),
    ("I'm not sure", False),
    ("Huh?", False),
    ("What do you mean?", False),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=text,
        expected_output=IsAnswerNegative(is_negative=expected).model_dump_json(),
        actual_output=None,
    )
    for text, expected in RAW_TEST_CASES
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_is_negative(test_case: LLMTestCase):
    test_case.actual_output = generate_response(test_case.input)
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

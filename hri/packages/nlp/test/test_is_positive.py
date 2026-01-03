import sys

sys.path.append("..")
sys.path.append("../../../../frida_constants")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_is_answer_positive_args
from nlp.assets.schemas import IsAnswerPositive
from openai import OpenAI
from frida_constants.hri_constants import MODEL
from config import API_KEY, BASE_URL, TEMPERATURE
from metrics.json_insensitive_values_match import JsonInsensitiveValuesMatch


# Sample function to test
def generate_response(interpreted_text: str):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages, response_format = get_is_answer_positive_args(interpreted_text)

    response = client.beta.chat.completions.parse(
        model=MODEL.GENERATE_RESPONSE.value,
        temperature=TEMPERATURE,
        messages=messages,
        response_format=response_format,
    )
    return response.choices[0].message.content


RAW_TEST_CASES = [
    # Positive test cases
    ("Yes", True),
    ("That's right", True),
    ("Correct", True),
    ("Absolutely", True),
    ("Of course", True),
    ("Sure", True),
    ("Exactly", True),
    ("I agree", True),
    ("Indeed", True),
    ("Affirmative", True),
    # Negative test cases
    ("No", False),
    ("That's wrong", False),
    ("Incorrect", False),
    ("I don’t think so", False),
    ("Not at all", False),
    ("I don't know", False),
    # Uncertain test cases
    ("Maybe", False),
    ("I'm not sure", False),
    ("Huh?", False),
    ("What do you mean?", False),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=text,
        expected_output=IsAnswerPositive(is_positive=expected).model_dump_json(),
        actual_output=None,
    )
    for text, expected in RAW_TEST_CASES
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_is_positive(test_case: LLMTestCase):
    test_case.actual_output = generate_response(test_case.input)
    assert_test(test_case, [JsonInsensitiveValuesMatch()])

import sys

sys.path.append("..")
sys.path.append("../../../../frida_constants")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_common_interests_dialog
from openai import OpenAI
from frida_constants.hri_constants import MODEL
from config import API_KEY, BASE_URL, TEMPERATURE
from metrics.embeddings_similarity import EmbeddingSimilarity


def generate_response(person1, person2, interests1, interests2):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)

    messages = get_common_interests_dialog(person1, person2, interests1, interests2)

    response = client.beta.chat.completions.parse(
        model=MODEL.GENERATE_RESPONSE.value,
        temperature=TEMPERATURE,
        messages=messages["messages"],
    )

    return response.choices[0].message.content


RAW_TEST_CASES = [
    # Test possible receptionist dialogs
    ("Oscar", "John", "movies and tv series", "tv series and videogames", "tv series"),
    ("Alice", "Bob", "reading and hiking", "hiking and swimming", "Alice and Bob's common interest is hiking."),
    ("Emma", "Liam", "photography and travel", "cooking and gardening", "Emma and Liam don't have a common interest between them"),
    ("Sophia", "Noah", "music and dancing", "music and painting", "Sophia and Noah's common interest is music."),
    ("Ava", "William", "chess and puzzles", "puzzles and coding", "Ava and William's common interest is puzzles."),
    ("Mia", "James", "skiing and snowboarding", "snowboarding and surfing", "Mia and James's common interest is snowboarding."),
    ("Charlotte", "Benjamin", "yoga and meditation", "running and cycling", "Charlotte and Benjamin don't have a common interest between them"),
    ("Amelia", "Lucas", "anime and comics", "comics and videogames", "Amelia and Lucas's common interest is comics."),
    ("Harper", "Henry", "cooking and baking", "baking and photography", "Harper and Henry's common interest is baking."),
    ("Evelyn", "Alexander", "painting and drawing", "sculpting and pottery", "Evelyn and Alexander don't have a common interest between them"),
    ("Abigail", "Michael", "fishing and camping", "camping and hiking", "Abigail and Michael's common interest is camping."),
    ("Emily", "Daniel", "tennis and badminton", "badminton and squash", "Emily and Daniel's common interest is badminton."),
    ("Elizabeth", "Matthew", "writing and poetry", "poetry and reading", "Elizabeth and Matthew's common interest is poetry."),
    ("Sofia", "Jackson", "gaming and streaming", "painting and drawing", "Sofia and Jackson don't have a common interest between them"),
    ("Avery", "David", "knitting and sewing", "sewing and crochet", "Avery and David's common interest is sewing."),
    ("Ella", "Joseph", "cycling and running", "swimming and yoga", "Ella and Joseph don't have a common interest between them"),
    ("Scarlett", "Samuel", "astronomy and physics", "physics and mathematics", "Scarlett and Samuel's common interest is physics."),
    ("John", "Mike", "Sports", "Basketball", "John and Mike's common interest is sports."),
    ("Grace", "John", "singing and dancing", "dancing and acting", "Grace and John's common interest is dancing."),
    ("Chloe", "Luke", "gardening and plants", "cooking and baking", "Chloe and Luke don't have a common interest between them"),
    ("Victoria", "Andrew", "movies and theater", "theater and concerts", "Victoria and Andrew's common interest is theater."),
    ("Riley", "Jack", "football and basketball", "basketball and baseball", "Riley and Jack's common interest is basketball."),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=f"{p1} | {p2} | {i1} | {i2}",
        expected_output=expected,
        actual_output=None,
        additional_metadata={
            "person1": p1,
            "person2": p2,
            "interests1": i1,
            "interests2": i2,
        },
    )
    for p1, p2, i1, i2, expected in RAW_TEST_CASES
]

dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize("test_case", dataset)
def test_common_interests(test_case: LLMTestCase):
    meta = test_case.additional_metadata

    test_case.actual_output = generate_response(
        meta["person1"],
        meta["person2"],
        meta["interests1"],
        meta["interests2"],
    )

    assert_test(test_case, [EmbeddingSimilarity(threshold=0.5)])

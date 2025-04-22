import sys

sys.path.append("..")

import pytest
from deepeval import assert_test
from deepeval.dataset import EvaluationDataset
from deepeval.test_case import LLMTestCase
from nlp.assets.dialogs import get_common_interests_dialog, format_response
from openai import OpenAI
from tqdm import tqdm

from config import API_KEY, BASE_URL, MODEL, TEMPERATURE
from metrics.embeddings_similarity import EmbeddingSimilarity


# Sample function to test
def generate_response(person1Name, person2Name, person1Interests, person2Interests, two_steps=False):
    client = OpenAI(api_key=API_KEY, base_url=BASE_URL)
    messages = get_common_interests_dialog(person1Name, person2Name, person1Interests, person2Interests)

    response = client.beta.chat.completions.parse(
        model=MODEL,
        temperature=TEMPERATURE,
        messages=messages["messages"],
    )

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
    ("Grace", "John", "singing and dancing", "dancing and acting", "Grace and John's common interest is dancing."),
    ("Chloe", "Luke", "gardening and plants", "cooking and baking", "Chloe and Luke don't have a common interest between them"),
    ("Victoria", "Andrew", "movies and theater", "theater and concerts", "Victoria and Andrew's common interest is theater."),
    ("Riley", "Jack", "football and basketball", "basketball and baseball", "Riley and Jack's common interest is basketball."),
]

# Define test cases
test_cases = [
    LLMTestCase(
        input=test_case[0] + test_case[1],
        expected_output=test_case[4],
        actual_output=generate_response(test_case[0], test_case[1], test_case[2], test_case[3]),
    )
    for test_case in tqdm(test_cases, desc="Generating test case responses")
]


dataset = EvaluationDataset(test_cases=test_cases)


@pytest.mark.parametrize(
    "test_case",
    dataset,
)
def test_data_extractor(test_case: LLMTestCase):
    if test_case.additional_metadata:
        threshold = test_case.additional_metadata.get("threshold", 0.5)
    else:
        threshold = 0.5
    assert_test(test_case, [EmbeddingSimilarity(threshold=threshold)])

from random import randint

from utils.config import SubtaskConfig
from utils.status import Status


def mock_extract_data(query, complete_text, context):
    """Pick a random word from complete_text"""

    split_text = complete_text.split()

    return Status.MOCKED, split_text[randint(0, len(split_text) - 1)]


def mock_interpret_keyword(keywords: list[str], timeout: float):
    """Pick a random word from keywords"""

    return Status.MOCKED, keywords[randint(0, len(keywords) - 1)]


def mock_common_interest(person1, interest1, person2, interest2, remove_thinking=True):
    """Pick a random word from keywords"""

    return Status.MOCKED, f"{person1} and {person2} both like {interest1}"


__config = {
    "topic_config": [],
    "mock_config": [
        {
            "function_name": "extract_data",
            "enabled": True,
            "mock_data": mock_extract_data,
        },
        {
            "function_name": "say",
            "enabled": True,
            "mock_data": Status.MOCKED,
        },
        {
            "function_name": "confirm",
            "enabled": True,
            "mock_data": (Status.MOCKED, "yes"),
        },
        {
            "function_name": "ask_and_confirm",
            "enabled": True,
            "mock_data": (Status.MOCKED, "yes"),
        },
        {
            "function_name": "interpret_keyword",
            "enabled": True,
            "mock_data": mock_interpret_keyword,
        },
        {
            "function_name": "common_interest",
            "enabled": True,
            "mock_data": mock_common_interest,
        },
        {
            "function_name": "hear",
            "enabled": True,
            "mock_data": (Status.MOCKED, "Hi Frida, can you bring me a glass of water?"),
        },
        {
            "function_name": "setup_services",
            "enabled": True,
            "mock_data": (Status.MOCKED),
        },
    ],
    "strict": True,
}

config = SubtaskConfig(**__config)

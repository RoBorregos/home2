from random import randint

from utils.config import SubtaskConfig

from frida_constants.hri_constants import COMMAND_INTERPRETER_SERVICE, STT_SERVICE_NAME


def mock_extract_data(query, complete_text):
    """Pick a random word from complete_text"""

    split_text = complete_text.split()

    return split_text[randint(0, len(split_text) - 1)]


__config = {
    "topic_config": [
        {
            "topic_name": COMMAND_INTERPRETER_SERVICE,
            "enabled": False,
            "type": "service",
        },
        {"topic_name": STT_SERVICE_NAME, "enabled": False, "type": "service"},
    ],
    "mock_config": [
        {
            "function_name": "extract_data",
            "enabled": False,
            "mock_data": mock_extract_data,
        },
        {
            "function_name": "say",
            "enabled": False,
            "mock_data": "Succeeded!",
        },
        {
            "function_name": "hear",
            "enabled": False,
            "mock_data": "Hi Frida, can you bring me a glass of water?",
        },
    ],
    "strict": True,
}

config = SubtaskConfig(**__config)

import json

from deepeval.metrics import BaseMetric
from deepeval.scorer import Scorer
from deepeval.test_case import LLMTestCase


class JsonInsensitiveValuesMatch(BaseMetric):
    def __init__(
        self,
        threshold: float = 0.5,
    ):
        self.threshold = threshold
        self.scorer = Scorer()

    def measure(self, test_case: LLMTestCase) -> float:
        try:
            js1 = remove_key(json.loads(test_case.expected_output), "rationale")
            js2 = remove_key(json.loads(test_case.actual_output), "rationale")

            self.score = 1

            if js1.keys() != js2.keys():
                self.score = 0
            else:
                for key in js1.keys():
                    if format(js1[key]) != format(js2[key]):
                        self.score = 0
                        break

            self.success = self.score > self.threshold
            return self.score
        except Exception as e:
            self.error = str(e)
            raise

    async def a_measure(self, test_case: LLMTestCase):
        return self.measure(test_case)

    def is_successful(self):
        if self.error is not None:
            self.success = False

        return self.success

    @property
    def __name__(self):
        return "Json case-insensitive exact match"


def remove_key(object, erase_key):
    """
    Remove keys from dicts recursively.
    """
    if isinstance(object, dict):
        return {key: remove_key(value, erase_key) for key, value in object.items() if key != erase_key}
    elif isinstance(object, list):
        return [remove_key(value, erase_key) for value in object]
    else:
        return object


def format(object):
    if isinstance(object, dict):
        return {format_string(key): format(value) for key, value in object.items()}
    elif isinstance(object, list):
        return [format(value) for value in object]
    elif isinstance(object, str):
        return format_string(object)
    else:
        return object


def format_string(string: str) -> str:
    return string.lower().strip()

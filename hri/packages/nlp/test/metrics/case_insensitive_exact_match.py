from deepeval.metrics import BaseMetric
from deepeval.scorer import Scorer
from deepeval.test_case import LLMTestCase


class CaseInsensitiveExactMatch(BaseMetric):
    def __init__(
        self,
        threshold: float = 0.5,
    ):
        self.threshold = threshold
        self.scorer = Scorer()

    def measure(self, test_case: LLMTestCase) -> float:
        self.score = format_string(test_case.actual_output) == format_string(test_case.expected_output)
        self.success = self.score > self.threshold
        return self.score

    async def a_measure(self, test_case: LLMTestCase):
        return self.measure(test_case)

    def is_successful(self):
        return self.success

    @property
    def __name__(self):
        return "Case-insensitive exact match"


def format_string(string: str) -> str:
    return string.lower().strip()

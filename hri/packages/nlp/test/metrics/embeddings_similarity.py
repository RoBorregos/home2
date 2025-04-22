from deepeval.metrics import BaseMetric
from deepeval.metrics.utils import initialize_embedding_model
from deepeval.test_case import LLMTestCase
from deepeval.utils import cosine_similarity


class EmbeddingSimilarity(BaseMetric):
    def __init__(
        self,
        threshold: float = 0.5,
    ):
        self.threshold = threshold
        self.embedding_model = initialize_embedding_model()

    def measure(self, test_case: LLMTestCase) -> float:
        # Use threshold from test case metadata if available, otherwise use default
        threshold = test_case.additional_metadata.get("threshold", self.threshold)
        emb = self.embedding_model.embed_text
        self.score = cosine_similarity(emb(test_case.actual_output), emb(test_case.expected_output))
        self.success = self.score > threshold
        return self.score

    async def a_measure(self, test_case: LLMTestCase):
        return self.measure(test_case)

    def is_successful(self):
        return self.success

    @property
    def __name__(self):
        return "Embedding similarity"


def format_string(string: str) -> str:
    return string.lower().strip()

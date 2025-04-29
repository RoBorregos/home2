import json

from deepeval.metrics import BaseMetric
from deepeval.metrics.utils import initialize_embedding_model
from deepeval.test_case import LLMTestCase
from deepeval.utils import cosine_similarity


class JsonEmbeddingSimilarity(BaseMetric):
    def __init__(
        self,
        threshold: float = 0.5,
        ignore_keys: list[str] = None,
        embeddings_keys: list[str] = None,
    ):
        self.threshold = threshold
        self.ignore_keys = ignore_keys if ignore_keys else []
        self.embeddings_keys = embeddings_keys if embeddings_keys else []
        self.embedding_model = initialize_embedding_model()

    def measure(self, test_case: LLMTestCase) -> float:
        try:
            js1 = remove_keys(json.loads(test_case.expected_output), self.ignore_keys + ["rationale"])
            js2 = remove_keys(json.loads(test_case.actual_output), self.ignore_keys + ["rationale"])

            self.score = self.compare_objects_recursively(js1, js2, self.embedding_model)
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

    def compare_objects_recursively(self, obj1, obj2, embedding_model):
        """
        Recursively compares two objects and calculates cosine similarity between their values.
        Returns a score between 0 and 1, where 1 means perfect similarity.
        """
        if isinstance(obj1, (str, int, float, bool)) and isinstance(obj2, (str, int, float, bool)):
            if isinstance(obj1, str) and isinstance(obj2, str):
                if not obj1.strip() and not obj2.strip():
                    return 1.0
                emb1 = embedding_model.embed_text(str(obj1))
                emb2 = embedding_model.embed_text(str(obj2))
                return cosine_similarity(emb1, emb2)
            return 1.0 if obj1 == obj2 else 0.0

        elif isinstance(obj1, list) and isinstance(obj2, list):
            if len(obj1) != len(obj2):
                return 0.0

            if not obj1:
                return 1.0

            similarities = []
            for i in range(len(obj1)):
                if i < len(obj2):
                    similarities.append(self.compare_objects_recursively(obj1[i], obj2[i], embedding_model))

            return min(similarities)

        elif isinstance(obj1, dict) and isinstance(obj2, dict):
            if set(obj1.keys()) != set(obj2.keys()):
                return 0.0

            if not obj1:
                return 1.0

            similarities = []
            for key in obj1:
                if key in self.embeddings_keys:
                    if obj1[key] == obj2[key]:
                        similarities.append(1.0)
                        continue
                    elif not obj1[key] or not obj2[key]:
                        similarities.append(0)
                        continue
                    emb1 = embedding_model.embed_text(str(obj1[key]))
                    emb2 = embedding_model.embed_text(str(obj2[key]))
                    similarities.append(cosine_similarity(emb1, emb2))
                else:
                    similarities.append(self.compare_objects_recursively(obj1[key], obj2[key], embedding_model))

            return min(similarities) if similarities else 1.0

        else:
            return 0.0


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


def remove_keys(object, erase_keys):
    """
    Remove keys from dicts recursively.
    """

    for key in erase_keys:
        object = remove_key(object, key)
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

from typing import List

from spacy.tokens import Span

# spacy labels
# https://stackoverflow.com/a/78475321

NAME_PRIORITY_LABELS = [
    "PERSON",
    "ORG",
    "NORP",
    "WORK_OF_ART",
    "LAW",
    "LANGUAGE",
    "EVENT",
    "PRODUCT",
    "GPE",
    "LOC",
    "FAC",
    "DATE",
    "TIME",
    "ORDINAL",
    "CARDINAL",
    "PERCENT",
    "MONEY",
    "QUANTITY",
]

LOC_PRIORITY_LABELS = [
    "GPE",
    "LOC",
    "FAC",
    "ORG",
    "EVENT",
    "NORP",
    "PERSON",
    "PRODUCT",
    "WORK_OF_ART",
    "LAW",
    "LANGUAGE",
    "DATE",
    "TIME",
    "ORDINAL",
    "CARDINAL",
    "PERCENT",
    "MONEY",
    "QUANTITY",
]


def extract_by_priority(
    ents: List[Span],
    priority_labels: List[str],
) -> str:
    """
    Extracts information from entities based on priority labels.
    If multiple labels are present, the first one in the priority list is used.
    All entities with the same chosen label will be concatenated and returned.

    Args:
        ents (List[Span]): List of spaCy named entity spans.
        priority_labels (List[str]): Labels to prioritize, in order.

    Returns:
        str: Concatenated entity texts for the highest priority label found.
    """
    label_to_ents = {}
    for ent in ents:
        if ent.label_ not in label_to_ents:
            label_to_ents[ent.label_] = []
        label_to_ents[ent.label_].append(ent.text)

    # Pick the first priority label that exists in the entities
    for label in priority_labels:
        if label in label_to_ents:
            return " ".join(label_to_ents[label])

    return ""

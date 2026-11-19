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
    Repeated entities and entities contained in a longer one ("Ethan" in "Ethan Hunt") collapse
    into one; if several different entities remain ("Adel and Jane") the answer is ambiguous.

    Args:
        ents (List[Span]): List of spaCy named entity spans.
        priority_labels (List[str]): Labels to prioritize, in order.

    Returns:
        str: The entity text for the highest priority label found, "" if none or ambiguous.
    """
    label_to_ents = {}
    for ent in ents:
        if ent.label_ not in label_to_ents:
            label_to_ents[ent.label_] = []
        label_to_ents[ent.label_].append(ent.text)

    # Pick the first priority label that exists in the entities
    for label in priority_labels:
        if label in label_to_ents:
            texts = set(label_to_ents[label])
            unique = [
                t
                for t in texts
                if not any(t != o and t.lower() in o.lower() for o in texts)
            ]
            return unique[0] if len(unique) == 1 else ""

    return ""

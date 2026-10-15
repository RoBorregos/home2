"""Phrase-aware hotword preparation for the local Whisper decoder."""

import re
import unicodedata
import warnings


def parse_hotwords(hotwords):
    """Split comma/semicolon/newline lists; preserve phrases and first spelling."""
    phrases = []
    seen = set()
    for value in re.split(r"[,;\n\r]+", hotwords or ""):
        phrase = " ".join(unicodedata.normalize("NFC", value).split())
        key = phrase.casefold()
        if phrase and key not in seen:
            phrases.append(phrase)
            seen.add(key)
    return phrases


def normalize_hotwords(hotwords):
    return ", ".join(parse_hotwords(hotwords))


def build_context_tokens(tokenizer, previous_tokens, hotwords, context_budget):
    """Fit whole phrases, reserving up to half the budget for recent history."""
    if context_budget <= 0:
        return []
    history_reserve = min(len(previous_tokens), context_budget // 2)
    hotword_budget = context_budget - history_reserve
    selected = []
    hotword_tokens = []
    omitted = False
    for phrase in parse_hotwords(hotwords):
        # Encode the complete candidate so punctuation/BPE boundaries count.
        candidate = tokenizer.encode(" " + ", ".join([*selected, phrase]))
        if len(candidate) <= hotword_budget:
            selected.append(phrase)
            hotword_tokens = candidate
        else:
            omitted = True
    if omitted:
        warnings.warn(
            "Hotword phrases exceeded the Whisper context budget and were omitted. "
            "Use a shorter, task-specific list with comma-separated phrases, "
            "ordered by priority.",
            RuntimeWarning,
        )
    remaining = context_budget - len(hotword_tokens)
    # [-0:] returns the entire list, so handle an exhausted budget explicitly.
    return hotword_tokens + (list(previous_tokens[-remaining:]) if remaining else [])

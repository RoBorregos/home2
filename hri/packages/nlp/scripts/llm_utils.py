#!/usr/bin/env python3

"""Miscellanous functions that interact with an LLM."""

import json
import os
from typing import Optional

from nlp.assets.dialogs import (
    format_response,
    get_categorize_shelves_args,
    get_common_interests_dialog,
    get_previous_command_answer,
)
from transformers import AutoModelForSequenceClassification, AutoTokenizer, pipeline

from frida_constants.hri_constants import MODEL
from frida_interfaces.srv import IsPositive

CURRENT_FILE_PATH = os.path.abspath(__file__)

try:
    FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("install")]
except ValueError:
    # If the file is not in the install directory, use the src directory
    FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("src")]

ASSETS_DIR = os.path.join(
    FILE_DIR, "src", "hri", "packages", "nlp", "assets", "is_positive_negative"
)


IS_POSITIVE_MODEL_NAME = "tasksource/deberta-small-long-nli"


class LLMUtils:
    base_url: Optional[str]

    def __init__(self) -> None:
        if not os.path.exists(os.path.join(ASSETS_DIR, IS_POSITIVE_MODEL_NAME)):
            print(
                f"Downloading {IS_POSITIVE_MODEL_NAME} to a local directory. This may take a while."
            )

            self.classifier = pipeline(
                "zero-shot-classification", model=IS_POSITIVE_MODEL_NAME
            )
            self.classifier.model.save_pretrained(ASSETS_DIR)
            self.classifier.tokenizer.save_pretrained(ASSETS_DIR)
        else:
            print(f"Loading {IS_POSITIVE_MODEL_NAME} from local directory...")

            tokenizer = AutoTokenizer.from_pretrained(ASSETS_DIR)
            model = AutoModelForSequenceClassification.from_pretrained(ASSETS_DIR)
            self.classifier = pipeline(
                "zero-shot-classification", model=model, tokenizer=tokenizer
            )
        self.candidate_labels = ["yes", "no", "i don't know"]

    def llm_wrapper(self, context, question):
        messages = get_previous_command_answer(context, question)

        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.LLM_WRAPPER.value,
                temperature=self.temperature,
                messages=messages,
            )
            .choices[0]
            .message.content
        )

        if "</think>" in response:
            response = response.split("</think>")[-1].strip()

        return response

    def common_interest(self, person1, person2, interests1, interests2):
        print("Generating common interest")

        messages = get_common_interests_dialog(
            person1, person2, interests1, interests2
        )["messages"]
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.CommonInterest.value,
                temperature=self.temperature,
                messages=messages,
            )
            .choices[0]
            .message.content
        )

        return response

    def generic_structured_output(self, messages, response_format):
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.GENERIC_STRUCTURED_OUTPUT.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )
        try:
            response_data = json.loads(response)
            result = response_format(**response_data)
        except Exception as e:
            print(f"Structured output error: {e}")
            return None
        return result

    def categorize_shelves(self, shelves):
        """Service to categorize shelves."""

        messages, response_format = get_categorize_shelves_args(shelves)

        response_content = (
            self.client.beta.chat.completions.parse(
                model=MODEL.CATEGORIZE_SHELVES.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )

        if "</think>" in response_content:
            response_content = response_content.split("</think>")[-1].strip()

        try:
            categorized_shelves = json.loads(response_content)
            formatted = [str(c) for c in categorized_shelves["categories"]]

            return formatted
        except Exception as e:
            print(f"Error parsing JSON: {e}")
            categorized_shelves = self.generic_structured_output(
                format_response(response_content), response_format
            )
            return categorized_shelves

    def is_positive(self, text) -> IsPositive.Response:
        """Service to extract information from text."""
        print("Determining if text is positive")
        result = self.get_most_likely_label(text)

        is_positive = result == "yes"
        print(f"The text is positive: {is_positive}")

        return is_positive

    def is_negative(self, text):
        """Service to extract information from text."""
        print("Determining if text is negative")
        result = self.get_most_likely_label(text)

        is_negative = result == "no"
        print(f"The text is negative: {is_negative}")
        return is_negative

    def get_most_likely_label(self, text):
        """Get the most likely label for a given text."""
        result = self.classifier(text, self.candidate_labels)

        print(f"Classification result: {str(result)}")

        scores = result["scores"]
        labels = result["labels"]

        # Get the index of the maximum score
        max_index = scores.index(max(scores))
        return labels[max_index]


def main(args=None):
    """Main function for testing purposes."""
    llm_utils = LLMUtils()

    # Example usage
    text = "I love programming!"
    is_positive_response = llm_utils.is_positive(text)
    print(f"Is the text positive? {is_positive_response}")

    is_negative_response = llm_utils.is_negative(text)
    print(f"Is the text negative? {is_negative_response}")

    # Add more tests as needed


if __name__ == "__main__":
    main()

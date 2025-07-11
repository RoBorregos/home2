#!/usr/bin/env python3

"""
Python ROS2 node to extract information from text
"""

import json
import os

# Libraries
from typing import Optional

import rclpy
import spacy
from nlp.assets.data_extraction_priority import (
    LOC_PRIORITY_LABELS,
    NAME_PRIORITY_LABELS,
    extract_by_priority,
)
from nlp.assets.dialogs import get_extract_data_args
from openai import OpenAI
from pydantic import BaseModel
from rclpy.node import Node

from frida_constants.hri_constants import MODEL
from frida_interfaces.srv import ExtractInfo

EXTRACT_DATA_SERVICE = "/extract_data"

CURRENT_FILE_PATH = os.path.abspath(__file__)

FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("install")]
ASSETS_DIR = os.path.join(
    FILE_DIR, "src", "hri", "packages", "nlp", "assets", "extract_data"
)


class ExtractedData(BaseModel):
    data: Optional[str] = None


class DataExtractor(Node):
    """Class to encapsulate the guest analysis node"""

    base_url: Optional[str]
    model: Optional[str]

    def __init__(self) -> None:
        global EXTRACT_DATA_SERVICE
        """Initialize the ROS2 node"""
        super().__init__("data_extractor")

        self.declare_parameter("base_url", "None")
        self.declare_parameter("EXTRACT_DATA_SERVICE", EXTRACT_DATA_SERVICE)
        self.declare_parameter("temperature", 0.5)
        self.declare_parameter("spacy_model", "en_core_web_md")

        spacy_model = (
            self.get_parameter("spacy_model").get_parameter_value().string_value
        )

        # Initialize spacy
        os.makedirs(ASSETS_DIR, exist_ok=True)

        try:
            self.nlp = spacy.load(os.path.join(ASSETS_DIR, spacy_model))
        except OSError:
            spacy.cli.download(spacy_model)
            self.nlp = spacy.load(spacy_model)
            self.nlp.to_disk(os.path.join(ASSETS_DIR, spacy_model))

        base_url = self.get_parameter("base_url").get_parameter_value().string_value
        if base_url == "None":
            base_url = None

        self.temperature = (
            self.get_parameter("temperature").get_parameter_value().double_value
        )

        self.client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=base_url
        )

        EXTRACT_DATA_SERVICE = (
            self.get_parameter("EXTRACT_DATA_SERVICE")
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info("Starting data extractor node")

        self.srv = self.create_service(
            ExtractInfo, EXTRACT_DATA_SERVICE, self.extract_info_requested
        )

        self.get_logger().info("Data extractor node started")

    def extract_info_requested(
        self, request: ExtractInfo.Request, response: ExtractInfo.Response
    ) -> ExtractInfo.Response:
        """Service to extract information from text."""

        self.get_logger().info(f"Extracting {request.data} from text")

        if len(request.full_text.strip().split(" ")) < 2:
            self.get_logger().error(
                "Received text with less than 2 words. Returning input text as result."
            )
            response.result = request.full_text
            return response

        # Optimized implementations for specific data extraction requests
        if request.data == "name":
            response.result = self.extract_name(request.full_text)
            return response
        elif request.data == "loc" or request.data == "location":
            response.result = self.extract_loc(request.full_text)
            return response

        # Check if the data extraction must be performed using the LLM
        if "LLM_" not in request.data:
            self.get_logger().error(
                "Invalid data type requested. Prepend with LLM_ to use a generic LLM for data extraction."
            )
            response.result = ""
            return response

        request.data = request.data.replace("LLM_", "")

        messages, response_format = get_extract_data_args(
            request.full_text, request.data, request.context
        )

        response_content = (
            self.client.beta.chat.completions.parse(
                model=MODEL.EXTRACT_INFO_REQUESTED.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )

        self.get_logger().info(f"Extracted data: {response_content}")

        try:
            response_data = json.loads(response_content)
            result = ExtractedData(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(str(e))

        response.result = result.data
        return response

    def extract_name(self, text: str) -> str:
        name = extract_by_priority(self.nlp(text).ents, NAME_PRIORITY_LABELS)
        # Add a fallback to the text if no name is found
        if len(name) == 0:
            self.get_logger().error(
                f"No name found in {text}. Defaulting to returning the same input"
            )
            name = text

        return name

    def extract_loc(self, text: str) -> str:
        loc = extract_by_priority(self.nlp(text).ents, LOC_PRIORITY_LABELS)
        # Add a fallback to the text if no loc is found
        if len(loc) == 0:
            self.get_logger().error(
                f"No location found in {text}. Defaulting to returning the same input"
            )
            loc = text

        return loc


def main(args=None):
    rclpy.init(args=args)
    try:
        data_extractor = DataExtractor()
        rclpy.spin(data_extractor)
    except rclpy.exceptions.ROSInterruptException as e:
        data_extractor.get_logger().error(f"Error: {e}")
    finally:
        data_extractor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

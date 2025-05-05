#!/usr/bin/env python3

"""
Python ROS2 node to extract information from text
"""

import json
import os

# Libraries
from typing import Optional

import rclpy
from nlp.assets.dialogs import get_extract_data_args
from openai import OpenAI
from pydantic import BaseModel
from rclpy.node import Node

from frida_interfaces.srv import ExtractInfo
from frida_constants.hri_constants import MODEL

EXTRACT_DATA_SERVICE = "/extract_data"


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

        self.get_logger().info("Extracting information from text")

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

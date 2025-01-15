#!/usr/bin/env python3

"""
Python ROS2 node to extract information from text
"""

# Libraries
from typing import Optional
import rclpy
from rclpy.node import Node
import os
import openai
from pydantic import BaseModel
import json
from frida_interfaces.srv import ExtractInfo

EXTRACT_DATA_SERVICE = "/extract_data"


class ExtractedData(BaseModel):
    data: Optional[str] = None


class DataExtractor(Node):
    """Class to encapsulate the guest analysis node"""

    def __init__(self) -> None:
        """Initialize the ROS2 node"""
        super().__init__("data_extractor")
        self.get_logger().info("Starting data extractor node")

        self.srv = self.create_service(
            ExtractInfo, EXTRACT_DATA_SERVICE, self.extract_info_requested)

        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.get_logger().info("Data extractor node started")

    def extract_info_requested(self, request: ExtractInfo.Request, response: ExtractInfo.Response) -> ExtractInfo.Response:
        """Service to extract information from text."""

        self.get_logger().info("Extracting information from text")

        instruction = "You will be presented with some text and data to extract. Please provide the requested information or leave empty if it isn't available."
        response_content = openai.beta.chat.completions.parse(
            model="gpt-4o-2024-08-06",
            messages=[
                {"role": "system", "content": instruction},
                {"role": "user", "content": str(request.full_text) +
                    "Data to extract: " + str(request.data)}
            ],
            response_format=ExtractedData
        ).choices[0].message.content

        self.get_logger().info(f"Extracted data: {response_content}")

        try:
            response_data = json.loads(response_content)
            result = ExtractedData(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(
                str(e))

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

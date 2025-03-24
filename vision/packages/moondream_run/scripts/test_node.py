#!/usr/bin/env python3

"""
Node for Moondream functions
"""

# Import the generated gRPC modules
import grpc
import os
import sys
import cv2
import moondream_proto_pb2
import moondream_proto_pb2_grpc

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from frida_interfaces.srv import PersonDescription

# from frida_constants.vision_constants import CAMERA_TOPIC
from ament_index_python.packages import get_package_share_directory


PATH = get_package_share_directory("moondream_run")
CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"

# Add the moondream_server directory to sys.path
sys.path.append(os.path.join(PATH, "moondream_server"))

# Print the current path for debugging
# print("Current PATH:", sys.__file__)


TEST_TOPIC = "/vision/test"


class TestNode(Node):
    def __init__(self):
        super().__init__("receptionist_commands")
        self.bridge = CvBridge()
        self.image = None

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )
        self.person_description_service = self.create_service(
            PersonDescription,
            TEST_TOPIC,
            self.person_description_callback,
        )
        options = [
            ("grpc.max_receive_message_length", 200 * 1024 * 1024),
            ("grpc.max_send_message_length", 200 * 1024 * 1024),
        ]
        channel = grpc.insecure_channel("localhost:50052", options=options)
        self.stub = moondream_proto_pb2_grpc.MoonDreamServiceStub(channel)

        self.get_logger().info("MoondreamNode Ready.")

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def person_description_callback(self, request, response):
        """Callback to describe the person in the image."""
        self.get_logger().info("Executing service Person Description")
        if self.image is None:
            response.success = False
            response.description = "No image received"
            return response

        _, image_bytes = cv2.imencode(
            ".jpg", self.image
        )  # You can use ".png" for PNG format
        image_bytes = image_bytes.tobytes()

        res = ""
        # Send the bytes to the server
        # beverage_response = stub.FindBeverage(
        #     moondream_proto_pb2.FindBeverageRequest(
        #         encoded_image=image_bytes, subject="purple gatorade"
        #     )
        # )

        encoded = self.stub.EncodeImage(
            moondream_proto_pb2.ImageRequest(image_data=image_bytes)
        )
        description_response = self.stub.GeneratePersonDescription(
            moondream_proto_pb2.DescriptionRequest(
                encoded_image=encoded.encoded_image, query="Describe the image"
            )
        )
        print(f"Description: {description_response.answer}")
        response.description = res
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

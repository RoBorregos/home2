#!/usr/bin/env python3

"""
Node for Moondream functions
"""

import rclpy
import pathlib
from ultralytics import YOLO
from moondream_run.moondream_lib import MoonDreamModel, Position
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from frida_interfaces.srv import PersonDescription
from frida_interfaces.srv import BeverageLocation
from frida_interfaces.srv import PersonPosture


CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"
PERSON_DESCRIPTION_TOPIC = "/vision/person_description"
PERSON_POSTURE_TOPIC = "/vision/person_posture"
BEVERAGE_TOPIC = "/vision/beverage_location"


YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/yolov8n.pt"
# MOONDREAM_LOCATION = MOONDREAM_LOCATION = str(pathlib.Path(__file__).parent) + "/moondream-2b-int8.mf.gz"

CONF_THRESHOLD = 0.5


class MoondreamNode(Node):
    def __init__(self):
        super().__init__("receptionist_commands")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )
        self.person_description_service = self.create_service(
            PersonDescription,
            PERSON_DESCRIPTION_TOPIC,
            self.person_description_callback,
        )
        self.beverage_location_service = self.create_service(
            BeverageLocation, BEVERAGE_TOPIC, self.beverage_location_callback
        )
        self.person_posture_service = self.create_service(
            PersonPosture, PERSON_POSTURE_TOPIC, self.person_posture_callback
        )

        self.yolo_model = YOLO(YOLO_LOCATION)
        self.moondream_model = MoonDreamModel()

        self.get_logger().info("MoondreamNode Ready.")

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def person_description_callback(self, request, response):
        """Callback to describe the person in the image."""
        self.get_logger().info("Executing service Person Description")
        result = ""

        try:
            if self.image is None:
                raise Exception("No image received yet.")

            query = "Describe the clothing of the person in the image in a detailed and specific manner. Include the type of clothing, colors, patterns, and any notable accessories. Ensure that the description is clear and distinct."
            cropped_frame = self.detect_and_crop_person()
            encoded_image = self.moondream_model.encode_image(cropped_frame)
            result = self.moondream_model.generate_person_description(
                encoded_image, query, stream=False
            )
        except Exception as e:
            self.get_logger().error(f"Error describing person: {e}")
            response.description = result
            response.success = False
            return response

        response.description = result
        response.success = True
        self.success(f'Person description: "{result}"')
        return response

    def beverage_location_callback(self, request, response):
        """Callback to locate x,y bounding box in the image."""
        self.get_logger().info("Executing service Beverage Location")

        if self.image is None:
            response.location = "No image received yet."
            response.success = False
            return response

        frame = self.image
        encoded_image = self.moondream_model.encode_image(frame)
        beverage_position = self.moondream_model.find_beverage(
            encoded_image, request.beverage
        )

        if beverage_position == Position.NOT_FOUND:
            self.get_logger().warn("Beverage not found")
            response.location = beverage_position.value
            response.success = False
        else:
            response.location = beverage_position.value
            response.success = True
            self.get_logger().info(f"Beverage found at: {response.location}")
        return response

    def person_posture_callback(self, request, response):
        """Callback to determine the position of the person in the image."""
        self.get_logger().info("Executing service Person Posture")

        if self.image is None:
            response.position = "No image received yet."
            return response

        query = "Determine if the person is sitting, standing, or lying down. Just mention the pose, no aditional information is needed."
        cropped_frame = self.detect_and_crop_person()
        encoded_image = self.moondream_model.encode_image(cropped_frame)

        # Use same method for dperson_description but with different query
        response.description = self.moondream_model.generate_person_description(
            encoded_image, query, stream=False
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoondreamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

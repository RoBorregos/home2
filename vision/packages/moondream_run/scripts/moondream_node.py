#!/usr/bin/env python3

"""
Node for Moondream functions
"""

import grpc
import rclpy
import pathlib
from ultralytics import YOLO
import cv2
import sys
import os

# from moondream_run.moondream_lib import MoonDreamModel, Position
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from enum import Enum

from frida_interfaces.srv import PersonDescription
from frida_interfaces.srv import BeverageLocation
from frida_interfaces.srv import PersonPosture

from ament_index_python.packages import get_package_share_directory

PATH = get_package_share_directory("moondream_run")
sys.path.append(os.path.join(PATH, "moondream_server"))

# Import the generated gRPC modules
import moondream_proto_pb2  # noqa
import moondream_proto_pb2_grpc  # noqa

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/yolov8n.pt"

CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"
PERSON_DESCRIPTION_TOPIC = "/vision/person_description"
PERSON_POSTURE_TOPIC = "/vision/person_posture"
BEVERAGE_TOPIC = "/vision/beverage_location"


class Position(Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NOT_FOUND = "not found"


# MOONDREAM_LOCATION = MOONDREAM_LOCATION = str(pathlib.Path(__file__).parent) + "/moondream-2b-int8.mf.gz"

CONF_THRESHOLD = 0.5


class MoondreamNode(Node):
    def __init__(self):
        super().__init__("receptionist_commands")
        self.bridge = CvBridge()
        self.image = None

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
        # self.moondream_model = MoonDreamModel()

        # gRPC client setup
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
            self.get_logger().warn("No image received yet.")
            return response

        cropped_frame = self.detect_and_crop_person()
        _, image_bytes = cv2.imencode(".jpg", cropped_frame)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            description_response = self.stub.GeneratePersonDescription(
                moondream_proto_pb2.DescriptionRequest(
                    encoded_image=encoded.encoded_image, query="Describe the image"
                )
            )
            response.description = description_response.answer
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error describing person: {e}")
            response.description = ""
            response.success = False

        return response

    def beverage_location_callback(self, request, response):
        """Callback to locate x,y bounding box in the image."""
        self.get_logger().info("Executing service Beverage Location")

        if self.image is None:
            response.location = "No image received yet."
            response.success = False
            self.get_logger().warn("No image received yet.")
            return response

        _, image_bytes = cv2.imencode(".jpg", self.image)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            beverage_position = self.stub.FindBeverage(
                moondream_proto_pb2.FindBeverageRequest(
                    encoded_image=encoded.encoded_image, subject=request.beverage
                )
            )

            response.location = beverage_position.position
            response.success = True

        except Exception as e:
            self.get_logger().error(f"Error locating beverage: {e}")
            response.location = ""
            response.success = False

        return response

        # frame = self.image
        # encoded_image = self.moondream_model.encode_image(frame)
        # beverage_position = self.moondream_model.find_beverage(
        #     encoded_image, request.beverage
        # )

        # if beverage_position == Position.NOT_FOUND:
        #     self.get_logger().warn("Beverage not found")
        #     response.location = beverage_position.value
        #     response.success = False
        # else:
        #     response.location = beverage_position.value
        #     response.success = True
        #     self.get_logger().info(f"Beverage found at: {response.location}")
        # return response

    def person_posture_callback(self, request, response):
        """Callback to determine the position of the person in the image."""
        self.get_logger().info("Executing service Person Posture")

        if self.image is None:
            response.position = "No image received yet."
            return response

        query = "Determine if the person is sitting, standing, or lying down. Just mention the pose, no additional information is needed."
        cropped_frame = self.detect_and_crop_person()
        encoded_image = self.moondream_model.encode_image(cropped_frame)

        response.description = self.moondream_model.generate_person_description(
            encoded_image, query, stream=False
        )
        return response

    def detect_and_crop_person(self):
        """Check if there is a person in the frame, crop the image to the person with the largest area, and return the cropped frame."""
        if self.image is None:
            self.get_logger().warn("No image received yet.")
            return None

        frame = self.image
        self.output_image = frame.copy()

        results = self.yolo_model(frame, verbose=False, classes=0)
        largest_area = 0
        largest_box = None

        for out in results:
            for box in out.boxes:
                x, y, w, h = [round(i) for i in box.xywh[0].tolist()]
                confidence = box.conf.item()
                area = w * h

                if confidence > CONF_THRESHOLD and area > largest_area:
                    largest_area = area
                    largest_box = (x, y, w, h)

        if largest_box:
            x, y, w, h = largest_box
            self.person_found = True
            cv2.rectangle(
                self.output_image,
                (int(x - w / 2), int(y - h / 2)),
                (int(x + w / 2), int(y + h / 2)),
                (0, 255, 0),
                2,
            )
            cropped_frame = frame[
                int(y - h / 2) : int(y + h / 2), int(x - w / 2) : int(x + w / 2)
            ]
            return cropped_frame

        return None


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

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

from frida_interfaces.srv import BeverageLocation
from frida_interfaces.srv import PersonPosture, Query, CropQuery, IsSitting

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    PERSON_POSTURE_TOPIC,
    BEVERAGE_TOPIC,
    QUERY_TOPIC,
    CROP_QUERY_TOPIC,
    IS_SITTING_TOPIC,
)
from enum import Enum

from ament_index_python.packages import get_package_share_directory

PATH = get_package_share_directory("moondream_run")
sys.path.append(os.path.join(PATH, "moondream_server"))

# Import the generated gRPC modules
import moondream_proto_pb2  # noqa
import moondream_proto_pb2_grpc  # noqa

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/yolov8n.pt"
NOT_FOUND = "not found"

# MOONDREAM_LOCATION = MOONDREAM_LOCATION = str(pathlib.Path(__file__).parent) + "/moondream-2b-int8.mf.gz"

CONF_THRESHOLD = 0.5


class Position(Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NOT_FOUND = "not found"


class MoondreamNode(Node):
    def __init__(self):
        super().__init__("moondream_node")
        self.bridge = CvBridge()
        self.image = None

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.beverage_location_service = self.create_service(
            BeverageLocation, BEVERAGE_TOPIC, self.beverage_location_callback
        )
        self.person_posture_service = self.create_service(
            PersonPosture, PERSON_POSTURE_TOPIC, self.person_posture_callback
        )

        self.query_service = self.create_service(
            Query, QUERY_TOPIC, self.query_callback
        )

        self.crop_query_service = self.create_service(
            CropQuery, CROP_QUERY_TOPIC, self.crop_query_callback
        )

        self.is_sitting_service = self.create_service(
            IsSitting, IS_SITTING_TOPIC, self.is_sitting_callback
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

    def query_callback(self, request, response):
        """Callback to query the image."""
        self.get_logger().info("Executing service Query")
        if self.image is None:
            response.result = "No image received yet."
            response.success = False
            self.get_logger().warn("No image received yet.")
            return response

        if request.person:
            crop = self.detect_and_crop_person()
            if crop is not None:
                self.image = crop
            else:
                self.get_logger().warn("No person detected. Describing general image.")

        _, image_bytes = cv2.imencode(".jpg", self.image)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            query_response = self.stub.Query(
                moondream_proto_pb2.QueryRequest(
                    encoded_image=encoded.encoded_image, query=request.query
                )
            )
            response.result = query_response.answer
            response.success = True
            self.success(f"Query executed successfully. Result: {response.result}")
        except Exception as e:
            self.get_logger().error(f"Error querying image: {e}")
            response.result = ""
            response.success = False

        return response

    def crop_query_callback(self, request, response):
        """Callback to describe the bag."""
        self.get_logger().info("Executing service Crop query")
        if self.image is None:
            response.result = "No image received yet."
            response.success = False
            self.get_logger().warn("No image received yet.")
            return response

        frame = self.image.copy()

        xmin = request.xmin
        ymin = request.ymin
        xmax = request.xmax
        ymax = request.ymax

        xmin = xmin * self.image.shape[1]
        ymin = ymin * self.image.shape[0]
        xmax = xmax * self.image.shape[1]
        ymax = ymax * self.image.shape[0]

        if (
            0 <= xmin < self.image.shape[1]
            and 0 <= ymin < self.image.shape[0]
            and 0 < xmax <= self.image.shape[1]
            and 0 < ymax <= self.image.shape[0]
        ):
            print(f"Crop coordinates: {xmin}, {ymin}, {xmax}, {ymax}")
            cropped = frame[int(ymin) : int(ymax), int(xmin) : int(xmax)]
            # save image

        else:
            response.result = "Crop coordinates are out of bounds."
            self.get_logger().warn("Crop coordinates are out of bounds.")
            response.success = False
            return response

        _, image_bytes = cv2.imencode(".jpg", cropped)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            bag_description = self.stub.Query(
                moondream_proto_pb2.QueryRequest(
                    encoded_image=encoded.encoded_image, query=request.query
                )
            )

            print(bag_description.answer)
            response.result = bag_description.answer
            response.success = True
            self.success(f"Query executed successfully. Result: {response.result}")

        except Exception as e:
            self.get_logger().error(f"Error describing bag: {e}")
            response.result = ""
            response.success = False

        return response

    def is_sitting_callback(self, request, response):
        """Callback to determine if a person in the provided image is sitting."""
        self.get_logger().info("Executing service Is Sitting")

        try:
            image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            success, image_bytes = cv2.imencode(".jpg", image)
            if not success:
                response.answer = False
                response.success = False
                return response

            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes.tobytes())
            )

            prompt = (
                "Is the person in this image sitting? " "Answer only with yes or no."
            )
            query_response = self.stub.Query(
                moondream_proto_pb2.QueryRequest(
                    encoded_image=encoded.encoded_image,
                    query=prompt,
                )
            )

            answer = query_response.answer.strip()
            answer = answer.lower()
            if answer == "yes":
                response.answer = True
            elif answer == "no":
                response.answer = False
            else:
                self.get_logger().warn(
                    f"Unexpected answer from Moondream: '{answer}'. Expected 'yes' or 'no'."
                )
                response.answer = False

            self.get_logger().info("Moondream answer: %s", answer)
            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"Error checking sitting posture: {e}")
            response.answer = False
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
            print(beverage_position.position)
            if beverage_position.position == Position.NOT_FOUND:
                self.get_logger().warn("Beverage not found")
                response.success = False
            else:
                self.success(f"Beverage location found at: {response.location}")
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

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

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

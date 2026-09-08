#!/usr/bin/env python3

"""
Node for Moondream functions
"""

import grpc
import rclpy
import cv2
import sys
import os

# from moondream_run.moondream_lib import MoonDreamModel, Position
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from frida_interfaces.srv import Query, CropQuery, ObjectPoints
from frida_interfaces.srv import MoondreamDetection
from frida_interfaces.msg import Point2D, ObjectDetection

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    QUERY_TOPIC,
    CROP_QUERY_TOPIC,
    OBJECT_POINTS_TOPIC,
    MOONDREAM_DETECTION_TOPIC,
)

from ament_index_python.packages import get_package_share_directory

PATH = get_package_share_directory("moondream_run")
sys.path.append(os.path.join(PATH, "moondream_server"))

# Import the generated gRPC modules
import moondream_proto_pb2  # noqa
import moondream_proto_pb2_grpc  # noqa

NOT_FOUND = "not found"

# MOONDREAM_LOCATION = MOONDREAM_LOCATION = str(pathlib.Path(__file__).parent) + "/moondream-2b-int8.mf.gz"

CONF_THRESHOLD = 0.5


class MoondreamNode(Node):
    def __init__(self):
        super().__init__("moondream_node")
        self.bridge = CvBridge()
        self.image = None

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.query_service = self.create_service(
            Query, QUERY_TOPIC, self.query_callback
        )

        self.crop_query_service = self.create_service(
            CropQuery, CROP_QUERY_TOPIC, self.crop_query_callback
        )
        self.object_points_service = self.create_service(
            ObjectPoints, OBJECT_POINTS_TOPIC, self.object_points_callback
        )
        self.detect_service = self.create_service(
            MoondreamDetection, MOONDREAM_DETECTION_TOPIC, self.detect_callback
        )

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
            self.get_logger().info(
                f"\033[92mSUCCESS:\033[0m Query executed successfully. Result: {response.result}"
            )
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
            query_response = self.stub.Query(
                moondream_proto_pb2.QueryRequest(
                    encoded_image=encoded.encoded_image, query=request.query
                )
            )

            print(query_response.answer)
            response.result = query_response.answer
            response.success = True
            self.get_logger().info(
                f"\033[92mSUCCESS:\033[0m Query executed successfully. Result: {response.result}"
            )

        except Exception as e:
            self.get_logger().error(f"Error describing bag: {e}")
            response.result = ""
            response.success = False

        return response

    def object_points_callback(self, request, response):
        """Callback to get the points of a subject in the current image."""
        self.get_logger().info("Executing service Object Points")

        if self.image is None:
            response.points = []
            response.success = False
            self.get_logger().warn("No image received yet.")
            return response

        _, image_bytes = cv2.imencode(".jpg", self.image)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            grpc_response = self.stub.FindObjectPoints(
                moondream_proto_pb2.FindObjectPointsRequest(
                    encoded_image=encoded.encoded_image,
                    subject=request.subject,
                )
            )

            if not grpc_response.found or len(grpc_response.points) == 0:
                response.points = []
                response.success = False
                self.get_logger().warn(f"No points found for {request.subject}")
                return response

            ros_points = []
            for pt in grpc_response.points:
                point = Point2D()
                point.x = pt.x
                point.y = pt.y
                ros_points.append(point)

            response.points = ros_points
            response.success = True
            self.get_logger().info(
                f"\033[92mSUCCESS:\033[0m Found {len(ros_points)} points for {request.subject}"
            )

        except Exception as e:
            self.get_logger().error(f"Error getting object points: {e}")
            response.points = []
            response.success = False

        return response

    def detect_callback(self, request, response):
        """Callback to detect a subject's bounding boxes (normalized) in the current image."""
        self.get_logger().info(f"Executing service Detect for '{request.subject}'")

        if self.image is None:
            response.success = False
            response.message = "No image received yet."
            self.get_logger().warn("No image received yet.")
            return response

        _, image_bytes = cv2.imencode(".jpg", self.image)
        image_bytes = image_bytes.tobytes()

        try:
            encoded = self.stub.EncodeImage(
                moondream_proto_pb2.ImageRequest(image_data=image_bytes)
            )
            grpc_response = self.stub.Detect(
                moondream_proto_pb2.DetectRequest(
                    encoded_image=encoded.encoded_image,
                    subject=request.subject,
                )
            )

            if not grpc_response.found:
                response.success = False
                response.message = f"No '{request.subject}' detected."
                self.get_logger().warn(response.message)
                return response

            for obj in grpc_response.objects:
                detection = ObjectDetection()
                detection.label_text = obj.name
                detection.score = 1.0
                detection.xmin = obj.x_min
                detection.ymin = obj.ymin
                detection.xmax = obj.x_max
                detection.ymax = obj.y_max
                response.detections.append(detection)

            response.success = True
            response.message = f"Found {len(response.detections)} detections."
            self.get_logger().info(
                f"\033[92mSUCCESS:\033[0m Detect found {len(response.detections)} '{request.subject}'"
            )

        except Exception as e:
            self.get_logger().error(f"Error detecting object: {e}")
            response.success = False
            response.message = str(e)

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

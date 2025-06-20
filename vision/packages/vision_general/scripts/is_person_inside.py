#!/usr/bin/env python3

"""
Node to send the 3D coordinates of a person detected in the camera feed.
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import cv2
import os
import json
from ament_index_python.packages import get_package_share_directory

from frida_interfaces.srv import PersonInsideReq, PointTransformation

from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
    PERSON_INSIDE_REQUEST_TOPIC,
)

from frida_constants.integration_constants import POINT_TRANSFORMER_TOPIC

from vision_general.utils.calculations import estimate_3d_from_pose

class Person3DEstimator:
    def __init__(self, camera_frame=CAMERA_FRAME):
        self.pose_detector = PoseDetection()
        self.camera_frame = camera_frame

    def estimate(self, image_bgr, depth_image, camera_info):
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        results = self.pose_detector.detect(image_rgb)

        if not results.pose_landmarks or len(results.pose_landmarks.landmark) <= 12:
            return None

        return estimate_3d_from_pose(image_bgr, results.pose_landmarks.landmark, camera_info, depth_image)

class ImageBuffer:
    def __init__(self):
        self.image = None
        self.depth = None
        self.info = None

    def ready(self):
        return self.image is not None and self.depth is not None and self.info is not None

class IsPersonInside(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.bridge = CvBridge()
        self.buffer = ImageBuffer()
        self.estimator = Person3DEstimator()

        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10)

        self.create_service(PersonInsideReq, PERSON_INSIDE_REQUEST_TOPIC, self.handle_request)

        self.point_transform_client = self.create_client(
            PointTransformation, POINT_TRANSFORMER_TOPIC
        )

        while not self.point_transform_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for point_transformer service...")


    def image_callback(self, data):
        self.buffer.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        try:
            self.buffer.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def image_info_callback(self, data):
        self.buffer.info = data

    def transform_point_to_map(self, point_stamped):
        request = PointTransformation.Request()
        request.target_frame = "map"
        request.point = point_stamped

        future = self.point_transform_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            return future.result().transformed_point
        else:
            self.get_logger().error("Transform failed or service unavailable")
            return None
        
    def is_inside_polygon(self, x, y, polygon):
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if (y1 > y) != (y2 > y):
                xinters = (y - y1) * (x2 - x1) / (y2 - y1 + 1e-10) + x1
                if x < xinters:
                    inside = not inside
        return inside
    
    def get_area_from_position(self, x, y):
        package_share_directory = get_package_share_directory("frida_constants")
        path = os.path.join(package_share_directory, "map_areas/areas.json")

        with open(path, "r") as file:
            areas = json.load(file)

        for area in areas:
            self.get_logger().info(f"Looking in area: {area}")
            if self.is_inside_polygon(x, y, self.areas[area]["polygon"]):
                return area
            
        return None

    def handle_request(self, request, response):
        if not self.buffer.ready():
            response.success = False
            return response

        frame = self.buffer.image[int(request.ymin):int(request.ymax), int(request.xmin):int(request.xmax)]
        point3d = self.estimator.estimate(frame, self.buffer.depth, self.buffer.info)

        if point3d is None:
            response.success = False
            return response

        person_coords = PointStamped()
        person_coords.header.frame_id = self.estimator.camera_frame
        person_coords.point.x = float(point3d[0])
        person_coords.point.y = float(point3d[1])
        person_coords.point.z = float(point3d[2])

        transformed = self.transform_point_to_map(person_coords)

        if transformed:
            x = transformed.point.x
            y = transformed.point.y
            self.get_logger().info(f"Person is at x={x:.2f}, y={y:.2f} in map")

            area = self.get_area_from_position(x, y)
            if area:
                self.get_logger().info(f"Person is inside: {area}")
                response.location = area
            else:
                self.get_logger().warn("Person is not inside any known area.")
                response.location = "Unknown"
                response.success = True
        else:
            self.get_logger().error("Failed to transform point to map frame")
            response.success = False
            response.location = "Unknown"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IsPersonInside()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

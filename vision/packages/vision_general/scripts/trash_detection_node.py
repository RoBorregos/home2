#!/usr/bin/env python3
"""
Node exposing a service to filter YOLO detections by label_text.
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import json
import os

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DETECTIONS_TOPIC,
    TRASH_SERVICE_CATEGORY,
    DEPTH_IMAGE_TOPIC,
    OBJECT_POINTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
    TRASHCAN_SERVICE,
)
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import ObjectPoints, TrashcanDetection, SetTrashCategory
from vision_general.utils.calculations import get_depth, deproject_pixel_to_point
from vision_general.utils.ros_utils import wait_for_future

TRASH_DEBUG_IMAGE_TOPIC = "/vision/trash_detection_debug"


class TrashDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("trash_detection_node")
        self.bridge = CvBridge()
        self.latest_detections = ObjectDetectionArray()
        self.image = None
        self.imageInfo = None
        self.category = None
        self.depth_image = None

        # Load object_to_category mapping from objects.json
        package_share_directory = get_package_share_directory("frida_constants")
        objects_json_path = os.path.join(package_share_directory, "data/objects.json")

        try:
            with open(objects_json_path, "r") as f:
                objects_data = json.load(f)
                self.object_to_category = objects_data.get("object_to_category", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load objects.json: {e}")
            self.object_to_category = {}

        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        self.create_service(
            SetTrashCategory,
            TRASH_SERVICE_CATEGORY,
            self.set_trash_category,
        )

        self.create_service(
            TrashcanDetection,
            TRASHCAN_SERVICE,
            self.get_trashcan,
        )

        self.trash_pub = self.create_publisher(
            ObjectDetectionArray, DETECTIONS_TOPIC, 10
        )
        self.debug_image_pub = self.create_publisher(Image, TRASH_DEBUG_IMAGE_TOPIC, 10)

        self.moondream_point_client = self.create_client(
            ObjectPoints, OBJECT_POINTS_TOPIC
        )

        self.get_logger().info("Trash service ready")

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def image_info_callback(self, data):
        self.imageInfo = data

    def detections_callback(self, data):
        self.latest_detections = data
        self.publish_trash()

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def set_trash_category(self, req, res):
        if not req.category:
            res.success = False
            self.get_logger().warn("No category input in request")
            return res
        elif req.category:
            requested_category = req.category.lower()
        else:
            requested_category = None

        self.category = requested_category
        res.success = True
        return res

    def publish_trash(self):
        filtered = ObjectDetectionArray()

        for det in self.latest_detections.detections:
            label = det.label_text.strip().lower()
            if not label or label.startswith("trash/"):
                continue

            # Check the detected object's category 
            obj_category = self.object_to_category.get(label)
            if obj_category and obj_category != self.category:
                continue

            detection = ObjectDetection()
            if obj_category == self.category:
                self.get_logger().info(f"Detected TRASH/{det.label_text}")
                detection.label = det.label
                detection.label_text = f"trash/{det.label_text}"
                detection.score = det.score
                detection.ymin = det.ymin
                detection.xmin = det.xmin
                detection.ymax = det.ymax
                detection.xmax = det.xmax
                detection.point3d = det.point3d
                filtered.detections.append(detection)

        self.trash_pub.publish(filtered)

    def get_trashcan(self, req, res):
        if self.imageInfo is None:
            self.get_logger().warn("Cannot detect trashcan without camera info")
            return []

        if self.depth_image is None:
            self.get_logger().warn("Cannot detect trashcan without depth image")
            return []

        trashcan_pts = self.get_moondream_points("black trashcan")

        if not trashcan_pts:
            self.get_logger().warn("No trashcan detected")
            return []

        trashcans = ObjectDetectionArray()
        debug_points = []
        for pt in trashcan_pts:
            self.get_logger().info("Detected trashcan")
            trashcan = ObjectDetection()
            point2d = (
                min(
                    max(int(pt[0] * self.imageInfo.width), 0), self.imageInfo.width - 1
                ),
                min(
                    max(int(pt[1] * self.imageInfo.height), 0),
                    self.imageInfo.height - 1,
                ),
            )
            debug_points.append(point2d)
            depth = get_depth(self.depth_image, point2d)
            point3d = deproject_pixel_to_point(self.imageInfo, point2d, depth)
            ptStamped = self.build_point_stamped(point3d)

            trashcan.label_text = "trashcan"
            trashcan.point3d = ptStamped
            trashcans.detections.append(trashcan)

        self.publish_trashcan_debug_image(debug_points)
        self.trash_pub.publish(trashcans)
        res.detection_array = trashcans
        res.success = True
        return res

    def publish_trashcan_debug_image(self, points2d):
        if self.image is None or not points2d:
            return

        debug_image = self.image.copy()

        for i, (u, v) in enumerate(points2d, start=1):
            cv2.circle(debug_image, (u, v), 12, (0, 0, 255), -1)
            cv2.circle(debug_image, (u, v), 20, (255, 255, 255), 2)
            cv2.putText(
                debug_image,
                f"trashcan_{i}",
                (u + 10, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

    def build_point_stamped(self, xyz):
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = CAMERA_FRAME
        point_stamped.point.x = float(xyz[2])
        point_stamped.point.y = float(-xyz[0])
        point_stamped.point.z = float(-xyz[1])
        return point_stamped

    def get_moondream_points(self, subject) -> list[tuple[float, float]]:
        """Get object points from the MoonDream service."""
        req = ObjectPoints.Request()
        req.subject = subject

        future = self.moondream_point_client.call_async(req)
        future = wait_for_future(future, 15)

        if future is False or not future.done():
            self.get_logger().error("MoonDream service call timed out or failed")
            return []

        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("MoonDream table point detection failed")
            return []

        return [(p.x, p.y) for p in result.points]


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

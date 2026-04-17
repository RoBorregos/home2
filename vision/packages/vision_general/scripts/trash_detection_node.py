#!/usr/bin/env python3
"""
Node exposing a service to filter YOLO detections by label_text.
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from frida_constants.vision_constants import (
    DETECTIONS_TOPIC,
    DEPTH_IMAGE_TOPIC,
    OBJECT_POINTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
    TRASHCAN_SERVICE,
    MOONDREAM_POINT_3D_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import ObjectPoints, TrashcanDetection, MoondreamPoint3D
from vision_general.utils.calculations import get_depth, deproject_pixel_to_point
from vision_general.utils.ros_utils import wait_for_future

TRASH_DEBUG_IMAGE_TOPIC = "/vision/trash_detection_debug"


class TrashDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("trash_detection_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.image = None
        self.imageInfo = None
        self.depth_image = None

        self.create_subscription(Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.create_service(
            TrashcanDetection,
            TRASHCAN_SERVICE,
            self.get_trashcan,
            callback_group=self.callback_group,
        )

        self.trashcan_pub = self.create_publisher(
            ObjectDetectionArray, DETECTIONS_TOPIC, 10
        )

        self.debug_image_pub = self.create_publisher(Image, TRASH_DEBUG_IMAGE_TOPIC, 10)

        self.moondream_point_client = self.create_client(
            ObjectPoints, OBJECT_POINTS_TOPIC
        )

        self.create_service(
            MoondreamPoint3D,
            MOONDREAM_POINT_3D_TOPIC,
            self.get_moondream_point_3d,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Trash service ready")

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.trash_debug_image is not None:
                self.debug_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(self.trash_debug_image, "bgr8")
                )
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def image_info_callback(self, data):
        self.imageInfo = data

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def get_trashcan(self, req, res):
        if self.imageInfo is None:
            self.get_logger().warn("Cannot detect trashcan without camera info")
            return []

        if self.depth_image is None:
            self.get_logger().warn("Cannot detect trashcan without depth image")
            return

        trashcan_pts = self.get_moondream_points("black trashcan")

        if not trashcan_pts:
            self.get_logger().warn("No trashcan detected")
            return

        trashcans = ObjectDetectionArray()
        debug_image = None
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
            depth = get_depth(self.depth_image, point2d)
            point3d = deproject_pixel_to_point(self.imageInfo, point2d, depth)
            ptStamped = self.build_point_stamped(point3d)

            trashcan.label_text = "trashcan"
            trashcan.point3d = ptStamped
            trashcans.detections.append(trashcan)

            if self.image is not None and debug_image is None:
                debug_image = self.image.copy()
                u, v = point2d
                cv2.circle(debug_image, (u, v), 12, (255, 0, 0), -1)
                cv2.circle(debug_image, (u, v), 20, (255, 255, 255), 2)
                cv2.putText(
                    debug_image,
                    "trashcan",
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2,
                )

        self.trashcan_pub.publish(trashcans)
        res.detection_array = trashcans
        res.success = True

        if debug_image is not None:
            self.trash_debug_image = debug_image
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

        return res

    def build_point_stamped(self, xyz):
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = CAMERA_FRAME
        point_stamped.point.x = float(xyz[2])
        point_stamped.point.y = float(-xyz[0])
        point_stamped.point.z = float(-xyz[1])
        return point_stamped

    def get_moondream_point_3d(self, req, res):
        """Service to return the 3D PointStamped at the center of a subject
        detected by moondream point (e.g. 'washing machine door opening').
        Points returned by moondream are averaged to recover the center of
        a circular/concentrated object.
        """
        res.success = False

        if self.imageInfo is None:
            self.get_logger().warn("Cannot compute 3D point without camera info")
            return res
        if self.depth_image is None:
            self.get_logger().warn("Cannot compute 3D point without depth image")
            return res

        pts = self.get_moondream_points(req.subject)
        if not pts:
            self.get_logger().warn(f"No moondream points for subject '{req.subject}'")
            return res

        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)

        point2d = (
            min(max(int(cx * self.imageInfo.width), 0), self.imageInfo.width - 1),
            min(max(int(cy * self.imageInfo.height), 0), self.imageInfo.height - 1),
        )
        depth = get_depth(self.depth_image, point2d)
        point3d = deproject_pixel_to_point(self.imageInfo, point2d, depth)

        res.point = self.build_point_stamped(point3d)
        res.success = True
        self.get_logger().info(
            f"Moondream 3D point for '{req.subject}': "
            f"({res.point.point.x:.3f}, {res.point.point.y:.3f}, {res.point.point.z:.3f})"
        )
        return res

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
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

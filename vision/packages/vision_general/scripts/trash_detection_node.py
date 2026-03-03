#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from frida_interfaces.srv import SetDetectorClasses
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.msg import ObjectDetectionArray
from frida_constants.vision_constants import (
    ZERO_SHOT_DETECTIONS_TOPIC,
    SET_DETECTOR_CLASSES_SERVICE,
    TRASH_CLASS_SETTER_SERVICE,
    DEPTH_IMAGE_TOPIC,
    TRASH_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
)
from vision_general.utils.calculations import (
    get2DCentroid,
    get_depth,
    deproject_pixel_to_point,
)
import tf2_ros
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_point


class TrashDetectionNode(Node):
    """
    Class for trash detection
    **classes to detect are customizable**
    """

    def __init__(self):
        super().__init__("trash_detection_node")

        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_detections = ObjectDetectionArray()
        self.image = None
        self.depth_img = None
        self.image_info = None

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_sub = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.detections_sub = self.create_subscription(
            ObjectDetectionArray,
            ZERO_SHOT_DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        self.trash_depth_publisher = self.create_publisher(
            PointStamped, TRASH_DEPTH_TOPIC, 10
        )

        self.set_classes_client = self.create_client(
            SetDetectorClasses,
            SET_DETECTOR_CLASSES_SERVICE,
            callback_group=self.callback_group,
        )

        self.set_trash_classes_srv = self.create_service(
            SetDetectorClasses,
            TRASH_CLASS_SETTER_SERVICE,
            self.set_yoloe_classes,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Trash Detection Node initialized.")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_img = depth_image
        except Exception as e:
            print(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.image_info = data

    def detections_callback(self, data):
        """
        Store latest yolo-e detections and publish 3D coordinates
        """
        self.latest_detections = data

        # Debug: mostrar qué datos tenemos
        self.get_logger().info(f"Received {len(data.detections)} detections")

        if self.depth_img is None or len(self.depth_img) == 0:
            self.get_logger().warn("No depth image available")
            # Aún así mostrar la imagen con detecciones
            if self.image is not None and len(data.detections) > 0:
                self.frame = self.image.copy()
                for detection in data.detections:
                    self.get_logger().info(
                        f"Trash detected at min: ({detection.xmin}, {detection.ymin} and max: ({detection.xmax}, {detection.ymax}) "
                    )
            return
        if self.image_info is None:
            self.get_logger().warn("No camera info available")
            return
        if self.image is None:
            self.get_logger().warn("No image available")
            return

        self.frame = self.image
        for detection in data.detections:
            try:
                object_coors = (
                    detection.xmin,
                    detection.ymin,
                    detection.xmax,
                    detection.ymax,
                )
                point2D = get2DCentroid(object_coors, self.frame)
                depth = get_depth(self.depth_img, point2D)
                point3D = deproject_pixel_to_point(self.image_info, point2D, depth)
                point3D = float(point3D[0]), float(point3D[1]), float(point3D[2])
                stamped_point = PointStamped()
                stamped_point.header.frame_id = "zed_camera_link"
                stamped_point.header.stamp = self.get_clock().now().to_msg()
                stamped_point.point.x = point3D[0]
                stamped_point.point.y = point3D[1]
                stamped_point.point.z = point3D[2]

                transform = self.tf_buffer.lookup_transform(
                    "base_link", stamped_point.header.frame_id, rclpy.time.Time()
                )

                transformed_point = do_transform_point(stamped_point, transform)
                self.trash_depth_publisher.publish(transformed_point)

                self.get_logger().info(
                    f"Trash '{detection.label_text}' at: x={stamped_point.point.x:.2f}, "
                    f"y={stamped_point.point.y:.2f}, z={stamped_point.point.z:.2f}"
                )

            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")

    def set_yoloe_classes(self, request, response):
        """
        Set classes for YOLO-E detector
        """
        if not self.set_classes_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_classes service not available")
            return False

        client_request = SetDetectorClasses.Request()
        client_request.class_names = request.class_names

        future = self.set_classes_client.call_async(client_request)
        future = self.wait_for_future(future, 15)

        if future.result() is not None:
            result = future.result()
            response.success = result.success
        else:
            self.get_logger().warn("Failed to set classes")
            response.success = False

        return response

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            pass

        return future


def main():
    rclpy.init()
    node = TrashDetectionNode()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

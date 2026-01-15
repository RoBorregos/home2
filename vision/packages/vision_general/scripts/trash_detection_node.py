#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from frida_interfaces.srv import DetectionHandler, SetDetectorClasses
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
    SET_DETECTOR_CLASSES_SERVICE,
    TRASH_DETECTION_SERVICE,
)


class TrashDetectionNode(Node):
    def __init__(self):
        super().__init__("trash_detection_node")

        self.callback_group = ReentrantCallbackGroup()

        self.bridge = CvBridge()
        self.current_image = None
        self.latest_detections = ObjectDetectionArray()

        self.image_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.detections_sub = self.create_subscription(
            ObjectDetectionArray,
            ZERO_SHOT_DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        self.detection_service = self.create_service(
            DetectionHandler,
            TRASH_DETECTION_SERVICE,
            self.detect_objects_callback,
            callback_group=self.callback_group,
        )

        self.set_classes_client = self.create_client(
            SetDetectorClasses, SET_DETECTOR_CLASSES_SERVICE
        )

        self.get_logger().info("Trash Detection Node initialized.")

    def image_callback(self, msg):
        """
        Update current image
        """
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detections_callback(self, msg):
        """
        Store latest yolo-e detections
        """
        self.latest_detections = msg

    def detect_objects_callback(self, request, response):
        """
        Handle detection request using yolo-e zero-shot detector
        """
        try:
            classes_to_detect = []

            # Handle both single label and multiple labels
            if hasattr(request, "labels") and request.labels:
                # Multiple labels passed as list
                classes_to_detect = request.labels
            elif hasattr(request, "label") and request.label:
                # Single label passed
                classes_to_detect = [request.label.strip()]

            if classes_to_detect:
                self._set_yoloe_classes(classes_to_detect)
                time.sleep(1.0)

            if self.current_image is None:
                self.get_logger().warn("No current image available")
                response.detection_array = ObjectDetectionArray()
                response.detection_array.detections = []
                response.success = False
                return response

            # DEBUG
            num_raw_detections = len(self.latest_detections.detections)
            self.get_logger().info(f"Raw detections from yolo-e: {num_raw_detections}")

            if num_raw_detections > 0:
                for i, det in enumerate(self.latest_detections.detections):
                    self.get_logger().info(
                        f"Detection {i}: label='{det.label}', confidence={det.score:.3f}"
                    )
            else:
                self.get_logger().warn("No raw detections received from yolo-e")

            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []

            if self.current_image is not None:
                h, w = self.current_image.shape[:2]

                for detection in self.latest_detections.detections:
                    obj_detection = ObjectDetection()
                    obj_detection.label_text = detection.label_text
                    obj_detection.score = detection.score
                    obj_detection.xmin = detection.xmin
                    obj_detection.ymin = detection.ymin
                    obj_detection.xmax = detection.xmax
                    obj_detection.ymax = detection.ymax

                    response.detection_array.detections.append(obj_detection)

            response.success = True

            num_detections = len(response.detection_array.detections)
            self.get_logger().info(f"Final detections: {num_detections}")

        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
            response.detection_array = ObjectDetectionArray()

        return response

    def _set_yoloe_classes(self, classes):
        """
        Set classes for YOLO-E detector
        """
        if not self.set_classes_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_classes service not available")
            return False

        request = SetDetectorClasses.Request()
        request.class_names = classes

        future = self.set_classes_client.call_async(request)
        future = self.wait_for_future(future, 15)

        if future.result() is not None:
            result = future.result()
            return result.success
        else:
            self.get_logger().warn("Failed to set classes")
            return False

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            # print("Waiting for future to complete...")
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

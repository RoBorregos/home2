#!/usr/bin/env python3
from object_detector_2d.Utils.ros_utils import wait_for_future
from frida_interfaces.srv import ShowDetection, DetectionHandler
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SHOW_DETECTION_TOPIC,
    DETECTION_HANDLER_TOPIC_SRV,
)
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node


class ShowDetectionNode(Node):

    def __init__(self):
        super().__init__("Show_detection_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.camara_image_callback, 10
        )

        self.handler_client = self.create_client(
            DetectionHandler,
            DETECTION_HANDLER_TOPIC_SRV,
            callback_group=self.callback_group,
        )

        self.srv = self.create_service(
            ShowDetection,
            SHOW_DETECTION_TOPIC,
            self.show_detections_callback,
            callback_group=self.callback_group,
        )

    def camara_image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except Exception as e:
            print(f"Error: {e}")

    def draw_bbox_label(self, detections):
        image = self.image

        for detection in detections:
            color = (255, 0, 0)
            ymin, xmin, ymax, xmax = (
                int(detection.ymin * image.shape[0]),
                int(detection.xmin * image.shape[1]),
                int(detection.ymax * image.shape[0]),
                int(detection.xmax * image.shape[1]),
            )
            thickness_rec = 6

            # draw bbox
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, thickness_rec)

            # draw label
            (text_width, text_height), _ = cv2.getTextSize(
                f"{detection.label_text}", cv2.FONT_HERSHEY_SIMPLEX, 1, 2
            )

            cv2.rectangle(
                image,
                (xmin - int(thickness_rec / 2), max(ymin - text_height - thickness_rec, 0)),
                (xmin + text_width + thickness_rec, max(ymin, text_height + thickness_rec)),
                color,
                -1,
            )

            cv2.putText(
                image,
                f"{detection.label_text}",
                (xmin + thickness_rec, max(ymin - thickness_rec, text_height)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
        return image

    def show_detections_callback(self, request, response):
        """Callback to draw label and bbox from the detections"""
        if not self.handler_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().error("No handler server available")
            response.result = Image()
            response.success = False
            return response

        request_h = DetectionHandler.Request()
        request_h.label = request.label
        request_h.closest_object = request.closest_object
        future = self.handler_client.call_async(request_h)
        future = wait_for_future(future)

        try:
            detections = future.result().detection_array.detections
        except Exception as e:
            print(f"Error: {e}")
            response.result = Image()
            response.success = False
            return response

        if not detections:
            self.get_logger().info("Nothing was detected, returning camara image")
            response.result = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
            response.success = False
            return response
        # create image
        frame_image = self.draw_bbox_label(detections)
        self.get_logger().info(f"Detections received : {len(detections)}")

        response.result = self.bridge.cv2_to_imgmsg(frame_image, "bgr8")
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShowDetectionNode()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

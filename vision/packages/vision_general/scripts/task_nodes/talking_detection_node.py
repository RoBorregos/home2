#!/usr/bin/env python3
"""Talking detection ROS2 node. Landmarks come from models.mediapipe_detector;
the mouth-ratio oscillation and debounce logic live in models.talking_detection.

Runs only when called: the request carries the frames of ONE person (e.g. the
face bbox crop from each frame, oldest first) and the response says whether that
person is talking. Callers such as face recognition send one request per face.

--- Run the node ---
    ros2 run vision_general talking_detection_node.py

--- Query the service ---
    ros2 service call /vision/is_talking frida_interfaces/srv/IsTalking "{frames: [...]}"
"""

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from frida_constants.vision_constants import (
    IS_TALKING_TOPIC,
    SILENT_MESSAGE,
    TALKING_MESSAGE,
)
from frida_interfaces.srv import IsTalking
from models.mediapipe_detector import MediapipeDetector
from models.talking_detection import MouthActivity
from rclpy.node import Node
from vision_runtime import spin


class TalkingDetection(Node):
    def __init__(self):
        super().__init__("talking_detection")
        self.bridge = CvBridge()
        self.detector = MediapipeDetector()

        self.create_service(IsTalking, IS_TALKING_TOPIC, self.is_talking_callback)
        self.get_logger().info("Talking detection ready")

    def is_talking_callback(self, request, response):
        activity = MouthActivity()
        for msg in request.frames:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                self.get_logger().warn(f"Image conversion error: {e}")
                continue
            faces = self.detector.detect(frame)
            activity.update(faces[0] if faces else None)

        response.is_talking = activity.confirmed_talking
        response.message = TALKING_MESSAGE if response.is_talking else SILENT_MESSAGE
        self.get_logger().info(
            f"Is talking query ({len(request.frames)} frames): {response.message}"
        )
        return response

    def destroy_node(self):
        self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    spin(TalkingDetection())


if __name__ == "__main__":
    main()

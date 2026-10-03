#!/usr/bin/env python3
"""Talking detection ROS2 node. Model logic lives in models.talking_detection.

--- Run the node ---
    ros2 run vision_general talking_detection_node.py

--- Query the service ---
    ros2 service call /vision/is_talking std_srvs/srv/Trigger
"""

import pathlib

import rclpy
from ament_index_python.packages import get_package_share_directory
from frida_constants.vision_constants import IMAGE_ORIENTED_TOPIC, IS_TALKING_TOPIC
from models.talking_detection import TalkingDetector
from std_srvs.srv import Trigger
from vision_runtime import VisionRuntime, spin

MODEL_PATH = (
    pathlib.Path(get_package_share_directory("vision_general"))
    / "Utils"
    / "models"
    / "face_landmarker.task"
)


class TalkingDetection(VisionRuntime):
    def __init__(self):
        super().__init__(
            "talking_detection",
            image_topic=IMAGE_ORIENTED_TOPIC,
            active_name="talking_detection",
        )
        self.detector = TalkingDetector(str(MODEL_PATH))
        self.processed_stamp = None

        self.create_service(
            Trigger,
            IS_TALKING_TOPIC,
            self.is_talking_callback,
            callback_group=self.callback_group,
        )
        self.create_timer(0.03, self.run, callback_group=self.callback_group)
        self.get_logger().info("Talking detection ready")

    def _active_callback(self, msg):
        super()._active_callback(msg)
        if not self.active:
            self.detector.reset()

    def run(self):
        if not self.active or self.image is None:
            return
        # Skip frames already processed so the debounce counts real frames.
        stamp = self.image_header.stamp if self.image_header is not None else None
        if stamp is not None and stamp == self.processed_stamp:
            return
        self.processed_stamp = stamp
        self.detector.update(self.image)

    def is_talking_callback(self, request, response):
        talking = self.detector.confirmed_talking
        response.success = talking
        response.message = "TALKING" if talking else "SILENT"
        self.get_logger().info(f"Is talking query: {response.message}")
        return response

    def destroy_node(self):
        self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    spin(TalkingDetection())


if __name__ == "__main__":
    main()

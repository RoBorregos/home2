#! /usr/bin/env python3
"""Runs the few-shot embedding gallery detector on its own, so it never delays ObjectDetect2D."""

import copy

import cv2 as cv
import rclpy
from frida_constants.vision_constants import (
    GALLERY_DETECTIONS_3D_TOPIC,
    GALLERY_DETECTIONS_ACTIVE_TOPIC,
    GALLERY_DETECTIONS_IMAGE_TOPIC,
    GALLERY_DETECTIONS_POSES_TOPIC,
    GALLERY_DETECTIONS_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray
from detectors.registry import ModelRegistry
from base_detector_node import BaseDetectorNode


class GalleryDetectorNode(BaseDetectorNode):
    MARKER_COLOR = (1.0, 0.5, 0.0)  # orange

    def __init__(self):
        super().__init__(
            "gallery_object_detector_2D_node",
            default_det_topic=GALLERY_DETECTIONS_TOPIC,
            default_det_img_topic=GALLERY_DETECTIONS_IMAGE_TOPIC,
            default_det_poses_topic=GALLERY_DETECTIONS_POSES_TOPIC,
            default_det_3d_topic=GALLERY_DETECTIONS_3D_TOPIC,
            default_active_topic=GALLERY_DETECTIONS_ACTIVE_TOPIC,
            fixed_active_topic="/vision/gallery_detector/active",
        )

        self.model = ModelRegistry.get("embedding_gallery")
        self.get_logger().info("Embedding gallery detector loaded")

    def box_color(self, det):
        return (0, 128, 255)  # orange in BGR

    # ------------------------------------------------------------------ inference

    def run(self, frame):
        if self.flip_image:
            frame = cv.rotate(frame, cv.ROTATE_180)
        visual = copy.deepcopy(frame)

        detections_2d = self.model.detect(frame)

        if self.flip_image:
            for det in detections_2d:
                det.bbox_.y2, det.bbox_.x2, det.bbox_.y1, det.bbox_.x1 = (
                    1 - det.bbox_.y1,
                    1 - det.bbox_.x1,
                    1 - det.bbox_.y2,
                    1 - det.bbox_.x2,
                )

        all_detections = self.extract_3d(detections_2d)
        all_detections = self.filter_by_distance(all_detections)

        self.publish_3d_markers(all_detections)
        self.publish_poses(all_detections)

        ros_detections = self.to_ros(all_detections)
        for det in ros_detections:
            det.point3d.header.stamp = self.curr_clock

        self.latest_detections = ros_detections
        self.pub_detections.publish(ObjectDetectionArray(detections=ros_detections))
        self.detections_frame = self.visualize(visual, ros_detections)

        if self.verbose:
            for i, d in enumerate(ros_detections):
                self.get_logger().info(f"Gallery #{i}: {d.label_text} ({d.score:.2f})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GalleryDetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

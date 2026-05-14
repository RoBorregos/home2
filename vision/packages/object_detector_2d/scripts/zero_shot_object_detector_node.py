#! /usr/bin/env python3
"""Runs YOLOE zero-shot detector with dynamically settable classes via SetDetectorClasses service."""

import copy

import cv2 as cv
import rclpy
from frida_constants.vision_constants import (
    SET_DETECTOR_CLASSES_SERVICE,
    ZERO_SHOT_DEFAULT_CLASSES,
    ZERO_SHOT_DETECTION_HANDLER_SRV,
    ZERO_SHOT_DETECTIONS_3D_TOPIC,
    ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
    ZERO_SHOT_DETECTIONS_POSES_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray
from frida_interfaces.srv import DetectionHandler, SetDetectorClasses
from detectors.registry import ModelRegistry
from base_detector_node import BaseDetectorNode


class ZeroShotDetectorNode(BaseDetectorNode):
    MARKER_COLOR = (0.0, 0.0, 1.0)  # blue

    def __init__(self):
        super().__init__(
            "zero_shot_object_detector_2D_node",
            default_det_topic=ZERO_SHOT_DETECTIONS_TOPIC,
            default_det_img_topic=ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
            default_det_poses_topic=ZERO_SHOT_DETECTIONS_POSES_TOPIC,
            default_det_3d_topic=ZERO_SHOT_DETECTIONS_3D_TOPIC,
            fixed_active_topic="/vision/zero_shot_detector/active",
        )

        initial_classes = self.declare_param(
            "CLASSES", ZERO_SHOT_DEFAULT_CLASSES
        ).string_array_value
        classes_srv = self.declare_param(
            "SET_DETECTOR_CLASSES_SERVICE", SET_DETECTOR_CLASSES_SERVICE
        ).string_value

        self.model = ModelRegistry.get("zero_shot")
        if initial_classes:
            self.model.set_classes(list(initial_classes))
        self.get_logger().info(
            f"ZeroShot loaded — initial classes: {list(initial_classes)}"
        )

        self.create_service(SetDetectorClasses, classes_srv, self.set_classes)

        self.create_service(
            DetectionHandler, ZERO_SHOT_DETECTION_HANDLER_SRV, self.detection_handler
        )

    def box_color(self, det):
        return (255, 0, 0)  # blue in BGR

    # ------------------------------------------------------------------ services

    def detection_handler(self, request, response):
        detections = list(self.latest_detections)
        if request.label and request.label != "all":
            detections = [d for d in detections if d.label_text == request.label]
        elif request.labels:
            labels_set = set(request.labels)
            detections = [d for d in detections if d.label_text in labels_set]
        if request.closest_object and detections:
            detections = [
                min(
                    detections,
                    key=lambda d: d.point3d.point.x**2
                    + d.point3d.point.y**2
                    + d.point3d.point.z**2,
                )
            ]
        response.detection_array.detections = detections
        response.success = len(detections) > 0
        self.get_logger().info(
            f"ZeroShotDetectionHandler: label='{request.label}' returned={len(detections)}"
        )
        return response

    def set_classes(self, request, response):
        self.model.set_classes(list(request.class_names))
        self.get_logger().info(
            f"Zero-shot classes updated: {list(request.class_names)}"
        )
        return response

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
                self.get_logger().info(f"ZeroShot #{i}: {d.label_text} ({d.score:.2f})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ZeroShotDetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#! /usr/bin/env python3
"""Runs YOLOE zero-shot detector with dynamically settable classes via SetDetectorClasses service."""

import copy

import cv2 as cv
import rclpy
from frida_constants.vision_constants import (
    SET_DETECTOR_CLASSES_SERVICE,
    ZERO_SHOT_DEFAULT_CLASSES,
    ZERO_SHOT_DETECTIONS_3D_TOPIC,
    ZERO_SHOT_DETECTIONS_ACTIVE_TOPIC,
    ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
    ZERO_SHOT_DETECTIONS_POSES_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
)
from frida_interfaces.msg import ObjectDetectionArray
from frida_interfaces.srv import SetDetectorClasses
from detectors.registry import ModelRegistry
from base_detector_node import BaseDetectorNode


class ZeroShotDetectorNode(BaseDetectorNode):
    MARKER_COLOR = (0.0, 0.0, 1.0)  # blue


# TODO DEFINE HOW TO GET params


class zero_shot_object_detector_node(object_detector_node):
    def __init__(self, node_name: str = "zero_shot_object_detector_2D_node"):
        super(object_detector_node, self).__init__(node_name)

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.camera_info = CameraInfo()
        self.detections_frame = []

        self.set_parameters()
        self.active_flag = not self.node_params.USE_ACTIVE_FLAG

        self.object_detector_2d = YoloEObjectDetector(
            self.node_params.YOLO_MODEL_PATH, self.object_detector_parameters
        )
        self.object_detector_2d.set_classes(self.active_classes)

        self.handleSubcriptions()
        self.handlePublishers()
        self.handleServices()
        self.runThread = None
        # Bypass super().__init__ (parent is the regular detector), so the
        # frame-skip bookkeeping fields that rgbImageCallback reads are never
        # created. Mirror them here so the shared callback doesn't crash.
        self._frame_count = 0
        self._skip_frames = 2

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Frames per second throughput estimator
        self.curr_clock = 0
        self._frame_count = 0
        self._skip_frames = 2

        self.get_logger().info("Object Detector 2D Node has been started")

    def set_parameters(self):
        self.active_classes = (
            self.get_parameter("CLASSES").get_parameter_value().string_array_value
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

    def box_color(self, det):
        return (255, 0, 0)  # blue in BGR

    # ------------------------------------------------------------------ services

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

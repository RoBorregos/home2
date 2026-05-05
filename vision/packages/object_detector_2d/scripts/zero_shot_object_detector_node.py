#! /usr/bin/env python3
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
from models.registry import ModelRegistry
from base_detector_node import BaseDetectorNode


class ZeroShotDetectorNode(BaseDetectorNode):
    _MARKER_COLOR = (0.0, 0.0, 1.0)  # blue

    def __init__(self):
        super().__init__(
            "zero_shot_object_detector_2D_node",
            default_det_topic=ZERO_SHOT_DETECTIONS_TOPIC,
            default_det_img_topic=ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
            default_det_poses_topic=ZERO_SHOT_DETECTIONS_POSES_TOPIC,
            default_det_3d_topic=ZERO_SHOT_DETECTIONS_3D_TOPIC,
            default_active_topic=ZERO_SHOT_DETECTIONS_ACTIVE_TOPIC,
            fixed_active_topic="/vision/zero_shot_detector/active",
        )

        def _p(name, default):
            return self.declare_parameter(name, default).get_parameter_value()

        initial_classes = _p("CLASSES", ZERO_SHOT_DEFAULT_CLASSES).string_array_value
        classes_srv = _p(
            "SET_DETECTOR_CLASSES_SERVICE", SET_DETECTOR_CLASSES_SERVICE
        ).string_value

        self.model = ModelRegistry.get("zero_shot")
        if initial_classes:
            self.model.set_classes(list(initial_classes))
        self.get_logger().info(
            f"ZeroShot loaded — initial classes: {list(initial_classes)}"
        )

        self.create_service(SetDetectorClasses, classes_srv, self._set_classes)

    def _box_color(self, det):
        return (255, 0, 0)  # blue in BGR

    # ------------------------------------------------------------------ services

    def _set_classes(self, request, response):
        self.model.set_classes(list(request.class_names))
        self.get_logger().info(
            f"Zero-shot classes updated: {list(request.class_names)}"
        )
        return response

    # ------------------------------------------------------------------ inference

    def _run(self, frame):
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

        all_detections = self._extract_3d(detections_2d)
        all_detections = self._filter_by_distance(all_detections)

        self._publish_3d_markers(all_detections)
        self._publish_poses(all_detections)

        ros_detections = self._to_ros(all_detections)
        for det in ros_detections:
            det.point3d.header.stamp = self.curr_clock

        self.latest_detections = ros_detections
        self.pub_detections.publish(ObjectDetectionArray(detections=ros_detections))
        self.detections_frame = self._visualize(visual, ros_detections)

        if self.verbose:
            for i, d in enumerate(ros_detections):
                self.get_logger().info(f"ZeroShot #{i}: {d.label_text} ({d.score:.2f})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ZeroShotDetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

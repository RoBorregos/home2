#! /usr/bin/env python3
"""Runs multiple YOLO models continuously, publishes detections, and exposes trash/detection/yolo services."""

import copy
import json
import os

import cv2 as cv
import rclpy
from ament_index_python.packages import get_package_share_directory
from frida_constants.vision_constants import (
    DETECTION_HANDLER_TOPIC_SRV,
    DETECTIONS_3D_TOPIC,
    DETECTIONS_ACTIVE_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
    DETECTIONS_POSES_TOPIC,
    DETECTIONS_TOPIC,
    TRASH_SERVICE_CATEGORY,
    YOLO_DETECTION_TOPIC,
    YOLO_DETECTIONS_PUBLISHER_TOPIC,
)

from frida_interfaces.srv import SetTrashCategory

MODELS_PATH = str(pathlib.Path(__file__).parent) + "/models/"

print("PATH   ", MODELS_PATH)
ARGS = {
    "RGB_IMAGE_TOPIC": CAMERA_TOPIC,
    "DEPTH_IMAGE_TOPIC": DEPTH_IMAGE_TOPIC,
    "CAMERA_INFO_TOPIC": CAMERA_INFO_TOPIC,
    "DETECTIONS_TOPIC": DETECTIONS_TOPIC,
    "DETECTIONS_IMAGE_TOPIC": DETECTIONS_IMAGE_TOPIC,
    "DETECTIONS_POSES_TOPIC": DETECTIONS_POSES_TOPIC,
    "DETECTIONS_3D_TOPIC": DETECTIONS_3D_TOPIC,
    "DETECTIONS_ACTIVE_TOPIC": DETECTIONS_ACTIVE_TOPIC,
    "DEBUG_IMAGE_TOPIC": DEBUG_IMAGE_TOPIC,
    "CAMERA_FRAME": CAMERA_FRAME,
    "TARGET_FRAME": "base_link",
    "YOLO_MODEL_PATH": "tmr2026.pt",
    "USE_ACTIVE_FLAG": False,
    "DEPTH_ACTIVE": True,
    "VERBOSE": False,
    "USE_YOLO8": False,
    "USE_YOLO26": True,
    "FLIP_IMAGE": False,
    "USE_ZED_TRANSFORM": True,
    "MIN_SCORE_THRESH": 0.75,
    "MAX_DEPTH_THRESH": 2.0,
}


@dataclass
class NodeParams:
    RGB_IMAGE_TOPIC: str = None
    DEPTH_IMAGE_TOPIC: str = None
    CAMERA_INFO_TOPIC: str = None
    DETECTIONS_TOPIC: str = None
    DETECTIONS_POSES_TOPIC: str = None
    DETECTIONS_3D_TOPIC: str = None
    DETECTIONS_ACTIVE_TOPIC: str = None
    DETECTIONS_IMAGE_TOPIC: str = None
    DEBUG_IMAGE_TOPIC: str = None
    YOLO_MODEL_PATH: str = None
    USE_ACTIVE_FLAG: bool = None
    VERBOSE: bool = None
    USE_YOLO8: bool = None
    USE_YOLO26: bool = None
    MAX_DEPTH_THRESH: float = None


# TODO DEFINE HOW TO GET params


class object_detector_node(rclpy.node.Node):
    def __init__(self, node_name: str = "object_detector_2D_node"):
        super().__init__(node_name)

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.detections_frame = []

        # Trash detection logic — set via /vision/set_trash_category service
        self.category = None
        try:
            objects_path = os.path.join(
                get_package_share_directory("frida_constants"), "data/objects.json"
            )
            with open(objects_path) as f:
                self.object_to_category = json.load(f).get("object_to_category", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load objects.json: {e}")
            self.object_to_category = {}

        self.pub_yolo_image = self.create_publisher(
            Image, YOLO_DETECTIONS_PUBLISHER_TOPIC, 5
        )

        self.create_service(
            SetTrashCategory, TRASH_SERVICE_CATEGORY, self.set_trash_category
        )
        self.create_service(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV, self.detection_handler
        )
        self.create_service(YoloDetect, YOLO_DETECTION_TOPIC, self.yolo_detect)

    def box_color(self, det):
        return (0, 0, 255) if det.label_text.startswith("trash/") else (0, 255, 0)

    # ------------------------------------------------------------------ services

    def set_trash_category(self, req, res):
        if not req.category:
            res.success = False
            self.get_logger().warn("No category in SetTrashCategory request")
            return res
        self.category = req.category.lower()
        res.success = True
        self.get_logger().info(f"Trash category set to: {self.category}")
        return res

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
            f"DetectionHandler: label='{request.label}' returned={len(detections)}"
        )
        return response

    def yolo_detect(self, request, response):
        if self.latest_frame is None:
            self.get_logger().warn("YoloDetect called but no frame received yet")
            response.success = False
            response.detections = []
            return response

        classes = list(request.classes) if request.classes else None
        raw = self.yolo_service_model.detect(self.latest_frame)
        if classes is not None:
            raw = [d for d in raw if d.class_id_ in classes]

        h, w = self.latest_frame.shape[:2]
        annotated = self.latest_frame.copy()
        ros_dets = []
        for d in raw:
            det = Detection()
            det.x1 = int(d.bbox_.x1 * w)
            det.y1 = int(d.bbox_.y1 * h)
            det.x2 = int(d.bbox_.x2 * w)
            det.y2 = int(d.bbox_.y2 * h)
            det.confidence = d.confidence_
            det.class_id = d.class_id_
            ros_dets.append(det)
            cv.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), (0, 255, 0), 2)
            cv.putText(
                annotated,
                f"{d.label_}: {d.confidence_:.2f}",
                (det.x1, det.y1 - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        self.pub_yolo_image.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        response.success = True
        response.detections = ros_dets
        return response

    # ------------------------------------------------------------------ inference

    def run(self, frame):
        if self.flip_image:
            frame = cv.rotate(frame, cv.ROTATE_180)
        visual = copy.deepcopy(frame)

        all_2d = []
        for model in self.models:
            all_2d.extend(model.detect(frame))
        merged = iou_deduplicate(all_2d, threshold=0.6)

        if self.flip_image:
            for det in merged:
                det.bbox_.y2, det.bbox_.x2, det.bbox_.y1, det.bbox_.x1 = (
                    1 - det.bbox_.y1,
                    1 - det.bbox_.x1,
                    1 - det.bbox_.y2,
                    1 - det.bbox_.x2,
                )

        all_detections = self.extract_3d(merged)
        all_detections = self.filter_by_distance(all_detections)

        self.publish_3d_markers(all_detections)
        self.publish_poses(all_detections)

        ros_detections = self.to_ros(all_detections)
        for det in ros_detections:
            det.point3d.header.stamp = self.curr_clock

        self.latest_detections = ros_detections

        published = []
        for det in ros_detections:
            d = copy.deepcopy(det)
            cat = self.object_to_category.get(d.label_text.strip().lower())
            if self.category and cat and cat == self.category:
                d.label_text = f"trash/{d.label_text}"
            published.append(d)

        self.pub_detections.publish(ObjectDetectionArray(detections=published))
        self.detections_frame = self.visualize(visual, published)

        if self.verbose:
            for i, d in enumerate(published):
                self.get_logger().info(
                    f"Detection #{str(index)}:   {all_detections[index].__str__()}"
                )

        self.visualize_3d_detections(all_detections)

        self.detections_pose_publisher.publish(
            PoseArray(
                poses=[
                    Pose(
                        position=Point(
                            x=detection.point_stamped_.point.x,
                            y=detection.point_stamped_.point.y,
                            z=detection.point_stamped_.point.z,
                        )
                    )
                    for detection in all_detections
                ]
            )
        )

        merged_detections = self.object_detector_2d.getFridaDetections(all_detections)
        for detection in merged_detections:
            detection.point3d.header.stamp = self.curr_clock

        # Process trash
        processed_detections = []
        for det in merged_detections:
            label = det.label_text.strip().lower()
            detection = copy.deepcopy(det)
            obj_category = self.object_to_category.get(label)
            if self.category and obj_category and obj_category == self.category:
                detection.label_text = f"trash/{det.label_text}"
                self.get_logger().info(f"Detected TRASH/{det.label_text}")
            processed_detections.append(detection)

        self.latest_detections = processed_detections
        self.detections_publisher.publish(
            ObjectDetectionArray(detections=processed_detections)
        )

        self.detections_frame = self.visualize_detections(
            visual_image, processed_detections
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObjectDetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

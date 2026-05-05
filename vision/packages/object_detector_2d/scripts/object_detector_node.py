#! /usr/bin/env python3
import copy
import json
import math
import os
import threading

import cv2 as cv
import rclpy
import rclpy.duration
import rclpy.node
import rclpy.qos
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DETECTION_HANDLER_TOPIC_SRV,
    DETECTIONS_3D_TOPIC,
    DETECTIONS_ACTIVE_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
    DETECTIONS_POSES_TOPIC,
    DETECTIONS_TOPIC,
    IMAGE_ORIENTED_TOPIC,
    TRASH_SERVICE_CATEGORY,
    YOLO_DETECTION_TOPIC,
)
from frida_interfaces.msg import Detection, ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DetectionHandler, SetTrashCategory, YoloDetect
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker, MarkerArray
import models  # noqa: F401
from models.registry import ModelRegistry
from models.utils import iou_deduplicate
from vision_3D_utils import deproject_pixel_to_point, get2DCentroid, get_depth


class ObjectDetectorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("object_detector_2D_node")

        # --- parameters ---
        def _p(name, default):
            return self.declare_parameter(name, default).get_parameter_value()

        model_names = _p("models", ["yolo_v26_finetuned"]).string_array_value
        self.depth_active = _p("DEPTH_ACTIVE", True).bool_value
        self.camera_frame = _p("CAMERA_FRAME", CAMERA_FRAME).string_value
        self.target_frame = _p("TARGET_FRAME", "base_link").string_value
        self.max_depth = _p("MAX_DEPTH_THRESH", 2.0).double_value
        self.use_zed_transform = _p("USE_ZED_TRANSFORM", True).bool_value
        self.flip_image = _p("FLIP_IMAGE", False).bool_value
        self.verbose = _p("VERBOSE", False).bool_value
        use_active_flag = _p("USE_ACTIVE_FLAG", False).bool_value
        self.active_flag = not use_active_flag

        rgb_topic = _p("RGB_IMAGE_TOPIC", IMAGE_ORIENTED_TOPIC).string_value
        depth_topic = _p("DEPTH_IMAGE_TOPIC", DEPTH_IMAGE_TOPIC).string_value
        info_topic = _p("CAMERA_INFO_TOPIC", CAMERA_INFO_TOPIC).string_value
        det_topic = _p("DETECTIONS_TOPIC", DETECTIONS_TOPIC).string_value
        det_img_topic = _p(
            "DETECTIONS_IMAGE_TOPIC", DETECTIONS_IMAGE_TOPIC
        ).string_value
        det_poses_topic = _p(
            "DETECTIONS_POSES_TOPIC", DETECTIONS_POSES_TOPIC
        ).string_value
        det_3d_topic = _p("DETECTIONS_3D_TOPIC", DETECTIONS_3D_TOPIC).string_value
        active_topic = _p(
            "DETECTIONS_ACTIVE_TOPIC", DETECTIONS_ACTIVE_TOPIC
        ).string_value

        # --- models ---
        self.models = [ModelRegistry.get(name) for name in model_names]
        self.get_logger().info(f"Loaded models: {[m.name for m in self.models]}")
        self.yolo_service_model = ModelRegistry.get("yolo_generic")
        self.get_logger().info(f"YOLO service model: {self.yolo_service_model.name}")

        # --- state ---
        self.bridge = CvBridge()
        self.latest_frame = None
        self.depth_image = []
        self.camera_info = None
        self.curr_clock = None
        self.detections_frame = []
        self.latest_detections = []
        self._run_thread = None
        self._frame_count = 0
        self._skip_frames = 2

        # trash category + object→category map
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

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # --- publishers ---
        self.pub_detections = self.create_publisher(ObjectDetectionArray, det_topic, 5)
        self.pub_poses = self.create_publisher(PoseArray, det_poses_topic, 5)
        self.pub_3d = self.create_publisher(MarkerArray, det_3d_topic, 5)
        self.pub_image = self.create_publisher(Image, det_img_topic, 5)

        # --- subscriptions ---
        qos = rclpy.qos.QoSProfile(
            depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )
        self.create_subscription(Image, rgb_topic, self._rgb_cb, qos)
        if self.depth_active:
            self.create_subscription(Image, depth_topic, self._depth_cb, qos)
            self.create_subscription(CameraInfo, info_topic, self._info_cb, qos)
        # honour both the legacy global topic and the per-node active topic
        self.create_subscription(
            Bool, "/vision/object_detector/active", self._active_cb, 10
        )
        if use_active_flag:
            self.create_subscription(Bool, active_topic, self._active_cb, 5)

        # --- services ---
        self.create_service(
            SetTrashCategory, TRASH_SERVICE_CATEGORY, self._set_trash_category
        )
        self.create_service(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV, self._detection_handler
        )
        self.create_service(YoloDetect, YOLO_DETECTION_TOPIC, self._yolo_detect)

    # ------------------------------------------------------------------ callbacks

    def _active_cb(self, msg):
        self.active_flag = msg.data

    def _depth_cb(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError:
            pass

    def _info_cb(self, data):
        if self.camera_info is None:
            self.camera_info = data

    def _rgb_cb(self, data):
        try:
            rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return
        self.latest_frame = rgb
        self.curr_clock = data.header.stamp
        self._frame_count += 1

        if not self.active_flag:
            self.detections_frame = rgb
        elif self._frame_count % (self._skip_frames + 1) == 0 and (
            self._run_thread is None or not self._run_thread.is_alive()
        ):
            frame = copy.deepcopy(rgb)
            self._run_thread = threading.Thread(
                target=self._run, args=(frame,), daemon=True
            )
            self._run_thread.start()

        if len(self.detections_frame) > 0:
            self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8")
            )

    # ------------------------------------------------------------------ services

    def _set_trash_category(self, req, res):
        if not req.category:
            res.success = False
            self.get_logger().warn("No category in SetTrashCategory request")
            return res
        self.category = req.category.lower()
        res.success = True
        self.get_logger().info(f"Trash category set to: {self.category}")
        return res

    def _detection_handler(self, request, response):
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

    def _yolo_detect(self, request, response):
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

        response.success = True
        response.detections = ros_dets
        return response

    # ------------------------------------------------------------------ inference

    def _run(self, frame):
        if self.flip_image:
            frame = cv.rotate(frame, cv.ROTATE_180)
        visual = copy.deepcopy(frame)

        # Run all models and merge
        all_2d = []
        for model in self.models:
            all_2d.extend(model.detect(frame))

        merged = iou_deduplicate(all_2d, threshold=0.6)

        # Unflip bounding boxes to original image coordinates
        if self.flip_image:
            for det in merged:
                det.bbox_.y2, det.bbox_.x2, det.bbox_.y1, det.bbox_.x1 = (
                    1 - det.bbox_.y1,
                    1 - det.bbox_.x1,
                    1 - det.bbox_.y2,
                    1 - det.bbox_.x2,
                )

        # 3D projection
        all_detections = self._extract_3d(merged)

        # Distance filter (passes 0,0,0 points when depth is inactive)
        all_detections = [
            d
            for d in all_detections
            if math.sqrt(
                d.point_stamped_.point.x**2
                + d.point_stamped_.point.y**2
                + d.point_stamped_.point.z**2
            )
            <= self.max_depth
        ]

        # 3D markers + poses
        self._publish_3d_markers(all_detections)
        self.pub_poses.publish(
            PoseArray(
                poses=[
                    Pose(
                        position=Point(
                            x=d.point_stamped_.point.x,
                            y=d.point_stamped_.point.y,
                            z=d.point_stamped_.point.z,
                        )
                    )
                    for d in all_detections
                ]
            )
        )

        # Convert to ROS messages
        ros_detections = self._to_ros(all_detections)
        for det in ros_detections:
            det.point3d.header.stamp = self.curr_clock

        # Store without trash labels (service queries use original labels)
        self.latest_detections = ros_detections

        # Apply trash labels to deep copies for publishing + visualization
        published = []
        for det in ros_detections:
            d = copy.deepcopy(det)
            cat = self.object_to_category.get(d.label_text.strip().lower())
            if self.category and cat and cat == self.category:
                d.label_text = f"trash/{d.label_text}"
            published.append(d)

        self.pub_detections.publish(ObjectDetectionArray(detections=published))
        self.detections_frame = self._visualize(visual, published)

        if self.verbose:
            for i, d in enumerate(published):
                self.get_logger().info(
                    f"Detection #{i}: {d.label_text} ({d.score:.2f})"
                )

    # ------------------------------------------------------------------ helpers

    def _extract_3d(self, detections):
        has_depth = (
            self.depth_active
            and len(self.depth_image) > 0
            and self.camera_info is not None
        )
        if has_depth and not self.use_zed_transform:
            while not self.tfBuffer.can_transform(
                self.camera_frame, self.target_frame, rclpy.time.Time().to_msg()
            ):
                pass
        for det in detections:
            pt = PointStamped(header=Header(frame_id=self.camera_frame), point=Point())
            if has_depth:
                p2d = get2DCentroid(
                    [det.bbox_.y1, det.bbox_.x1, det.bbox_.y2, det.bbox_.x2],
                    self.depth_image,
                )
                depth = get_depth(self.depth_image, p2d)
                p3d = deproject_pixel_to_point(self.camera_info, p2d, depth)
                pt.point.x, pt.point.y, pt.point.z = (
                    float(p3d[0]),
                    float(p3d[1]),
                    float(p3d[2]),
                )
            det.point_stamped_ = pt
        return detections

    def _publish_3d_markers(self, detections):
        markers = MarkerArray()
        for i, det in enumerate(detections):
            m = Marker()
            m.header.frame_id = self.camera_frame
            m.header.stamp = self.curr_clock
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.id = i
            m.pose.position.x = det.point_stamped_.point.x
            m.pose.position.y = det.point_stamped_.point.y
            m.pose.position.z = det.point_stamped_.point.z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.a = 1.0
            m.color.g = 1.0
            m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            markers.markers.append(m)
        self.pub_3d.publish(markers)

    def _to_ros(self, detections) -> list:
        result = []
        for det in detections:
            result.append(
                ObjectDetection(
                    label=det.class_id_,
                    label_text=det.label_,
                    score=det.confidence_,
                    ymin=float(min(det.bbox_.y1, det.bbox_.y2)),
                    xmin=float(min(det.bbox_.x1, det.bbox_.x2)),
                    ymax=float(max(det.bbox_.y1, det.bbox_.y2)),
                    xmax=float(max(det.bbox_.x1, det.bbox_.x2)),
                    point3d=det.point_stamped_,
                )
            )
        return result

    def _visualize(self, image, detections) -> object:
        for det in detections:
            h, w = image.shape[:2]
            left, right = int(det.xmin * w), int(det.xmax * w)
            top, bottom = int(det.ymin * h), int(det.ymax * h)
            color = (0, 0, 255) if det.label_text.startswith("trash/") else (0, 255, 0)
            cv.rectangle(image, (left, top), (right, bottom), color, 7)
            cv.putText(
                image,
                f"{det.label_text}: {det.score:.2f}",
                (left, top - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                2,
                color,
                2,
            )
            cv.circle(
                image, ((left + right) // 2, (top + bottom) // 2), 5, (0, 0, 255), -1
            )
        return image


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObjectDetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#! /usr/bin/env python3
"""Base class for detector nodes: shared params, subscriptions, publishers, 3D projection, and visualization."""

import copy
import math
import threading
from abc import ABC, abstractmethod

import cv2 as cv
import rclpy
import rclpy.duration
import rclpy.node
import rclpy.qos
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    DEPTH_IMAGE_TOPIC,
    IMAGE_ORIENTED_TOPIC,
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker, MarkerArray
import models  # noqa: F401
from vision_3D_utils import deproject_pixel_to_point, get2DCentroid, get_depth


class BaseDetectorNode(rclpy.node.Node, ABC):
    MARKER_COLOR = (0.0, 1.0, 0.0)  # (r, g, b) — subclasses can override

    def __init__(
        self,
        node_name: str,
        *,
        default_det_topic: str,
        default_det_img_topic: str,
        default_det_poses_topic: str,
        default_det_3d_topic: str,
        default_active_topic: str,
        fixed_active_topic: str,
    ):
        super().__init__(node_name)

        def p(name, default):
            return self.declare_parameter(name, default).get_parameter_value()

        self.depth_active = p("DEPTH_ACTIVE", True).bool_value
        self.camera_frame = p("CAMERA_FRAME", CAMERA_FRAME).string_value
        self.target_frame = p("TARGET_FRAME", "base_link").string_value
        self.max_depth = p("MAX_DEPTH_THRESH", 2.0).double_value
        self.use_zed_transform = p("USE_ZED_TRANSFORM", True).bool_value
        self.flip_image = p("FLIP_IMAGE", False).bool_value
        self.verbose = p("VERBOSE", False).bool_value
        use_active_flag = p("USE_ACTIVE_FLAG", False).bool_value
        self.active_flag = not use_active_flag

        rgb_topic = p("RGB_IMAGE_TOPIC", IMAGE_ORIENTED_TOPIC).string_value
        depth_topic = p("DEPTH_IMAGE_TOPIC", DEPTH_IMAGE_TOPIC).string_value
        info_topic = p("CAMERA_INFO_TOPIC", CAMERA_INFO_TOPIC).string_value
        det_topic = p("DETECTIONS_TOPIC", default_det_topic).string_value
        det_img_topic = p("DETECTIONS_IMAGE_TOPIC", default_det_img_topic).string_value
        det_poses_topic = p(
            "DETECTIONS_POSES_TOPIC", default_det_poses_topic
        ).string_value
        det_3d_topic = p("DETECTIONS_3D_TOPIC", default_det_3d_topic).string_value
        active_topic = p("DETECTIONS_ACTIVE_TOPIC", default_active_topic).string_value

        # --- state ---
        self.bridge = CvBridge()
        self.latest_frame = None
        self.depth_image = []
        self.camera_info = None
        self.curr_clock = None
        self.detections_frame = []
        self.latest_detections = []
        self.run_thread = None
        self.frame_count = 0
        self.skip_frames = 2

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
        self.create_subscription(Image, rgb_topic, self.rgb_cb, qos)
        if self.depth_active:
            self.create_subscription(Image, depth_topic, self.depth_cb, qos)
            self.create_subscription(CameraInfo, info_topic, self.info_cb, qos)
        self.create_subscription(Bool, fixed_active_topic, self.active_cb, 10)
        if use_active_flag:
            self.create_subscription(Bool, active_topic, self.active_cb, 5)

    # ------------------------------------------------------------------ callbacks

    def active_cb(self, msg):
        self.active_flag = msg.data

    def depth_cb(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError:
            pass

    def info_cb(self, data):
        if self.camera_info is None:
            self.camera_info = data

    def rgb_cb(self, data):
        try:
            rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return
        self.latest_frame = rgb
        self.curr_clock = data.header.stamp
        self.frame_count += 1

        if not self.active_flag:
            self.detections_frame = rgb
        elif self.frame_count % (self.skip_frames + 1) == 0 and (
            self.run_thread is None or not self.run_thread.is_alive()
        ):
            frame = copy.deepcopy(rgb)
            self.run_thread = threading.Thread(
                target=self.run, args=(frame,), daemon=True
            )
            self.run_thread.start()

        if len(self.detections_frame) > 0:
            self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8")
            )

    # ------------------------------------------------------------------ inference

    @abstractmethod
    def run(self, frame): ...

    # ------------------------------------------------------------------ helpers

    def extract_3d(self, detections):
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

    def filter_by_distance(self, detections):
        return [
            d
            for d in detections
            if math.sqrt(
                d.point_stamped_.point.x**2
                + d.point_stamped_.point.y**2
                + d.point_stamped_.point.z**2
            )
            <= self.max_depth
        ]

    def publish_3d_markers(self, detections):
        r, g, b = self.MARKER_COLOR
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
            m.color.r, m.color.g, m.color.b = r, g, b
            m.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            markers.markers.append(m)
        self.pub_3d.publish(markers)

    def publish_poses(self, detections):
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
                    for d in detections
                ]
            )
        )

    def to_ros(self, detections) -> list:
        return [
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
            for det in detections
        ]

    def box_color(self, det):
        return (0, 255, 0)

    def visualize(self, image, detections):
        for det in detections:
            h, w = image.shape[:2]
            left, right = int(det.xmin * w), int(det.xmax * w)
            top, bottom = int(det.ymin * h), int(det.ymax * h)
            color = self.box_color(det)
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
            cv.circle(image, ((left + right) // 2, (top + bottom) // 2), 5, color, -1)
        return image

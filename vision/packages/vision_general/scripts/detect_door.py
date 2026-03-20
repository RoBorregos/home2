#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from frida_interfaces.srv import DetectDoor
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
)
from vision_general.utils.calculations import (
    get_depth,
    deproject_pixel_to_point,
)
from cv_bridge import CvBridge
import math
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import cv2
import numpy as np
import os


DOOR_MODEL_PATH = 'src/vision/packages/vision_general/vision_general/utils/models/door.pt'

# Detection parameters
HANDLE_HISTORY_SIZE = 5
HANDLE_CONSISTENCY_THRESHOLD = 3
HANDLE_SPATIAL_TOLERANCE = 0.08
DETECTION_SCANS = 5


def point_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


def average_points(points):
    avg = Point()
    avg.x = sum(p.x for p in points) / len(points)
    avg.y = sum(p.y for p in points) / len(points)
    avg.z = sum(p.z for p in points) / len(points)
    return avg


class DoorDetectionService(Node):
    def __init__(self):
        super().__init__('door_detection_service')

        # Camera subscribers
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        self.rgb_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self._rgb_cb, 10
        )
        self.depth_sub = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._depth_cb, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10
        )

        # TF for camera->base transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load YOLO door model
        model_path = os.path.realpath(DOOR_MODEL_PATH)
        self.get_logger().info(f'Loading door model from: {model_path}')
        self.model = YOLO(model_path)

        # Service
        self.srv = self.create_service(
            DetectDoor, '/vision/detect_door', self.detect_door_callback
        )

        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, '/vision/door_detections', 10)
        self.create_timer(0.5, self._publish_debug_image)

        self.get_logger().info('Door detection service ready at /vision/detect_door')

    # ── Camera callbacks ──────────────────────────────────────────────

    def _rgb_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _info_cb(self, msg):
        self.camera_info = msg

    # ── Helpers ───────────────────────────────────────────────────────

    def _pixel_to_base_point(self, cx, cy):
        """Convert pixel coords to 3D point in base frame."""
        depth = get_depth(self.depth_image, (cx, cy))
        if math.isnan(depth):
            return None

        point_3d = deproject_pixel_to_point(self.camera_info, (cx, cy), depth)

        point_stamped = PointStamped()
        point_stamped.header.frame_id = CAMERA_FRAME
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point.x = float(point_3d[0])
        point_stamped.point.y = float(point_3d[1])
        point_stamped.point.z = float(point_3d[2])

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base', CAMERA_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            point_base = do_transform_point(point_stamped, transform)
            return point_base.point
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None

    def _run_single_detection(self):
        """Run one YOLO inference. Returns (handle_point, axis_point) in base frame."""
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return None, None

        results = self.model.predict(self.rgb_image, verbose=False, conf=0.3)

        handle_box = None
        handle_conf = 0.0
        axis_box = None
        axis_conf = 0.0

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                label = self.model.names[cls_id].lower()
                self.get_logger().info(f'Detected "{label}" conf={conf:.2f}')

                if ('handle' in label or 'handler' in label) and conf > handle_conf:
                    handle_conf = conf
                    handle_box = box
                elif ('axis' in label or 'hinge' in label) and conf > axis_conf:
                    axis_conf = conf
                    axis_box = box

        handle_point = None
        axis_point = None

        if handle_box is not None:
            x1, y1, x2, y2 = handle_box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            handle_point = self._pixel_to_base_point(cx, cy)

        if axis_box is not None:
            x1, y1, x2, y2 = axis_box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            axis_point = self._pixel_to_base_point(cx, cy)

        return handle_point, axis_point

    # ── Debug visualization ─────────────────────────────────────────

    def _publish_debug_image(self):
        if self.rgb_image is None:
            return

        debug_img = self.rgb_image.copy()
        results = self.model.predict(debug_img, verbose=False, conf=0.3)

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                label = self.model.names[cls_id]

                if 'handle' in label.lower() or 'handler' in label.lower():
                    color = (0, 255, 0)  # green
                elif 'axis' in label.lower() or 'hinge' in label.lower():
                    color = (255, 0, 0)  # blue
                else:
                    color = (0, 255, 255)  # yellow

                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug_img, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))

    # ── Service callback ──────────────────────────────────────────────

    def detect_door_callback(self, request, response):
        self.get_logger().info('Door detection request received')

        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            self.get_logger().error('No camera data available')
            response.success = False
            response.handle_detected = False
            response.axis_detected = False
            return response

        handle_history = []
        axis_pos = None
        axis_seen = False

        for i in range(DETECTION_SCANS):
            rclpy.spin_once(self, timeout_sec=0.3)
            handle_pt, axis_pt = self._run_single_detection()

            if handle_pt is not None:
                handle_history.append(handle_pt)

            if axis_pt is not None and not axis_seen:
                axis_pos = axis_pt
                axis_seen = True
                self.get_logger().info(
                    f'Axis detected at ({axis_pos.x:.3f}, {axis_pos.y:.3f}, {axis_pos.z:.3f})'
                )

            time.sleep(0.2)

        # Find consistent handle position
        handle_pos = None

        if handle_history:
            best_cluster = []
            for i, p in enumerate(handle_history):
                cluster = [p]
                for j, q in enumerate(handle_history):
                    if i != j and point_distance(p, q) < HANDLE_SPATIAL_TOLERANCE:
                        cluster.append(q)
                if len(cluster) > len(best_cluster):
                    best_cluster = cluster

            if len(best_cluster) >= HANDLE_CONSISTENCY_THRESHOLD:
                handle_pos = average_points(best_cluster)
                self.get_logger().info(
                    f'Handle CONFIRMED ({len(best_cluster)}/{len(handle_history)} consistent) '
                    f'at ({handle_pos.x:.3f}, {handle_pos.y:.3f}, {handle_pos.z:.3f})'
                )
            else:
                handle_pos = average_points(best_cluster)
                self.get_logger().warn('Handle detected but not fully confirmed — using best estimate.')

        # Build response
        response.success = handle_pos is not None
        response.handle_detected = handle_pos is not None
        response.axis_detected = axis_seen

        if handle_pos is not None:
            response.handle_position = handle_pos
        if axis_seen:
            response.axis_position = axis_pos

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DoorDetectionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

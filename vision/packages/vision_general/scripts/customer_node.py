#!/usr/bin/env python3

"""
Node to detect customers who are sitting and raising their hand.
Uses YOLO pose on the full image for person detection + keypoints in one pass.
"""

import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
from vision_general.utils.calculations import (
    get2DCentroid,
    point2d_to_ros_point_stamped,
)

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from builtin_interfaces.msg import Time
from frida_interfaces.srv import CropQuery, Customer
from frida_interfaces.msg import PersonList, Person
from vision_general.utils.ros_utils import wait_for_future
from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOPIC,
    CROP_QUERY_TOPIC,
    CUSTOMER,
    GET_CUSTOMER_TOPIC,
)

CONF_THRESHOLD = 0.4
CAMERA_FRAME = "zed_left_camera_optical_frame"
SAVE_MIN_INTERVAL = 2.0  # s between saved run images (avoid spamming during sweeps)


class CustomerNode(Node):
    def __init__(self):
        super().__init__("customer_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # Reject detections farther than this (public/recording zone at the
        # venue perimeter). Tune on site with `ros2 param set`.
        self.declare_parameter("max_customer_range", 5.0)
        # Extra moondream check on waving candidates: filters people raising a
        # phone/camera to record (looks identical to waving in keypoints).
        self.declare_parameter("verify_calling_with_moondream", True)
        # Run images (confirmed callers) land here; the repo is volume-mounted
        # at /workspace/src in the vision container, so they survive on the host.
        self.declare_parameter("save_dir", "/workspace/src/restaurant_runs")

        self.save_dir = self.get_parameter("save_dir").value
        try:
            os.makedirs(self.save_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().warn(f"Cannot create save dir {self.save_dir}: {e}")
            self.save_dir = None
        self._last_save_time = 0.0

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )

        self.get_customer_service = self.create_service(
            Customer,
            GET_CUSTOMER_TOPIC,
            self.get_customer_callback,
            callback_group=self.callback_group,
        )

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)
        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)
        self.customer_publisher = self.create_publisher(Image, CUSTOMER, 10)
        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOPIC, 10)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        self.image = None
        self.depth_image = None
        self.depth_image_time = None
        self.imageInfo = None
        self.output_image = []
        self.customer_image = []

        self.pose_detection = PoseDetection()

        self.create_timer(0.1, self.publish_image)
        self.get_logger().info("Customer Node Ready")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image_time = data.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def publish_image(self):
        """Publish the image to the camera topic"""
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

        if len(self.customer_image) != 0:
            h, w = self.customer_image.shape[:2]
            square_size = max(h, w)
            square_img = cv2.copyMakeBorder(
                self.customer_image,
                top=(square_size - h) // 2,
                bottom=(square_size - h + 1) // 2,
                left=(square_size - w) // 2,
                right=(square_size - w + 1) // 2,
                borderType=cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )
            self.customer_publisher.publish(
                self.bridge.cv2_to_imgmsg(square_img, "bgr8")
            )

    def get_customer_callback(self, req, res):
        res.found = False
        res.people = PersonList()
        res.people.list = []
        # Table scan sets include_non_waving=True to map seated customers who are no
        # longer waving; the WAIT_FOR_CALL search leaves it False (waving only).
        include_non_waving = getattr(req, "include_non_waving", False)

        # An exception here would drop the response and make the client wait
        # out its full timeout — refuse early while camera data is missing.
        if self.image is None or self.depth_image is None or self.imageInfo is None:
            self.get_logger().warn("Camera image/depth/info not available yet")
            return res

        max_range = float(self.get_parameter("max_customer_range").value)
        verify_calling = bool(self.get_parameter("verify_calling_with_moondream").value)

        frame = self.image.copy()
        depth_image = self.depth_image
        self.output_image = frame.copy()
        image_width = frame.shape[1]

        people = self.pose_detection.detect_people(frame, conf=CONF_THRESHOLD)
        self.get_logger().info(f"Detected {len(people)} people")

        confirmed_crops = []
        for person_data in people:
            x1, y1, x2, y2 = person_data["bbox"]
            points = person_data["keypoints"]
            kp_conf = person_data["kp_conf"]

            # Expand bbox by 20% for moondream and debug image
            h, w = frame.shape[:2]
            bw, bh = x2 - x1, y2 - y1
            pad_x, pad_y = int(bw * 0.1), int(bh * 0.1)
            ex1 = max(0, x1 - pad_x)
            ey1 = max(0, y1 - pad_y)
            ex2 = min(w, x2 + pad_x)
            ey2 = min(h, y2 + pad_y)

            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            self.pose_detection.draw_landmarks(self.output_image, points, kp_conf)

            raising = self.pose_detection.is_waving_from_keypoints(points, kp_conf)
            if not include_non_waving and not raising:
                self.get_logger().info("Customer not raising hand")
                continue

            point2D = get2DCentroid((y1, x1, y2, x2), depth_image)
            coords = point2d_to_ros_point_stamped(
                self.imageInfo,
                depth_image,
                point2D,
                CAMERA_FRAME,
                Time(sec=0, nanosec=0),
            )
            distance = math.sqrt(
                coords.point.x**2 + coords.point.y**2 + coords.point.z**2
            )
            if not math.isfinite(distance) or distance < 0.1:
                self.get_logger().info("Candidate has invalid depth — skipping")
                continue
            if distance > max_range:
                # Public / recording zone at the perimeter: too far to be a
                # customer we can serve.
                self.get_logger().info(
                    f"Candidate at {distance:.1f} m > {max_range:.1f} m — ignoring"
                )
                continue

            sitting = self.pose_detection.is_sitting_from_keypoints(points, kp_conf)
            if not sitting:
                self.get_logger().info("Checking sitting with moondream")
                sitting = self.is_sitting_moondream([ex1, ey1, ex2, ey2])

            # WAIT_FOR_CALL: require waving + sitting. Table scan: any seated person.
            if not (sitting and (raising or include_non_waving)):
                continue

            # A raised phone/camera also puts a wrist above the shoulder —
            # arbitrate real "calling a waiter" waves with moondream.
            if raising and not include_non_waving and verify_calling:
                if not self.is_calling_moondream([ex1, ey1, ex2, ey2]):
                    self.get_logger().info(
                        "Moondream says candidate is not calling (e.g. recording) — ignoring"
                    )
                    continue

            self.get_logger().info(
                "Customer detected (sitting%s)" % (", raising" if raising else "")
            )
            # Mark every confirmed caller in the full frame (scored: "detect
            # calling or waving customer" — referees see all of them at once).
            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                self.output_image,
                f"CUSTOMER {len(confirmed_crops) + 1}"
                + (" CALLING" if raising else ""),
                (x1, max(y1 - 10, 25)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )
            confirmed_crops.append(frame[ey1:ey2, ex1:ex2].copy())

            self.results_publisher.publish(coords)

            pt_x_norm = float(point2D[1]) / (frame.shape[1] / 2) - 1.0
            self.centroid_publisher.publish(Point(x=pt_x_norm, y=0.0, z=0.0))

            person = Person()
            person.x = (x1 + x2) // 2
            person.y = (y1 + y2) // 2
            person.point3d = coords
            diff = person.x - (image_width / 2)
            max_degree = 50.0
            angle = diff * max_degree / (image_width / 2)
            person.angle = angle
            res.people.list.append(person)

            res.found = True
            self.get_logger().info(
                f"Customer position (3D): x={coords.point.x:.3f}  y={coords.point.y:.3f}  z={coords.point.z:.3f}, angle={angle:.1f}"
            )

        # Show ALL confirmed callers side by side on the display crop topic,
        # not just the last one.
        if confirmed_crops:
            self.customer_image = self._compose_crops(confirmed_crops)
            if not include_non_waving:
                self.save_run_images(self.output_image, self.customer_image)

        self.get_logger().info(f"Customers detected: {len(res.people.list)}")
        return res

    def _compose_crops(self, crops, height=360, gap=8):
        """Horizontal composite of all confirmed customer crops (same height,
        black gaps) so the display shows every caller at once."""
        resized = []
        for crop in crops:
            h, w = crop.shape[:2]
            if h == 0 or w == 0:
                continue
            scale = height / float(h)
            resized.append(cv2.resize(crop, (max(1, int(w * scale)), height)))
        if not resized:
            return []
        if len(resized) == 1:
            return resized[0]
        spacer = np.zeros((height, gap, 3), dtype=resized[0].dtype)
        parts = []
        for i, img in enumerate(resized):
            if i:
                parts.append(spacer)
            parts.append(img)
        return cv2.hconcat(parts)

    def save_run_images(self, annotated_frame, crop):
        """Save the annotated frame + caller crop for post-run review and as
        referee evidence of the detected caller. Rate-limited."""
        if self.save_dir is None:
            return
        now = time.time()
        if now - self._last_save_time < SAVE_MIN_INTERVAL:
            return
        self._last_save_time = now
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        try:
            cv2.imwrite(
                os.path.join(self.save_dir, f"caller_{stamp}_frame.jpg"),
                annotated_frame,
            )
            if crop is not None and len(crop) != 0:
                cv2.imwrite(
                    os.path.join(self.save_dir, f"caller_{stamp}_crop.jpg"), crop
                )
        except Exception as e:
            self.get_logger().warn(f"Could not save run images: {e}")

    def is_calling_moondream(self, bbox):
        """Ask moondream whether the person is genuinely calling a waiter (vs.
        holding up a phone/camera, which fools the keypoint waving check)."""
        return self._crop_query_yes(
            bbox,
            "Is this person raising a hand or arm to call a waiter? "
            "If they are holding a phone or camera up to record, answer no. "
            "Answer only with yes or no.",
        )

    def is_sitting_moondream(self, bbox):
        return self._crop_query_yes(
            bbox, "Is the person in this image sitting? Answer only with yes or no."
        )

    def _crop_query_yes(self, bbox, query):
        """Run a moondream yes/no query on a bbox crop. False on any failure."""
        if self.image is None or bbox is None:
            return False

        if not self.moondream_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("Moondream crop query service not available")
            return False

        frame = self.image.copy()
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        if width <= 0 or height <= 0:
            return False

        xmin = max(0.0, min(1.0, float(x1) / float(width)))
        ymin = max(0.0, min(1.0, float(y1) / float(height)))
        xmax = max(0.0, min(1.0, float(x2) / float(width)))
        ymax = max(0.0, min(1.0, float(y2) / float(height)))

        request = CropQuery.Request()
        request.xmin = xmin
        request.ymin = ymin
        request.xmax = xmax
        request.ymax = ymax
        request.query = query

        future = self.moondream_client.call_async(request)
        wait_for_future(future)

        if not future.done():
            self.get_logger().warn("Service call timed out")
            return False

        result = future.result()
        if result is None or not result.success:
            return False

        if future.exception() is not None:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return False

        answer = result.result.strip().lower()
        if answer.startswith("yes"):
            return True
        if answer.startswith("no"):
            return False

        self.get_logger().warn(
            f"Unexpected answer from Moondream crop query: '{answer}'. Expected 'yes' or 'no'."
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CustomerNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

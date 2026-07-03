#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for HRIC commands.
"""

import cv2
import json
import numpy as np
import queue
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from rclpy.task import Future
from vision_general.utils.trt_utils import load_yolo_trt
from std_msgs.msg import Int16

from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import (
    ChairsToRemove,
    DetectHand,
    FindSeat,
    MoondreamDetection,
    YoloDetect,
    MapAreas,
)
from vision_general.utils.calculations import point2d_to_ros_point_stamped
from frida_constants.navigation_constants import AREAS_SERVICE
from frida_constants.vision_constants import (
    CAMERA_ROTATION_TOPIC,
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    CHAIR_REMOVAL_IMAGE_TOPIC,
    CHAIRS_TO_REMOVE_SERVICE,
    CHECK_PERSON_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DETECT_HAND_SERVICE,
    FIND_SEAT_TOPIC,
    IMAGE_ORIENTED_TOPIC,
    IMAGE_TOPIC_HRIC,
    MOONDREAM_DETECTION_TOPIC,
    YOLO_DETECTION_TOPIC,
)
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from vision_general.utils.area_check import filter_detections_in_house
from vision_general.utils.debug_pub import DebugImagePublisher

package_share_dir = get_package_share_directory("vision_general")

# YOLO COCO keypoint indices for wrist (hand proxy)
LEFT_WRIST_IDX = 9
RIGHT_WRIST_IDX = 10
KP_CONF = 0.3
# Wrist selection: reject deprojected points closer than this (invalid depth),
# and treat wrists within this depth difference as a tie (pick the higher one).
HAND_MIN_DEPTH = 0.15
HAND_DEPTH_TIE_M = 0.15


def _load_yolo_pose(model_name="yolo11m-pose.pt"):
    return load_yolo_trt(model_name, task="pose")


PERCENTAGE = 0.3
MAX_DEGREE = 50
AREA_PERCENTAGE_THRESHOLD = 0.01
CONF_THRESHOLD = 0.4
CHECK_TIMEOUT = 5


class HRICCommands(Node):
    def __init__(self):
        super().__init__("HRIC_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self._img_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image,
            IMAGE_ORIENTED_TOPIC,
            self.image_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            self._img_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Int16,
            CAMERA_ROTATION_TOPIC,
            self._rotation_callback,
            10,
            callback_group=self.callback_group,
        )

        self.find_seat_service = self.create_service(
            FindSeat,
            FIND_SEAT_TOPIC,
            self.find_seat_callback,
            callback_group=self.callback_group,
        )
        self.image_publisher = DebugImagePublisher(
            self, IMAGE_TOPIC_HRIC, "hric_commands", callback_group=self.callback_group
        )
        self.person_detection_action_server = ActionServer(
            self,
            DetectPerson,
            CHECK_PERSON_TOPIC,
            self.detect_person_callback,
            callback_group=self.callback_group,
        )

        self.yolo_client = self.create_client(
            YoloDetect, YOLO_DETECTION_TOPIC, callback_group=self.callback_group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("YOLO service not available, waiting...")

        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.output_image = []
        self.check = False
        self.rotation = 0

        # Areas of the active map. Fetched asynchronously by _poll_areas:
        # blocking on the service future inside a callback can never complete
        # on a single-threaded executor (see fetch_map_areas docstring), and
        # the 2x5s timeouts pushed FindSeat past the task manager's deadline.
        self.areas = None
        self.areas_client = self.create_client(
            MapAreas, AREAS_SERVICE, callback_group=self.callback_group
        )
        self._areas_future = None
        self._areas_timer = self.create_timer(
            2.0, self._poll_areas, callback_group=self.callback_group
        )

        # YOLO pose replaces mediapipe Hands — wrist keypoints as hand proxy
        self.pose_model = _load_yolo_pose("yolo11m-pose.pt")

        self.detect_hand_service = self.create_service(
            DetectHand,
            DETECT_HAND_SERVICE,
            self.detect_hand_callback,
            callback_group=self.callback_group,
        )

        self.moondream_client = self.create_client(
            MoondreamDetection,
            MOONDREAM_DETECTION_TOPIC,
            callback_group=self.callback_group,
        )
        self.chairs_to_remove_service = self.create_service(
            ChairsToRemove,
            CHAIRS_TO_REMOVE_SERVICE,
            self.chairs_to_remove_callback,
            callback_group=self.callback_group,
        )
        self.chair_image_publisher = self.create_publisher(
            Image, CHAIR_REMOVAL_IMAGE_TOPIC, 10, callback_group=self.callback_group
        )
        self.chair_image = None

        self.get_logger().info("HRIC Commands Ready.")

        self.create_timer(0.1, self.publish_image, callback_group=self.callback_group)
        # web_video_server subscribes only after the display switches topic, so
        # keep re-publishing the last chair-removal frame.
        self.create_timer(
            0.5, self.publish_chair_image, callback_group=self.callback_group
        )

    def _rotation_callback(self, msg):
        value = int(msg.data) % 360
        if value != self.rotation:
            self.rotation = value
            self.get_logger().info(f"Camera rotation set to {self.rotation}")

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def run_hand_inference(self):
        """Detect hand position using YOLO pose wrist keypoints (TensorRT accelerated)."""
        if self.image is None:
            return

        frame = self.image
        h, w = frame.shape[:2]
        results = self.pose_model(frame, verbose=False)

        if (
            not results
            or results[0].keypoints is None
            or results[0].keypoints.xy is None
            or len(results[0].keypoints.xy) == 0
        ):
            self.get_logger().info("No hand detected")
            return None

        if self.depth_image is None or self.camera_info is None:
            self.get_logger().warn("Depth image or camera info not available")
            return None

        # The guest interacting with the robot is the LARGEST person in frame,
        # not whichever detection the network happens to list first.
        person_idx = 0
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 1:
            xyxy = boxes.xyxy.cpu().numpy()
            areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
            person_idx = int(areas.argmax())

        points = results[0].keypoints.xy[person_idx].cpu().numpy()  # pixel coords
        conf = (
            results[0].keypoints.conf[person_idx].cpu().numpy()
            if results[0].keypoints.conf is not None
            else np.ones(17, dtype=np.float32)
        )

        # Candidate wrists: confident enough and inside the image.
        candidates = []
        for idx in (LEFT_WRIST_IDX, RIGHT_WRIST_IDX):
            cx, cy = int(points[idx][0]), int(points[idx][1])
            if conf[idx] > KP_CONF and 0 <= cx < w and 0 <= cy < h:
                candidates.append((cx, cy))

        if not candidates:
            self.get_logger().info("No hand detected")
            return None

        # The extended (bag) hand is the wrist NEAREST the robot. Deproject
        # every candidate and keep only valid 3D points — never pick by
        # keypoint confidence: a relaxed hanging hand often scores higher
        # than the extended one.
        valid = []  # (dist, cy, cx, stamped)
        for cx, cy in candidates:
            stamped = point2d_to_ros_point_stamped(
                self.camera_info,
                self.depth_image,
                (cx, cy),
                CAMERA_FRAME,
                Time(sec=0, nanosec=0),
                rotation=self.rotation,
            )
            p = stamped.point
            dist = float(np.sqrt(p.x**2 + p.y**2 + p.z**2))
            if np.isfinite(dist) and dist > HAND_MIN_DEPTH:
                valid.append((dist, cy, cx, stamped))

        annotated = frame.copy()
        for cx, cy in candidates:
            cv2.circle(annotated, (cx, cy), 8, (160, 160, 160), 2)

        if not valid:
            # A depth-invalid point would send the arm toward the camera
            # origin — report no hand and let the task retry.
            self.output_image = annotated
            self.get_logger().warn(
                "Wrist depth invalid for all candidates; no hand point"
            )
            return None

        valid.sort()
        chosen = valid[0]
        if len(valid) > 1 and valid[1][0] - valid[0][0] < HAND_DEPTH_TIE_M:
            # Depths are too close to discriminate (nobody clearly extending a
            # hand forward) — take the HIGHER wrist (raised hand; image y is
            # smaller upward).
            chosen = min(valid, key=lambda c: c[1])

        dist, cy, cx, stamped = chosen
        cv2.circle(annotated, (cx, cy), 10, (0, 255, 0), 3)
        cv2.putText(
            annotated,
            f"hand {dist:.2f}m",
            (cx + 14, cy - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        self.output_image = annotated
        return stamped

    def detect_hand_callback(self, request, response):
        hand_point = self.run_hand_inference()
        if hand_point is not None:
            response.point = hand_point
            response.success = True
            self.get_logger().info(
                f"Hand detected at ({hand_point.point.x:.3f}, "
                f"{hand_point.point.y:.3f}, {hand_point.point.z:.3f})"
            )
        else:
            response.success = False
            self.get_logger().info("No hand detected")
        return response

    def find_seat_callback(self, request, response):
        """Callback to find an available seat."""
        self.get_logger().info("Executing service Find Seat")

        if self.image is None:
            response.success = False
            self.get_logger().warn("No image received yet.")
            return response

        frame = self.image
        self.output_image = frame.copy()

        self.people = []
        self.chairs = []
        self.couches = []

        try:
            self.get_detections(frame)

            has_chair_seat, angle = self.check_chairs(frame)

            if has_chair_seat:
                response.success = True
                response.angle = angle
                self.success(f"Seat found in chair at angle: {angle}")
                return response

            has_couch_seat, angle = self.check_couches(frame)

            if has_couch_seat:
                response.success = True
                response.angle = angle
                self.success(f"Seat found in couch at angle: {angle}")
                return response
        except Exception as e:
            self.get_logger().error(f"Find Seat failed: {e}")
            response.success = False
            return response

        response.success = False
        self.get_logger().warn("No seat found")
        return response

    async def detect_person_callback(self, goal_handle):
        """Callback to return a response until a person is
        detected in a frame or timeout is reached."""
        self.get_logger().info("Executing action Detect Person")

        self.person_found = False
        self.goal_handle = goal_handle
        self.start_time = time.time()
        self.detection_future = Future()

        self.timer = self.create_timer(0.1, self.detect_person)
        await self.detection_future

        result = DetectPerson.Result()
        result.success = self.person_found
        goal_handle.succeed()
        if self.person_found:
            self.success("Person detected")
        else:
            self.get_logger().warn("No person detected")
        return result

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self):
        """Publish the image with the detections if available."""
        self.image_publisher.publish(self.output_image)

    def publish_chair_image(self):
        """Re-publish the last chair-removal annotated frame if available."""
        if self.chair_image is not None:
            self.chair_image_publisher.publish(self.chair_image)

    def chairs_to_remove_callback(self, request, response):
        """Chairs between the robot and the dining table (the ones to ask a
        human to remove): a chair whose bbox bottom edge is lower in the image
        than the table's bottom edge stands in front of the table. Chairs come
        from the YOLO service, the table from moondream. Publishes the
        annotated frame for the robot display."""
        if self.image is None:
            response.message = "No image received yet"
            self.get_logger().warn(response.message)
            return response
        frame = self.image.copy()
        img_h, img_w = frame.shape[:2]

        req = YoloDetect.Request()
        req.classes = [56]  # COCO chair
        future = self.yolo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result() is None or not future.result().success:
            response.message = "YOLO chair detection failed"
            self.get_logger().error(response.message)
            return response
        chairs = list(future.result().detections)

        response.total_chairs = len(chairs)
        if not chairs:
            response.success = True
            response.table_found = True
            response.message = "No chairs detected"
            self.get_logger().info(response.message)
            return response

        tables = []
        if self.moondream_client.wait_for_service(timeout_sec=3.0):
            md_req = MoondreamDetection.Request()
            md_req.subject = "dining table"
            future = self.moondream_client.call_async(md_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
            result = future.result() if future.done() else None
            if result is not None and result.success:
                tables = result.detections
        if not tables:
            response.success = True
            response.message = "No dining table found"
            self.get_logger().warn(response.message)
            return response

        # Largest table bbox (moondream returns normalized coords)
        table = max(tables, key=lambda d: (d.xmax - d.xmin) * (d.ymax - d.ymin))
        table_bottom = table.ymax * img_h

        cv2.rectangle(
            frame,
            (int(table.xmin * img_w), int(table.ymin * img_h)),
            (int(table.xmax * img_w), int(table.ymax * img_h)),
            (255, 0, 0),
            2,
        )
        for det in chairs:
            if det.y2 > table_bottom:
                response.chairs.append(det)
                cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), (0, 0, 255), 3)
            else:
                cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), (0, 255, 0), 1)
        self.chair_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        response.success = True
        response.table_found = True
        response.message = f"{len(response.chairs)}/{len(chairs)} chair(s) to remove"
        self.get_logger().info(response.message)
        return response

    def getAngle(self, x, width):
        """Get the angle for the robot to point at the available seat."""
        diff = x - (width / 2)
        move = diff * MAX_DEGREE / (width / 2)
        return move

    def detect_person(self):
        """Check if there is a person in the frame and resolve the future promise using YOLO service."""
        if self.image is None:
            self.get_logger().warn("No image received yet.")
            return

        frame = self.image
        self.output_image = frame.copy()
        width = frame.shape[1]

        # Call YOLO service for person detection (class 0)
        req = YoloDetect.Request()
        req.classes = [0]
        future = self.yolo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or not future.result().success:
            self.get_logger().error("YOLO detection failed")
            return

        for det in future.result().detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            confidence = det.confidence
            x = int((x1 + x2) / 2)

            if (
                confidence > CONF_THRESHOLD
                and x >= int(width * PERCENTAGE)
                and x <= int(width * (1 - PERCENTAGE))
            ):
                self.person_found = True
                cv2.rectangle(
                    self.output_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )
                break
            cv2.rectangle(
                self.output_image,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2,
            )

        if self.person_found or (time.time() - self.start_time) > CHECK_TIMEOUT:
            self.timer.cancel()
            self.detection_future.set_result(self.person_found)

    def get_detections(self, frame) -> None:
        """Obtain YOLO detections for people, chairs, and couches using YOLO service."""
        req = YoloDetect.Request()
        req.classes = [0, 56, 57]
        future = self.yolo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or not future.result().success:
            self.get_logger().error("YOLO detection failed")
            return

        self.get_logger().info(
            f"YOLO service response: success={getattr(future.result(), 'success', None)}, detections={getattr(future.result(), 'detections', None)}"
        )

        if future.result() is None or not future.result().success:
            self.get_logger().error("YOLO detection failed")
            return

        for det in future.result().detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            class_id = det.class_id
            label = det.class_id
            bbox = (x1, y1, x2, y2)
            confidence = det.confidence
            area = (x2 - x1) * (y2 - y1)
            area_percentage = area / (frame.shape[0] * frame.shape[1])
            color = (255, 0, 0)

            if (
                confidence < CONF_THRESHOLD
                or area_percentage < AREA_PERCENTAGE_THRESHOLD
            ):
                continue

            if class_id == 0:
                self.people.append({"bbox": bbox, "label": label, "class_id": class_id})
                color = (0, 0, 255)
            elif class_id == 56:
                self.chairs.append({"bbox": bbox, "label": label, "class_id": class_id})
            elif class_id == 57:
                self.couches.append(
                    {"bbox": bbox, "label": label, "class_id": class_id}
                )

            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                self.output_image,
                str(label),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
                cv2.LINE_AA,
            )

    def _poll_areas(self):
        """Fetch map areas without blocking the executor; stop once loaded."""
        if self.areas is not None:
            self._areas_timer.cancel()
            return
        if self._areas_future is None:
            if self.areas_client.service_is_ready():
                self._areas_future = self.areas_client.call_async(MapAreas.Request())
        elif self._areas_future.done():
            result = self._areas_future.result()
            if result is not None and result.areas:
                self.areas = json.loads(result.areas)
                self.get_logger().info(
                    f"Loaded areas for rooms: {list(self.areas.keys())}"
                )
            self._areas_future = None

    def _get_areas(self):
        """Return the cached map areas (None until _poll_areas has loaded them)."""
        if self.areas is None:
            self.get_logger().warn("Map areas not loaded yet; skipping house filter.")
        return self.areas

    def check_chairs(self, frame) -> tuple[bool, float]:
        """Check if there is an available chair with
        no person within the bbox and return the angle
        of the largest available chair."""
        chair_queue = queue.PriorityQueue()

        chairs_in_room = filter_detections_in_house(
            frame,
            self.chairs,
            [56],
            ["living_room"],
            self.camera_info,
            self.depth_image,
            self.tf_buffer,
            self._get_areas(),
            CAMERA_FRAME,
            rotation=self.rotation,
        )

        for chair in chairs_in_room:
            occupied = False
            xmin = chair["bbox"][0]
            xmax = chair["bbox"][2]
            y_center_chair = (chair["bbox"][1] + chair["bbox"][3]) / 2
            cv2.circle(
                self.output_image,
                (int((xmax - xmin) / 2), int(y_center_chair)),
                5,
                (0, 255, 255),
                -1,
            )

            for person in self.people:
                center_x = (person["bbox"][0] + person["bbox"][2]) / 2
                person_y = person["bbox"][3]

                if center_x >= xmin and center_x <= xmax and person_y > y_center_chair:
                    occupied = True
                    cv2.circle(
                        self.output_image,
                        (int(center_x), int(person_y)),
                        5,
                        (0, 0, 255),
                        -1,
                    )
                    break

            if not occupied:
                area = xmax - xmin
                output = (chair["bbox"][0] + chair["bbox"][2]) / 2
                chair_queue.put(
                    (
                        -1 * area,
                        output,
                        chair["bbox"][0],
                        chair["bbox"][1],
                        chair["bbox"][2],
                        chair["bbox"][3],
                    )
                )
            else:
                cv2.rectangle(
                    self.output_image,
                    (xmin, chair["bbox"][1]),
                    (xmax, chair["bbox"][3]),
                    (0, 0, 255),
                    2,
                )

        if chair_queue.qsize() != 0:
            _, output, a, b, c, d = chair_queue.get()
            cv2.rectangle(self.output_image, (a, b), (c, d), (0, 255, 0), 2)
            self.get_logger().info(f"Chair found: {output}")
            return True, self.getAngle(output, frame.shape[1])

        return False, 0

    def check_couches(self, frame) -> tuple[bool, float]:
        """Check if there is an available space in the
        couches and return the angle of the largest space."""
        available_spaces = queue.PriorityQueue()

        couches_in_room = filter_detections_in_house(
            frame,
            self.couches,
            [57],
            ["living_room"],
            self.camera_info,
            self.depth_image,
            self.tf_buffer,
            self._get_areas(),
            CAMERA_FRAME,
            rotation=self.rotation,
        )

        # Check if there are couch spaces available
        width = frame.shape[1]
        for couch in couches_in_room:
            # YOLO can return x2 == image width; clamp to valid indices
            couch_left = min(max(couch["bbox"][0], 0), width - 1)
            couch_right = min(max(couch["bbox"][2], 0), width - 1)
            space = np.zeros(width, dtype=int)

            # Fill the space with 1 if there is a person
            for person in self.people:
                xmin = person["bbox"][0]
                xmax = person["bbox"][2]
                space[xmin : xmax + 1] = 1

            left = couch_left
            space[couch_left] = 0
            space[couch_right] = 0

            for i in range(couch_left, couch_right):
                if space[i] == 0:
                    if left is None:
                        left = i

                else:
                    if left is not None:
                        available_spaces.put((-1 * (i - left), left, i))
                        left = None

            if left is not None:
                available_spaces.put((-1 * (couch_right - left), left, couch_right))

        if available_spaces.qsize() > 0:
            _, left, right = available_spaces.get()
            output = (left + right) / 2
            cv2.rectangle(
                self.output_image, (left, 0), (right, frame.shape[0]), (0, 255, 0), 2
            )
            self.get_logger().info(f"Space found: {output}")
            return True, self.getAngle(output, frame.shape[1])

        return False, 0


def main(args=None):
    rclpy.init(args=args)
    node = HRICCommands()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

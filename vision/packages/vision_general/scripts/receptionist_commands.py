#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import cv2
import mediapipe as mp
import numpy as np
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from rclpy.task import Future

from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import DetectHand, FindSeat, YoloDetect
from vision_general.utils.calculations import get_depth, deproject_pixel_to_point
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    CHECK_PERSON_TOPIC,
    DEPTH_IMAGE_TOPIC,
    DETECT_HAND_SERVICE,
    FIND_SEAT_TOPIC,
    IMAGE_TOPIC_RECEPTIONIST,
)

from ament_index_python.packages import get_package_share_directory

package_share_dir = get_package_share_directory("vision_general")

PERCENTAGE = 0.3
MAX_DEGREE = 50
AREA_PERCENTAGE_THRESHOLD = 0.01
CONF_THRESHOLD = 0.4
CHECK_TIMEOUT = 5


class ReceptionistCommands(Node):
    def __init__(self):
        super().__init__("receptionist_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.find_seat_service = self.create_service(
            FindSeat,
            FIND_SEAT_TOPIC,
            self.find_seat_callback,
            callback_group=self.callback_group,
        )
        self.image_publisher = self.create_publisher(
            Image, IMAGE_TOPIC_RECEPTIONIST, 10, callback_group=self.callback_group
        )
        self.person_detection_action_server = ActionServer(
            self,
            DetectPerson,
            CHECK_PERSON_TOPIC,
            self.detect_person_callback,
            callback_group=self.callback_group,
        )

        self.yolo_client = self.create_client(
            YoloDetect, "yolo_detect", callback_group=self.callback_group
        )

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("YOLO service not available, waiting...")

        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.output_image = []
        self.check = False

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.depth_sub = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, qos
        )

        self.detect_hand_service = self.create_service(
            DetectHand,
            DETECT_HAND_SERVICE,
            self.detect_hand_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("ReceptionistCommands Ready.")

        self.create_timer(0.1, self.publish_image)

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
        if self.image is None:
            return

        image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.hands.process(image_rgb)

        if not results.multi_hand_landmarks:
            self.get_logger().info("No hand detected")
            return None

        hand_landmarks = results.multi_hand_landmarks[0]
        lm = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

        h, w, _ = self.image.shape
        px, py = int(lm.x * w), int(lm.y * h)

        if self.depth_image is not None and self.camera_info is not None:
            dh, dw = self.depth_image.shape[:2]
            dpx = int(px * dw / w)
            dpy = int(py * dh / h)
            dpx = max(0, min(dpx, dw - 1))
            dpy = max(0, min(dpy, dh - 1))
            depth = get_depth(self.depth_image, (dpx, dpy))
            point3d = deproject_pixel_to_point(self.camera_info, (px, py), depth)

            stamped = PointStamped()
            stamped.header.frame_id = CAMERA_FRAME
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.point.x = float(point3d[0])
            stamped.point.y = float(point3d[1])
            stamped.point.z = float(point3d[2])
            return stamped
        else:
            self.get_logger().warn("Depth image or camera info not available")
            return None

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
        if len(self.output_image) != 0:
            # cv2.imshow("Receptionist Commands", self.output_image)
            # cv2.waitKey(1)
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

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
                self.people.append({"bbox": bbox, "label": label, "class": class_id})
                color = (0, 0, 255)
            elif class_id == 56:
                self.chairs.append({"bbox": bbox, "label": label, "class": class_id})
            elif class_id == 57:
                self.couches.append({"bbox": bbox, "label": label, "class": class_id})

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

    def check_chairs(self, frame) -> tuple[bool, float]:
        """Check if there is an available chair with
        no person within the bbox and return the angle
        of the largest available chair."""
        chair_queue = queue.PriorityQueue()

        for chair in self.chairs:
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

        if len(self.chairs) != 0:
            _, output, a, b, c, d = chair_queue.get()
            cv2.rectangle(self.output_image, (a, b), (c, d), (0, 255, 0), 2)
            self.get_logger().info(f"Chair found: {output}")
            return True, self.getAngle(output, frame.shape[1])

        return False, 0

    def check_couches(self, frame) -> tuple[bool, float]:
        """Check if there is an available space in the
        couches and return the angle of the largest space."""
        available_spaces = queue.PriorityQueue()

        # Check if there are couch spaces available
        for couch in self.couches:
            couch_left = couch["bbox"][0]
            couch_right = couch["bbox"][2]
            space = np.zeros(frame.shape[1], dtype=int)

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
    node = ReceptionistCommands()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

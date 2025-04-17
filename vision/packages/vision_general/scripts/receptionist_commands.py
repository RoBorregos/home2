#!/usr/bin/env python3

"""
Node to detect people and find
available seats. Tasks for receptionist
commands.
"""

import cv2
from ultralytics import YOLO
import pathlib
import numpy as np
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.task import Future

from frida_interfaces.action import DetectPerson
from frida_interfaces.srv import FindSeat
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    CHECK_PERSON_TOPIC,
    FIND_SEAT_TOPIC,
    IMAGE_TOPIC,
)

from ament_index_python.packages import get_package_share_directory

package_share_dir = get_package_share_directory("vision_general")

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"

PERCENTAGE = 0.3
MAX_DEGREE = 30
AREA_PERCENTAGE_THRESHOLD = 0.2
CONF_THRESHOLD = 0.5
CHECK_TIMEOUT = 5


class ReceptionistCommands(Node):
    def __init__(self):
        super().__init__("receptionist_commands")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.find_seat_service = self.create_service(
            FindSeat, FIND_SEAT_TOPIC, self.find_seat_callback
        )
        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)
        self.person_detection_action_server = ActionServer(
            self, DetectPerson, CHECK_PERSON_TOPIC, self.detect_person_callback
        )

        self.image = None
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.output_image = []
        self.check = False

        self.get_logger().info("ReceptionistCommands Ready.")

        self.create_timer(0.1, self.publish_image)

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def find_seat_callback(self, request, response):
        """Callback to find an available seat."""
        self.get_logger().info("Executing service Find Seat")

        if self.image is None:
            response.success = False
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
            cv2.imshow("Receptionist Commands", self.output_image)
            cv2.waitKey(1)
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def getAngle(self, x, width):
        """Get the angle for the robot to point at the available seat."""
        diff = x - (width / 2)
        move = diff * MAX_DEGREE / (width / 2)
        return move

    def detect_person(self):
        """Check if there is a person in the frame and
        resolve the future promise."""
        if self.image is None:
            self.get_logger().warn("No image received yet.")
            return

        frame = self.image
        self.output_image = frame.copy()
        width = frame.shape[1]

        results = self.yolo_model(frame, verbose=False, classes=0)

        for out in results:
            for box in out.boxes:
                x, y, w, h = [round(i) for i in box.xywh[0].tolist()]
                confidence = box.conf.item()

                if (
                    confidence > CONF_THRESHOLD
                    and x >= int(width * PERCENTAGE)
                    and x <= int(width * (1 - PERCENTAGE))
                ):
                    self.person_found = True
                    cv2.rectangle(
                        self.output_image,
                        (int(x - w / 2), int(y - h / 2)),
                        (int(x + w / 2), int(y + h / 2)),
                        (0, 255, 0),
                        2,
                    )
                    break

                cv2.rectangle(
                    self.output_image,
                    (int(x - w / 2), int(y - h / 2)),
                    (int(x + w / 2), int(y + h / 2)),
                    (255, 0, 0),
                    2,
                )

        if self.person_found or (time.time() - self.start_time) > CHECK_TIMEOUT:
            self.timer.cancel()
            self.detection_future.set_result(self.person_found)

    def get_detections(self, frame) -> None:
        """Obtain yolo detections for people, chairs and couches."""
        results = self.yolo_model(frame, verbose=False, classes=[0, 56, 57])

        for out in results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                class_id = box.cls[0].item()
                label = self.yolo_model.names[class_id]
                bbox = (x1, y1, x2, y2)
                confidence = box.conf.item()
                area = (x2 - x1) * (y2 - y1)
                area_percentage = area / (frame.shape[0] * frame.shape[1])
                color = (255, 0, 0)

                if (
                    confidence < CONF_THRESHOLD
                    or area_percentage < AREA_PERCENTAGE_THRESHOLD
                ):
                    continue

                if class_id == 0:
                    self.people.append(
                        {"bbox": bbox, "label": label, "class": class_id}
                    )
                    color = (0, 0, 255)

                elif class_id == 56:
                    self.chairs.append(
                        {"bbox": bbox, "label": label, "class": class_id}
                    )

                elif class_id == 57:
                    self.couches.append(
                        {"bbox": bbox, "label": label, "class": class_id}
                    )

                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    self.output_image,
                    label,
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

#!/usr/bin/env python3

"""Node to handle Restaurant commands"""

from __future__ import annotations

import json
import pathlib
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    SET_TARGET_TOPIC,
)
from frida_constants.vision_enums import Gestures

from pose_detection import PoseDetection


YOLO_LOCATION = str(pathlib.Path(__file__).parent / "Utils" / "yolov8n.pt")


@dataclass
class CallingCustomer:
    """Semantic representation of a customer calling the robot."""

    customer_id: int
    gesture: Gestures
    bbox: Tuple[int, int, int, int]
    confidence: float
    timestamp: float = field(default_factory=lambda: time.time())


class RestaurantCommands(Node):
    """Vision-driven command node for the Restaurant task."""

    #CALLING_CUSTOMER_TOPIC = "/vision/restaurant/calling_customer" TODO: define properly
    CONF_THRESHOLD = 0.5
    DETECTION_TIMER_PERIOD = 0.4
    PERSON_CLASS_ID = 0
    

    def __init__(self) -> None:
        super().__init__("restaurant_commands")

        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.pose_detection = PoseDetection()
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.CALLING_GESTURES = {
            Gestures.RAISING_LEFT_ARM,
            Gestures.RAISING_RIGHT_ARM,
            Gestures.WAVING,
        }

        self.image = None
        self.tracked_customers: Dict[int, CallingCustomer] = {}
        self.customer_sequence = 0

        self.image_subscriber = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10,
        )

        self.calling_customer_publisher = self.create_publisher(
            String, self.CALLING_CUSTOMER_TOPIC, 10
        )

        # TODO: replace String with the tracker-specific interface once defined.
        self.tracker_placeholder_publisher = self.create_publisher(
            String, SET_TARGET_TOPIC, 10
        )

        self.get_logger().info("RestaurantCommands node initialized. Pose detection ready.")

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.image is None:
                return

            self.output_image = self.image.copy()
            self.get_detections(self.image, 0)  # default: 0  - person

        except Exception as e:
            print(f"Error: {e}")

    def _process_image(self) -> None:
        """Periodically inspect the latest frame for calling customers."""

        if self.image is None:
            return

        calling_customer = self._detect_calling_customer(self.image)
        if calling_customer is None:
            return

        self._publish_calling_customer(calling_customer)
        self._send_to_tracker(calling_customer)

    def _detect_calling_customer(self, frame, all) -> Optional[CallingCustomer]:
        """Detect and return the first or all customers calling the robot, if any."""

        people = self._detect_people(frame)
        if not people:
            self.get_logger().debug("No people detected in the current frame.")
            return None

        customers = []
        for person in people:
            x1, y1, x2, y2 = person["bbox"]
            cropped_frame = frame[y1:y2, x1:x2]

            gesture = self.pose_detection.detectGesture(cropped_frame)
            if gesture not in self.CALLING_GESTURES:
                continue

            #customer_id = 
            current_customer = CallingCustomer(
                    #customer_id=customer_id,
                    gesture=gesture,
                    bbox=person["bbox"],
                    confidence=person["confidence"],
                )
            
            if all:
                return current_customer
            else:
                customers.append(current_customer)

        return customers

    def _detect_people(self, frame) -> List[Dict[str, Any]]:
        """Run the YOLO detector and return filtered person detections."""

        detections: List[Dict[str, Any]] = []
        try:
            results = self.yolo_model(frame, verbose=False, classes=[self.PERSON_CLASS_ID])
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return detections

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [round(val) for val in box.xyxy[0].tolist()]
                confidence = box.conf.item()
                if confidence < self.CONF_THRESHOLD:
                    continue
                detections.append(
                    {
                        "bbox": (x1, y1, x2, y2),
                        "confidence": confidence,
                        "area": (x2 - x1) * (y2 - y1),
                    }
                )

        return detections

    def _publish_calling_customer(self, calling_customer: CallingCustomer) -> None:
        """Publish detected calling customer information."""
        self.get_logger().info(f"Publishing calling customer: {calling_customer}")


    def _send_to_tracker(self, calling_customer: CallingCustomer) -> None:
        """Placeholder bridge towards the external tracker node."""




def main(args=None) -> None:
    """Initialize the rclpy context and spin the RestaurantCommands node."""

    rclpy.init(args=args)
    node = RestaurantCommands()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

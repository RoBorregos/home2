#!/usr/bin/env python3

"""
Node to handle GPSR commands.
"""

import cv2
from ultralytics import YOLO
import pathlib

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from frida_interfaces.srv import CountBy, CountByGesture, CountByPose

from ament_index_python.packages import get_package_share_directory

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    COUNT_BY_PERSON_TOPIC,
    IMAGE_TOPIC,
    # COUNT_BY_COLOR_TOPIC,
    # COUNT_BY_CLOTHES_TOPIC,
    # COUNT_BY_OBJECTS_TOPIC,
    COUNT_BY_GESTURES_TOPIC,
    COUNT_BY_POSE_TOPIC,
)

from frida_constants.vision_enums import Poses, Gestures

from pose_detection import PoseDetection

package_share_dir = get_package_share_directory("vision_general")

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"
# MOONDREAM_LOCATION = str(
#     pathlib.Path(package_share_dir) / "Utils/moondream-2b-int8.mf.gz"
# )
PERCENTAGE = 0.3
MAX_DEGREE = 30
AREA_PERCENTAGE_THRESHOLD = 0.2
CONF_THRESHOLD = 0.5
CHECK_TIMEOUT = 5


class GPSRCommands(Node):
    def __init__(self):
        super().__init__("gpsr_commands")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        # Define services for GPSR commands

        self.count_by_gestures_service = self.create_service(
            CountByGesture, COUNT_BY_GESTURES_TOPIC, self.count_by_gestures_callback
        )

        self.count_by_pose_service = self.create_service(
            CountByPose, COUNT_BY_POSE_TOPIC, self.count_by_pose_callback
        )

        # self.count_by_objects_service = self.create_service(
        #     CountBy, COUNT_BY_OBJECTS_TOPIC, self.count_by_objects_callback
        # )

        self.count_by_person_service = self.create_service(
            CountBy, COUNT_BY_PERSON_TOPIC, self.count_by_person_callback
        )

        # self.count_by_color_service = self.create_service(
        #     CountBy, COUNT_BY_COLOR_TOPIC, self.count_by_color_callback
        # )

        # self.count_by_clothes_service = self.create_service(
        #     CountBy, COUNT_BY_CLOTHES_TOPIC, self.count_by_clothes_callback
        # )

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.image = None
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.pose_detection = PoseDetection()
        self.output_image = []

        self.get_logger().info("GPSRCommands Ready.")
        self.create_timer(0.1, self.publish_image)

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

    def count_by_gestures_callback(self, request, response):
        """Callback to count gestures in the image."""
        self.get_logger().info("Executing service Count By Gestures")

        if self.image is None:
            response.success = False
            response.count = 0
            return response

        frame = self.image
        self.output_image = frame.copy()

        gesture_requested = request.gesture_requested

        # Convert gesture_requested to Enum Gestures
        try:
            gesture_requested_enum = Gestures[gesture_requested]
        except KeyError:
            self.get_logger().warn(f"Pose {gesture_requested} is not valid.")
            response.success = False
            response.count = 0
            return response

        # Detect people using YOLO
        self.get_detections(frame, 0)

        gesture = self.count_gestures(frame)

        gesture_count = gesture.get(gesture_requested_enum, 0)

        response.success = True
        response.count = gesture_count
        self.get_logger().info(f"Gesture {gesture_requested} counted: {gesture_count}")
        return response

    def count_by_pose_callback(self, request, response):
        """Callback to count a specific pose in the image."""
        self.get_logger().info("Executing service Count By Pose")

        if self.image is None:
            response.success = False
            response.count = 0
            return response

        frame = self.image
        self.output_image = frame.copy()

        pose_requested = request.pose_requested

        # Convert pose_requested to Enum Poses
        try:
            pose_requested_enum = Poses[pose_requested]
        except KeyError:
            self.get_logger().warn(f"Pose {pose_requested} is not valid.")
            response.success = False
            response.count = 0
            return response

        # Detect people using YOLO
        self.get_detections(frame, 0)

        pose = self.count_poses(frame)

        pose_count = pose.get(pose_requested_enum, 0)

        response.success = True
        response.count = pose_count
        self.get_logger().info(f"Pose {pose_requested} counted: {pose_count}")
        return response

    def count_by_objects_callback(self, request, response):
        """Callback to count objects in the image."""
        self.get_logger().info("Executing service Count By Objects")
        if self.image is None:
            response.success = False
            return response

        frame = self.image
        self.output_image = frame.copy()

        # Detect objects using YOLO
        # self.get_detections(frame, comp)

        # Update objects_count
        objects_count = 0

        response.success = True
        response.count = objects_count
        self.get_logger().info(f"Objects counted: {objects_count}")
        return response

    def count_by_person_callback(self, request, response):
        """Callback to count people in the image."""
        self.get_logger().info("Executing service Count By Person")
        if self.image is None:
            response.success = False
            return response

        frame = self.image
        self.output_image = frame.copy()

        # Detect people using YOLO
        self.get_detections(frame, 0)

        # Count people detected
        people_count = len(self.people)

        response.success = True
        response.count = people_count
        self.get_logger().info(f"People counted: {people_count}")
        return response

    def count_by_color_callback(self, request, response):
        """Callback to count by color in the image."""
        self.get_logger().info("Executing service Count By Color")

        # Function to count by color - to implement
        # self.count_color(frame, color)

        color_count = 0

        response.success = True
        response.count = color_count
        self.get_logger().info(f"Color counted: {color_count}")
        return response

    def count_by_clothes_callback(self, request, response):
        """Callback to count by clothes in the image."""
        self.get_logger().info("Executing service Count By Color")

        # Function to count by color - to implement
        # self.count_clothes(frame, clothes)

        clothes_count = 0

        response.success = True
        response.count = clothes_count
        self.get_logger().info(f"Clothes counted: {clothes_count}")
        return response

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self):
        """Publish the image with the detections if available."""
        if len(self.output_image) != 0:
            cv2.imshow("GPSR Commands", self.output_image)
            cv2.waitKey(1)
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def count_poses(self, frame):
        """Count the poses in the image and return a dictionary."""
        pose_count = {
            Poses.UNKNWON: 0,
            Poses.STANDING: 0,
            Poses.SITTING: 0,
            Poses.LYING_DOWN: 0,
        }

        # Detect poses for each detected person
        for person in self.people:
            x, y, w, h = person["bbox"]
            cropped_frame = frame[y : y + h, x : x + w]

            pose = self.pose_detection.detectPose(cropped_frame)

            # Increment the pose count based on detected pose
            if pose in pose_count:
                pose_count[pose] += 1

        return pose_count

    def count_gestures(self, frame):
        """Count the gestures in the image and return a dictionary."""
        gesture_count = {
            Gestures.WAVING: 0,
            Gestures.RAISING_LEFT_ARM: 0,
            Gestures.RAISING_RIGHT_ARM: 0,
            Gestures.POINTING_LEFT: 0,
            Gestures.POINTING_RIGHT: 0,
        }

        # Detect gestures for each detected person
        for person in self.people:
            x, y, w, h = person["bbox"]
            cropped_frame = frame[y : y + h, x : x + w]

            gesture = self.pose_detection.detectGesture(cropped_frame)

            # Increment the gesture count based on detected gesture
            if gesture in gesture_count:
                gesture_count[gesture] += 1

        return gesture_count

    def get_detections(self, frame, comp_class) -> None:
        """Obtain YOLO detections for people."""
        results = self.yolo_model(frame, verbose=False, classes=[comp_class])

        if comp_class == 0:
            self.people = []
            for out in results:
                for box in out.boxes:
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                    confidence = box.conf.item()

                    if confidence > CONF_THRESHOLD:
                        self.people.append(
                            {"bbox": (x1, y1, x2, y2), "confidence": confidence}
                        )

                    cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        self.output_image,
                        "Person",
                        (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )


def main(args=None):
    rclpy.init(args=args)
    node = GPSRCommands()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

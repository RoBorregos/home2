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
import time

from frida_interfaces.srv import (
    CountBy,
    CountByPose,
    PersonPoseGesture,
    CropQuery,
    CountByColor,
)

from ament_index_python.packages import get_package_share_directory

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    COUNT_BY_PERSON_TOPIC,
    IMAGE_TOPIC,
    COUNT_BY_COLOR_TOPIC,
    COUNT_BY_POSE_TOPIC,
    POSE_GESTURE_TOPIC,
    CROP_QUERY_TOPIC,
    COUNT_BY_GESTURE_TOPIC,
)

from frida_constants.vision_enums import Poses, Gestures, DetectBy

from pose_detection import PoseDetection

package_share_dir = get_package_share_directory("vision_general")

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"
PERCENTAGE = 0.3
MAX_DEGREE = 30
AREA_PERCENTAGE_THRESHOLD = 0.2
CONF_THRESHOLD = 0.5
TIMEOUT = 5.0
TIMEOUT = 5.0


class GPSRCommands(Node):
    def __init__(self):
        super().__init__("gpsr_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        # Define services for GPSR commands
        self.count_by_pose_service = self.create_service(
            CountByPose,
            COUNT_BY_POSE_TOPIC,
            self.count_by_pose_callback,
            callback_group=self.callback_group,
        )

        self.count_by_gestures_service = self.create_service(
            CountByPose, COUNT_BY_GESTURE_TOPIC, self.count_by_gestures_callback
        )

        self.count_by_person_service = self.create_service(
            CountBy, COUNT_BY_PERSON_TOPIC, self.count_by_person_callback
        )

        self.count_by_color_service = self.create_service(
            CountByColor,
            COUNT_BY_COLOR_TOPIC,
            self.count_by_color_callback,
            callback_group=self.callback_group,
        )

        self.pose_gesture_detection_service = self.create_service(
            PersonPoseGesture,
            POSE_GESTURE_TOPIC,
            self.detect_pose_gesture_callback,
            callback_group=self.callback_group,
        )

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.image = None
        self.yolo_model = YOLO(YOLO_LOCATION)
        self.pose_detection = PoseDetection()
        self.output_image = []

        self.get_logger().info("GPSRCommands Ready.")
        # self.create_timer(0.1, self.publish_image)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

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

    def count_by_pose_callback(self, request, response):
        """Callback to count a specific pose in the image."""
        self.get_logger().info("Executing service Count By Pose")

        pose_count = {
            Poses.UNKNOWN: 0,
            Poses.STANDING: 0,
            Poses.SITTING: 0,
            Poses.LYING_DOWN: 0,
        }

        if self.image is None:
            response.success = False
            response.count = 0
            return response

        frame = self.image
        self.output_image = frame.copy()

        pose_requested = request.pose_requested

        # Convert pose_requested to Enum Poses
        try:
            pose_requested_enum = Poses(pose_requested)
        except KeyError:
            self.get_logger().warn(f"Pose {pose_requested} is not valid.")
            response.success = False
            response.count = 0
            return response

        self.get_detections(frame, 0)

        # replace underscore with space in the pose_requested
        pose_requested = pose_requested.replace("_", "  ")
        pose_requested = pose_requested.replace("_", "", 1)

        for person in self.people:
            x1, y1, x2, y2 = person["bbox"]

            prompt = f"Reply only with 1 if the person is {pose_requested}. Otherwise, reply only with 0."
            status, response_q = self.moondream_crop_query(
                prompt, [float(y1), float(x1), float(y2), float(x2)]
            )
            if status:
                print(response_q)
                response_clean = response_q.strip()
                if response_clean == "1":
                    pose_count[pose_requested_enum] += 1
                    self.get_logger().info(f"Person is {pose_requested}.")
                elif response_clean != "0":
                    self.get_logger().warn(f"Unexpected response: {response_clean}")

        response.success = True
        response.count = pose_count[pose_requested_enum]
        self.get_logger().info(f"People with pose {pose_requested}: {response.count}")
        return response

    def count_by_gestures_callback(self, request, response):
        """Callback to count gestures in the image."""
        self.get_logger().info("Executing service Count By Gestures")

        if self.image is None:
            response.success = False
            response.count = 0
            return response

        frame = self.image
        self.output_image = frame.copy()

        gesture_requested = request.pose_requested

        # Convert gesture_requested to Enum Gestures
        try:
            gesture_requested_enum = Gestures(gesture_requested)
        except KeyError:
            self.get_logger().warn(f"Gesture {gesture_requested} is not valid.")
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

    def count_gestures(self, frame):
        """Count the gestures in the image and return a dictionary."""
        gesture_count = {
            Gestures.UNKNOWN: 0,
            Gestures.UNKNOWN: 0,
            Gestures.WAVING: 0,
            Gestures.RAISING_LEFT_ARM: 0,
            Gestures.RAISING_RIGHT_ARM: 0,
            Gestures.POINTING_LEFT: 0,
            Gestures.POINTING_RIGHT: 0,
        }

        # Detect gestures for each detected person
        for person in self.people:
            x1, y1, x2, y2 = person["bbox"]

            # Crop the frame to the bounding box of the person
            cropped_frame = frame[y1:y2, x1:x2]

            gesture = self.pose_detection.detectGesture(cropped_frame)

            # Increment the gesture count based on detected gesture
            if gesture in gesture_count:
                gesture_count[gesture] += 1

        return gesture_count

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
        """Callback to count people wearing a specific color and clothing."""
        self.get_logger().info("Executing service Count By Color")

        if self.image is None:
            response.success = False
            response.count = 0
            return response

        frame = self.image
        self.output_image = frame.copy()

        clothing = request.clothing
        color = request.color

        self.get_detections(frame, 0)

        count = 0

        for person in self.people:
            x1, y1, x2, y2 = person["bbox"]

            prompt = f"Reply only with 1 if the person is wearing a {color} {clothing}. Otherwise, reply only with 0."
            status, response_q = self.moondream_crop_query(
                prompt, [float(y1), float(x1), float(y2), float(x2)]
            )
            if status:
                print(response_q)
                response_clean = response_q.strip()
                if response_clean == "1":
                    count += 1
                    self.get_logger().info(
                        f"Person {count} is wearing a {color} {clothing}."
                    )
                elif response_clean != "0":
                    self.get_logger().warn(f"Unexpected response: {response_clean}")

        response.success = True
        response.count = count
        self.get_logger().info(f"People wearing a {color} {clothing}: {count}")
        return response

    def detect_pose_gesture_callback(self, request, response):
        """Callback to detect a specific pose or gesture in the image."""
        self.get_logger().info("Executing service Pose Detection")

        if self.image is None:
            response.success = False
            response.result = ""
            return response

        frame = self.image
        self.output_image = frame.copy()

        # Detect people using YOLO
        self.get_detections(frame, 0)

        # Detect gesture for the person with the biggest bounding box
        biggest_person = max(self.people, key=lambda p: p["area"], default=None)
        x1, y1, x2, y2 = biggest_person["bbox"]
        cropped_frame = frame[y1:y2, x1:x2]

        # Crop the frame to the bounding box of the person
        type_requested = request.type_requested

        if type_requested == DetectBy.POSES.value:
            prompt = "Respond 'standing' if the person in the image is standing, 'sitting' if the person in the image is sitting, 'lying down' if the person in the image is lying down or 'unknown' if the person is not doing any of the previous."
            status, response_q = self.moondream_crop_query(
                prompt, [float(y1), float(x1), float(y2), float(x2)]
            )

            if status:
                self.get_logger().info(f"The person is {response_q}.")
                response_clean = response_q.replace(" ", "_")
                response_clean = response_clean.replace("_", "", 1)

            response.result = response_clean

        elif type_requested == DetectBy.GESTURES.value:
            gesture = self.detect_gesture(cropped_frame)  
            response.result = gesture  
            response_clean = gesture
            self.get_logger().info(f"The person is {gesture}")
        else:
            self.get_logger().warn(f"Type {type_requested} is not valid.")
            response.success = False
            response.result = ""
            return response

        response.success = True
        self.get_logger().info(f"{type_requested} detected: {response_clean}")
        return response

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self):
        """Publish the image with the detections if available."""
        if len(self.output_image) != 0:
            # cv2.imshow("GPSR Commands", self.output_image)
            cv2.waitKey(1)
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

    def detect_gesture(self, cropped_frame):
        """Detect the pose in the image."""
        gestures = [
            Gestures.UNKNOWN,
            Gestures.WAVING,
            Gestures.RAISING_LEFT_ARM,
            Gestures.RAISING_RIGHT_ARM,
            Gestures.POINTING_LEFT,
            Gestures.POINTING_RIGHT,
        ]

        gesture = self.pose_detection.detectGesture(cropped_frame)

        if gesture in gestures:
            return gesture.value

        return Gestures.UNKNOWN.value

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
                            {
                                "bbox": (x1, y1, x2, y2),
                                "confidence": confidence,
                                "area": (x2 - x1) * (y2 - y1),
                            }
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

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            print("Waiting for future to complete...")
        return future

    def moondream_crop_query(self, prompt: str, bbox: list[float]) -> tuple[int, str]:
        """Makes a query of the current image using moondream."""
        self.get_logger().info(f"Querying image with prompt: {prompt}")

        height, width = self.image.shape[:2]

        ymin = bbox[0] / height
        xmin = bbox[1] / width
        ymax = bbox[2] / height
        xmax = bbox[3] / width

        request = CropQuery.Request()
        request.query = prompt
        request.ymin = ymin
        request.xmin = xmin
        request.ymax = ymax
        request.xmax = xmax

        future = self.moondream_client.call_async(request)
        future = self.wait_for_future(future, 15)
        result = future.result()
        if result is None:
            self.get_logger().error("Moondream service returned None.")
            return 0, "0"
        if result.success:
            self.get_logger().info(f"Moondream result: {result.result}")
            return 1, result.result


def main(args=None):
    rclpy.init(args=args)
    node = GPSRCommands()
    executor = rclpy.executors.MultiThreadedExecutor(5)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

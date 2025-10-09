#!/usr/bin/env python3

"""
Node to handle GPSR commands.
"""

import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
import os
import json

from frida_interfaces.srv import (
    CountBy,
    CountByPose,
    PersonPoseGesture,
    CropQuery,
    CountByColor,
    ReadQr,
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
    READ_QR_TOPIC,
)

from frida_constants.vision_enums import Poses, Gestures, DetectBy

from frida_interfaces.srv import YoloDetect  
from frida_interfaces.msg import Detection   

from pose_detection import PoseDetection

package_share_dir = get_package_share_directory("vision_general")

constants = get_package_share_directory("frida_constants")
file_path = os.path.join(constants, "map_areas/areas.json")


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
            CountByPose,
            COUNT_BY_GESTURE_TOPIC,
            self.count_by_gestures_callback,
            callback_group=self.callback_group,
        )

        self.count_by_person_service = self.create_service(
            CountBy,
            COUNT_BY_PERSON_TOPIC,
            self.count_by_person_callback,
            callback_group=self.callback_group,
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

        self.read_qr_service = self.create_service(
            ReadQr,
            READ_QR_TOPIC,
            self.read_qr_callback,
            callback_group=self.callback_group,
        )

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.yolo_client = self.create_client(YoloDetect, "yolo_detect", callback_group=self.callback_group)

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("YOLO service not available, waiting...")

        self.image = None
        self.pose_detection = PoseDetection()
        self.qr_detector = cv2.QRCodeDetector()
        self.output_image = []
        self.people = []

        self.get_logger().info("GPSRCommands Ready.")
        # self.create_timer(0.1, self.publish_image)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        # Load areas from the JSON file
        with open(file_path, "r") as file:
            self.areas = json.load(file)

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.image is None:
                return

            self.output_image = self.image.copy()
            self.get_detections(0)  # default: 0  - person

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

        self.get_detections(0)

        if len(self.people) == 0:
            self.get_logger().warn("No people detected in the image.")
            response.success = True
            response.count = 0
            return response

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
        self.get_detections(0)

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
            Gestures.WAVING: 0,
            Gestures.RAISING_LEFT_ARM: 0,
            Gestures.RAISING_RIGHT_ARM: 0,
            Gestures.POINTING_LEFT: 0,
            Gestures.POINTING_RIGHT: 0,
        }

        if len(self.people) == 0:
            self.get_logger().warn("No people detected in the image.")
            return gesture_count

        # Detect gestures for each detected person
        for person in self.people:
            x1, y1, x2, y2 = person["bbox"]

            # Crop the frame to the bounding box of the person
            cropped_frame = frame[y1:y2, x1:x2]

            gesture = self.pose_detection.detectGesture(cropped_frame)

            # Increment the gesture count based on detected gesture
            if gesture in gesture_count:
                gesture_count[gesture] += 1
                if gesture == Gestures.WAVING:
                    gesture_count[Gestures.RAISING_LEFT_ARM] += 1
                    gesture_count[Gestures.RAISING_RIGHT_ARM] += 1

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
        self.get_detections(0)

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

        self.get_detections(0)

        if len(self.people) == 0:
            self.get_logger().warn("No people detected in the image.")
            response.success = True
            response.count = 0
            return response

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
        self.get_detections(0)

        if len(self.people) == 0:
            self.get_logger().warn("No people detected in the image.")
            response.success = False
            response.result = ""
            return response

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

    def read_qr_callback(self, request, response):
        """Callback to detect and decode QR code in the image"""
        self.get_logger().info("Executing service Read Qr")
        if self.image is None:
            response.success = False
            response.result = ""
            return response

        frame = self.image
        self.output_image = frame.copy()

        # Detect QR using cv2.QRCodeDetector
        retval, _, _ = self.qr_detector.detectAndDecode(frame)

        if retval == "":
            response.success = False
            response.result = ""
            self.get_logger().warn("No qr code detected")
            return response

        self.success(f"QR code read successfully: {retval}")
        response.result = retval
        response.success = True
        return response

    def success(self, message):
        """Log a success message."""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def publish_image(self):
        """Publish the image with the detections if available."""
        if len(self.output_image) != 0:
            # cv2.imshow("GPSR Commands", self.output_image)
            # cv2.waitKey(1)
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

    def get_detections(self, comp_class=None, timeout=5.0):
        """
        Obtain YOLO detections via the YOLO service.
        comp_class: int or None (None = detect all classes)
        """

        # Create request
        req = YoloDetect.Request()
        req.classes = [comp_class] if comp_class is not None else []

        # Call YOLO service
        future = self.yolo_client.call_async(req)

        # Wait for the future while spinning the node
        future = self.wait_for_future(future, 15)
        result = future.result()
        
        if result is None or not result.success:
            self.get_logger().error("YOLO detection failed")
            return []

        # Parse detections
        detections = []
        for det in result.detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            conf, cls_id = det.confidence, det.class_id
            detections.append({
                "bbox": (x1, y1, x2, y2),
                "confidence": conf,
                "class_id": cls_id,
                "area": (x2 - x1) * (y2 - y1),
            })

        # Store people if comp_class == 0
        if comp_class == 0:
            self.people = [d for d in detections if d["class_id"] == 0]

        return detections

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            # print("Waiting for future to complete...")
            pass

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

    def is_inside(self, x, y, polygon):
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if (y1 > y) != (y2 > y):
                xinters = (y - y1) * (x2 - x1) / (
                    y2 - y1 + 1e-10
                ) + x1  # Avoid zero division
                if x < xinters:
                    inside = not inside
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = GPSRCommands()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

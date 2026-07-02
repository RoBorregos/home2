#!/usr/bin/env python3

"""
Node to handle GPSR commands.
"""

import cv2
import rclpy
import rclpy.qos
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_general.utils.ros_utils import wait_for_future

from frida_interfaces.srv import (
    CountBy,
    CountByPose,
    PersonPoseGesture,
    CropQuery,
    CountByColor,
    MapAreas,
)

from ament_index_python.packages import get_package_share_directory

from frida_constants.vision_constants import (
    IMAGE_ORIENTED_TOPIC,
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    COUNT_BY_PERSON_TOPIC,
    DEPTH_IMAGE_TOPIC,
    IMAGE_TOPIC,
    COUNT_BY_COLOR_TOPIC,
    COUNT_BY_POSE_TOPIC,
    POSE_GESTURE_TOPIC,
    CROP_QUERY_TOPIC,
    COUNT_BY_GESTURE_TOPIC,
    YOLO_DETECTION_TOPIC,
)
from frida_constants.navigation_constants import AREAS_SERVICE
from vision_general.utils.area_check import filter_detections_in_house, fetch_map_areas
from tf2_ros import Buffer, TransformListener

from frida_constants.vision_enums import Poses, Gestures, DetectBy

from frida_interfaces.srv import YoloDetect

from pose_detection import PoseDetection

package_share_dir = get_package_share_directory("vision_general")


class GPSRCommands(Node):
    def __init__(self):
        super().__init__("gpsr_commands")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.image_subscriber = self.create_subscription(
            Image, IMAGE_ORIENTED_TOPIC, self.image_callback, qos
        )
        self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_callback,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos,
            callback_group=self.callback_group,
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

        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 10)

        self.yolo_client = self.create_client(
            YoloDetect, YOLO_DETECTION_TOPIC, callback_group=self.callback_group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.pose_detection = PoseDetection()
        self.output_image = []
        self.people = []

        self.get_logger().info("GPSRCommands Ready.")
        self.create_timer(0.1, self.publish_image, callback_group=self.callback_group)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )

        # Areas of the active map
        self.areas = None
        self.areas_client = self.create_client(
            MapAreas, AREAS_SERVICE, callback_group=self.callback_group
        )

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(f"Error: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

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
        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

        pose_requested = request.pose_requested

        # Convert pose_requested to Enum Poses
        try:
            pose_requested_enum = Poses(pose_requested)
        except KeyError:
            self.get_logger().warn(f"Pose {pose_requested} is not valid.")
            response.success = False
            response.count = 0
            return response

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
        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

        gesture_requested = request.pose_requested

        # Convert gesture_requested to Enum Gestures
        try:
            gesture_requested_enum = Gestures(gesture_requested)
        except KeyError:
            self.get_logger().warn(f"Gesture {gesture_requested} is not valid.")
            response.success = False
            response.count = 0
            return response

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

        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

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
        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

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
        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

        clothing = request.clothing
        color = request.color

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
        self.people = self.get_detections(0)

        self.people = self._filter_people(frame, self.people)

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
        """Detect the gesture in the image."""
        gestures = [
            Gestures.UNKNOWN,
            Gestures.WAVING,
            Gestures.RAISING_LEFT_ARM,
            Gestures.RAISING_RIGHT_ARM,
            Gestures.POINTING_LEFT,
            Gestures.POINTING_RIGHT,
        ]

        self.people = self.get_detections(0)

        gesture = self.pose_detection.detectGesture(cropped_frame)

        if gesture in gestures:
            return gesture.value

        return Gestures.UNKNOWN.value

    def _dicts_to_detection_msgs(self, detections: list):
        from frida_interfaces.msg import Detection

        msgs = []
        for d in detections:
            msg = Detection()
            msg.x1, msg.y1, msg.x2, msg.y2 = d["bbox"]
            msg.confidence = d["confidence"]
            msg.class_id = d["class_id"]
            msgs.append(msg)
        return msgs

    def _get_areas(self):
        """Fetch the active map's areas from nav_central (cached after first call)."""
        if self.areas is None:
            self.areas = fetch_map_areas(self, self.areas_client, self.get_logger())
        return self.areas

    def _filter_people(self, frame, people):
        filtered = filter_detections_in_house(
            frame,
            people,
            [0],
            None,
            self.camera_info,
            self.depth_image,
            self.tf_buffer,
            self._get_areas(),
            CAMERA_FRAME,
        )
        self._draw_people(filtered)
        return filtered

    def _draw_people(self, people):
        """Draw the filtered person detections on output_image so the display
        (IMAGE_TOPIC via publish_image) shows what is actually being counted."""
        if len(self.output_image) == 0:
            return
        for person in people:
            x1, y1, x2, y2 = person["bbox"]
            cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                self.output_image,
                f"person {person['confidence']:.2f}",
                (x1, max(y1 - 10, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

    def get_detections(self, comp_class=None, timeout=5.0):
        """
        Obtain YOLO detections via the YOLO service.
        comp_class: int or None (None = detect all classes)
        """

        # Ensure YOLO service is available. The TRT engine deserializes on the
        # detector's first inference, so right after launch the service can
        # take several seconds to answer — give it a real chance instead of
        # instantly returning 0 detections (which reads as "0 people counted").
        if not self.yolo_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "YOLO service not available, returning no detections."
            )
            return []

        # Create request
        req = YoloDetect.Request()
        req.classes = [comp_class] if comp_class is not None else []

        # Call YOLO service
        future = self.yolo_client.call_async(req)

        # Wait for the future while spinning the node
        future = wait_for_future(future, 15)
        result = future.result()

        if result is None or not result.success:
            self.get_logger().error("YOLO detection failed")
            return []

        # Parse detections
        detections = []
        for det in result.detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            conf, cls_id = det.confidence, det.class_id
            detections.append(
                {
                    "bbox": (x1, y1, x2, y2),
                    "confidence": conf,
                    "class_id": cls_id,
                    "area": (x2 - x1) * (y2 - y1),
                }
            )

        # Store people if comp_class == 0
        if comp_class == 0:
            self.people = [d for d in detections if d["class_id"] == 0]

        return detections

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
        future = wait_for_future(future, 15)
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
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

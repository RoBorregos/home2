#!/usr/bin/env python3

import pathlib
import rclpy
import rclpy.qos
from vision_general.utils.trt_utils import load_yolo_trt
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DISHWASHER_LAYOUT_DETECTION_TOPIC,
    DISHWASHER_RACK_DETECTION_TOPIC,
    DISHWASHER_TABLET_DETECTION_TOPIC,
    OBJECT_POINTS_TOPIC,
    DISHWASHER_DEBUG_IMAGE_TOPIC,
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DishwasherDetection, ObjectPoints
from vision_general.utils.calculations import (
    point2d_to_ros_point_stamped,
    get2DCentroid,
)
from vision_general.utils.ros_utils import wait_for_future
import cv2


class DishwasherNode(Node):
    def __init__(self):
        super().__init__("dishwasher")
        self.get_logger().info("Dishwasher node initialized")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        MODELS_PATH = (
            pathlib.Path(__file__).resolve().parent.parent / "Utils" / "models"
        )

        self.layout_model = load_yolo_trt(str(MODELS_PATH / "dishwasher_layout.pt"))
        self.get_logger().info("Dishwasher layout model loaded")

        self.rack_model = load_yolo_trt(str(MODELS_PATH / "dishwasher_rack.pt"))
        self.get_logger().info("Rack model loaded")

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )

        self.image_depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.image_depth_callback, qos
        )

        self.dishwasher_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_LAYOUT_DETECTION_TOPIC,
            self.dishwasher_layout_callback,
            callback_group=self.callback_group,
        )

        self.rack_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_RACK_DETECTION_TOPIC,
            self.dishwasher_rack_callback,
            callback_group=self.callback_group,
        )

        self.tablet_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_TABLET_DETECTION_TOPIC,
            self.dishwasher_tablet_callback,
            callback_group=self.callback_group,
        )

        self.moondream_client = self.create_client(
            ObjectPoints, OBJECT_POINTS_TOPIC, callback_group=self.callback_group
        )

        self.debug_publisher = self.create_publisher(
            Image, DISHWASHER_DEBUG_IMAGE_TOPIC, 10
        )

        self.image = None

    def publish_debug_image(self, detections):
        if self.image is None:
            return

        debug_image = self.image.copy()

        for det in detections:
            # draw bounding box if x1 != x2 and y1 != y2
            if int(det.xmin) != int(det.xmax) and int(det.ymin) != int(det.ymax):
                cv2.rectangle(
                    debug_image,
                    (int(det.xmin), int(det.ymin)),
                    (int(det.xmax), int(det.ymax)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    debug_image,
                    f"{det.label_text} {det.score:.2f}",
                    (int(det.xmin), int(det.ymin) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            else:
                # it's a point (like the tablet)
                cv2.circle(
                    debug_image, (int(det.xmin), int(det.ymin)), 5, (0, 0, 255), -1
                )
                cv2.putText(
                    debug_image,
                    f"{det.label_text}",
                    (int(det.xmin) - 20, int(det.ymin) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )

        self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_info_callback(self, msg):
        self.image_info = msg

    def image_depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def dishwasher_layout_callback(self, request, response):
        if self.image is None:
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            return response

        results = self.layout_model(
            source=self.image,
            conf=0.35,
            verbose=False,
        )

        response.detection_array = ObjectDetectionArray()
        response.detection_array.detections = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = result.names.get(class_id, str(class_id))
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0].item())

                object_detection = ObjectDetection(
                    label=class_id,
                    label_text=class_name,
                    score=confidence,
                    xmin=x1,
                    ymin=y1,
                    xmax=x2,
                    ymax=y2,
                )
                response.detection_array.detections.append(object_detection)

        self.publish_debug_image(response.detection_array.detections)
        response.success = True
        return response

    def dishwasher_rack_callback(self, request, response):
        if self.image is None or self.depth_image is None or self.image_info is None:
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            return response

        results = self.rack_model(
            source=self.image,
            conf=0.35,
            verbose=False,
        )

        response.detection_array = ObjectDetectionArray()
        response.detection_array.detections = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = result.names.get(class_id, str(class_id))
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0].item())

                point_2D = get2DCentroid(
                    [y1, x1, y2, x2],
                    self.image,
                )

                point_3D = point2d_to_ros_point_stamped(
                    self.image_info,
                    self.depth_image,
                    point_2D,
                    CAMERA_FRAME,
                    self.get_clock().now().to_msg(),
                )

                object_detection = ObjectDetection(
                    label=class_id,
                    label_text=class_name,
                    score=confidence,
                    xmin=x1,
                    ymin=y1,
                    xmax=x2,
                    ymax=y2,
                    point3d=point_3D,
                )

                response.detection_array.detections.append(object_detection)

        self.publish_debug_image(response.detection_array.detections)
        response.success = True
        return response

    def dishwasher_tablet_callback(self, request, response):
        if self.image is None or self.depth_image is None or self.image_info is None:
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            self.get_logger().error("No image, depth, or camera info received")
            return response

        req = ObjectPoints.Request()
        req.subject = "A small, clear plastic pouch containing a blue and green laundry detergent packet rests on a plain off-white surface. The packet features a white swirl design on a blue and green background."

        future = self.moondream_client.call_async(req)
        future = wait_for_future(future, 15)

        if future is False or not future.done():
            self.get_logger().error(
                "MoonDream service call timed out or failed for dishwasher tablet"
            )
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            return response

        result = future.result()

        response.detection_array = ObjectDetectionArray()
        response.detection_array.detections = []

        if result is None or not result.success or len(result.points) == 0:
            self.get_logger().error(
                "MoonDream tablet point detection failed or no points found"
            )
            response.success = False
            return response

        for i, point in enumerate(result.points):
            # Moondream returns normalized coords, need to scale to image dims
            u = int(point.x * self.image_info.width)
            v = int(point.y * self.image_info.height)

            point_2D = (u, v)

            point_3D = point2d_to_ros_point_stamped(
                self.image_info,
                self.depth_image,
                point_2D,
                CAMERA_FRAME,
                self.get_clock().now().to_msg(),
            )

            object_detection = ObjectDetection(
                label=0,  # generic id
                label_text="tablet",
                score=1.0,
                xmin=float(u),
                ymin=float(v),
                xmax=float(u),
                ymax=float(v),
                point3d=point_3D,
            )

            response.detection_array.detections.append(object_detection)

        self.publish_debug_image(response.detection_array.detections)
        response.success = True
        return response


if __name__ == "__main__":
    rclpy.init()
    dishwasher_node = DishwasherNode()
    rclpy.spin(dishwasher_node)
    rclpy.shutdown()

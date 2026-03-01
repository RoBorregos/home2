#! /usr/bin/env python3
import rclpy
import rclpy.duration
import rclpy.node
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from frida_interfaces.msg import ObjectDetectionArray
from dataclasses import dataclass
import pathlib
import threading
import copy
from typing import List
import cv2 as cv
from detectors.YoloV5ObjectDetector import YoloV5ObjectDetector
from detectors.YoloV8ObjectDetector import YoloV8ObjectDetector
from detectors.ObjectDetector import Detection, ObjectDectectorParams
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    DETECTIONS_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
    DETECTIONS_POSES_TOPIC,
    DETECTIONS_3D_TOPIC,
    DETECTIONS_ACTIVE_TOPIC,
    DEBUG_IMAGE_TOPIC,
    CAMERA_FRAME,
)

MODELS_PATH = str(pathlib.Path(__file__).parent) + "/models/"

print("PATH   ", MODELS_PATH)
ARGS = {
    "RGB_IMAGE_TOPIC": CAMERA_TOPIC,
    "DEPTH_IMAGE_TOPIC": DEPTH_IMAGE_TOPIC,
    "CAMERA_INFO_TOPIC": CAMERA_INFO_TOPIC,
    "DETECTIONS_TOPIC": DETECTIONS_TOPIC,
    "DETECTIONS_IMAGE_TOPIC": DETECTIONS_IMAGE_TOPIC,
    "DETECTIONS_POSES_TOPIC": DETECTIONS_POSES_TOPIC,
    "DETECTIONS_3D_TOPIC": DETECTIONS_3D_TOPIC,
    "DETECTIONS_ACTIVE_TOPIC": DETECTIONS_ACTIVE_TOPIC,
    "DEBUG_IMAGE_TOPIC": DEBUG_IMAGE_TOPIC,
    "CAMERA_FRAME": CAMERA_FRAME,
    "TARGET_FRAME": "base_link",
    "YOLO_MODEL_PATH": "tmr_30classes_v2.pt",
    "USE_ACTIVE_FLAG": False,
    "DEPTH_ACTIVE": True,
    "VERBOSE": False,
    "USE_YOLO8": True,
    "FLIP_IMAGE": False,
    "USE_ZED_TRANSFORM": True,
    "MIN_SCORE_THRESH": 0.3,
}


@dataclass
class NodeParams:
    RGB_IMAGE_TOPIC: str = None
    DEPTH_IMAGE_TOPIC: str = None
    CAMERA_INFO_TOPIC: str = None
    DETECTIONS_TOPIC: str = None
    DETECTIONS_POSES_TOPIC: str = None
    DETECTIONS_3D_TOPIC: str = None
    DETECTIONS_ACTIVE_TOPIC: str = None
    DETECTIONS_IMAGE_TOPIC: str = None
    DEBUG_IMAGE_TOPIC: str = None
    YOLO_MODEL_PATH: str = None
    USE_ACTIVE_FLAG: bool = None
    VERBOSE: bool = None
    USE_YOLO8: bool = None
    USE_YOLO26: bool = None


# TODO DEFINE HOW TO GET params


class object_detector_node(rclpy.node.Node):
    def __init__(self, node_name: str = "object_detector_2D_node"):
        super().__init__(node_name)

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.camera_info = CameraInfo()
        self.detections_frame = []

        self.set_parameters()
        self.active_flag = not self.node_params.USE_ACTIVE_FLAG

        if self.node_params.USE_YOLO26 or self.node_params.USE_YOLO8:
            self.object_detector_2d = YoloV8ObjectDetector(
                self.node_params.YOLO_MODEL_PATH, self.object_detector_parameters
            )
        else:
            self.object_detector_2d = YoloV5ObjectDetector(
                self.node_params.YOLO_MODEL_PATH, self.object_detector_parameters
            )

        self.handleSubcriptions()
        self.handlePublishers()
        self.runThread = None

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Frames per second throughput estimator
        self.curr_clock = 0

        self.get_logger().info("Object Detector 2D Node has been started")

    def set_parameters(self):
        self.object_detector_parameters = ObjectDectectorParams()
        self.object_detector_parameters.depth_active = (
            self.get_parameter("DEPTH_ACTIVE").get_parameter_value().bool_value
        )
        self.object_detector_parameters.min_score_thresh = (
            self.get_parameter("MIN_SCORE_THRESH").get_parameter_value().double_value
        )
        self.object_detector_parameters.target_frame = (
            self.get_parameter("TARGET_FRAME").get_parameter_value().string_value
        )
        self.object_detector_parameters.camera_frame = (
            self.get_parameter("CAMERA_FRAME").get_parameter_value().string_value
        )
        self.object_detector_parameters.flip_image = (
            self.get_parameter("FLIP_IMAGE").get_parameter_value().bool_value
        )
        self.object_detector_parameters.use_zed_transfrom = (
            self.get_parameter("USE_ZED_TRANSFORM").get_parameter_value().bool_value
        )

        self.node_params = NodeParams()
        self.node_params.RGB_IMAGE_TOPIC = (
            self.get_parameter("RGB_IMAGE_TOPIC").get_parameter_value().string_value
        )
        self.node_params.DEPTH_IMAGE_TOPIC = (
            self.get_parameter("DEPTH_IMAGE_TOPIC").get_parameter_value().string_value
        )
        self.node_params.CAMERA_INFO_TOPIC = (
            self.get_parameter("CAMERA_INFO_TOPIC").get_parameter_value().string_value
        )
        self.node_params.DETECTIONS_TOPIC = (
            self.get_parameter("DETECTIONS_TOPIC").get_parameter_value().string_value
        )
        self.node_params.DETECTIONS_POSES_TOPIC = (
            self.get_parameter("DETECTIONS_POSES_TOPIC")
            .get_parameter_value()
            .string_value
        )
        self.node_params.DETECTIONS_3D_TOPIC = (
            self.get_parameter("DETECTIONS_3D_TOPIC").get_parameter_value().string_value
        )
        self.node_params.DETECTIONS_ACTIVE_TOPIC = (
            self.get_parameter("DETECTIONS_ACTIVE_TOPIC")
            .get_parameter_value()
            .string_value
        )
        self.node_params.DETECTIONS_IMAGE_TOPIC = (
            self.get_parameter("DETECTIONS_IMAGE_TOPIC")
            .get_parameter_value()
            .string_value
        )
        self.node_params.DEBUG_IMAGE_TOPIC = (
            self.get_parameter("DEBUG_IMAGE_TOPIC").get_parameter_value().string_value
        )
        self.node_params.YOLO_MODEL_PATH = MODELS_PATH + (
            self.get_parameter("YOLO_MODEL_PATH").get_parameter_value().string_value
        )
        self.get_logger().info(f"path: {self.node_params.YOLO_MODEL_PATH}")
        self.node_params.USE_ACTIVE_FLAG = (
            self.get_parameter("USE_ACTIVE_FLAG").get_parameter_value().bool_value
        )
        self.node_params.VERBOSE = (
            self.get_parameter("VERBOSE").get_parameter_value().bool_value
        )
        self.node_params.USE_YOLO8 = (
            self.get_parameter("USE_YOLO8").get_parameter_value().bool_value
        )
        self.node_params.USE_YOLO26 = (
            self.get_parameter("USE_YOLO26").get_parameter_value().bool_value
        )
        self.get_logger().info(
            "Listening to image on topic: " + self.node_params.RGB_IMAGE_TOPIC
        )

    def handlePublishers(self):
        self.detections_publisher = self.create_publisher(
            ObjectDetectionArray, self.node_params.DETECTIONS_TOPIC, 5
        )
        self.detections_pose_publisher = self.create_publisher(
            PoseArray, self.node_params.DETECTIONS_POSES_TOPIC, 5
        )
        self.detections_3d = self.create_publisher(
            MarkerArray, self.node_params.DETECTIONS_3D_TOPIC, 5
        )
        self.detections_image_publisher = self.create_publisher(
            Image, self.node_params.DETECTIONS_IMAGE_TOPIC, 5
        )
        self.debug_image_publisher = self.create_publisher(
            Image, self.node_params.DEBUG_IMAGE_TOPIC, 5
        )

        if self.node_params.VERBOSE:
            self.get_logger().info(
                "Publishers have been created with the following topics: "
            )
            self.get_logger().info(self.node_params.DETECTIONS_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_POSES_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_3D_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_IMAGE_TOPIC)
            self.get_logger().info(self.node_params.DEBUG_IMAGE_TOPIC)

    def handleSubcriptions(self):
        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        if self.node_params.VERBOSE:
            self.get_logger().info(
                "Subcribers have been created with the following topics: "
            )
            self.get_logger().info(self.node_params.RGB_IMAGE_TOPIC)
        self.rgb_image_sub = self.create_subscription(
            Image, self.node_params.RGB_IMAGE_TOPIC, self.rgbImageCallback, qos
        )
        if self.object_detector_parameters.depth_active:
            self.depth_subscriber = self.create_subscription(
                Image, self.node_params.DEPTH_IMAGE_TOPIC, self.depthImageCallback, qos
            )
            self.recieved_camera_info = False
            self.camera_info_subscriber = self.create_subscription(
                CameraInfo,
                self.node_params.CAMERA_INFO_TOPIC,
                self.cameraInfoCallback,
                qos,
            )

            if self.node_params.VERBOSE:
                self.get_logger().info(self.node_params.DEPTH_IMAGE_TOPIC)
                self.get_logger().info(self.node_params.CAMERA_INFO_TOPIC)

        if self.node_params.USE_ACTIVE_FLAG:
            self.create_subscription(
                Bool,
                self.node_params.DETECTIONS_ACTIVE_TOPIC,
                self.activeFlagCallback,
                5,
            )
            if self.node_params.VERBOSE:
                self.get_logger().info(self.node_params.DETECTIONS_ACTIVE_TOPIC)

    # Callback for active flag
    def activeFlagCallback(self, msg):
        self.activeFlag = msg.data

    # Function to handle a ROS depthPublishers have been created input.
    def depthImageCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            if self.node_params.VERBOSE:
                self.get_logger().info("SUCCESS: Recieved depth image")
        except CvBridgeError as e:
            if self.node_params.VERBOSE:
                self.get_logger().error(f"Failed to recieve depth image, exception{e}")

    # Function to handle ROS camera info input.
    def cameraInfoCallback(self, data):
        if not self.recieved_camera_info:
            self.object_detector_2d.object_detector_params_.camera_info = data
            self.recieved_camera_info = True
            if self.node_params.VERBOSE:
                self.get_logger().info("SUCCESS: Recieved camera info")

    def rgbImageCallback(self, data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.curr_clock = data.header.stamp
        if not self.active_flag:
            self.detections_frame = self.rgb_image
        elif (
            self.active_flag and self.runThread is None or not self.runThread.is_alive()
        ):
            self.runThread = threading.Thread(
                target=self.run, args=(self.rgb_image,), daemon=True
            )
            self.runThread.start()

        if self.node_params.VERBOSE:
            self.get_logger().info("SUCCESS: Recieved rgb image")

        if len(self.detections_frame) > 0:
            self.detections_image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8")
            )

    def visualize_detections(
        self,
        image,
        detections: List[Detection],
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        agnostic_mode=False,
    ):
        """Visualize detections on an input image."""

        # Convert image to BGR format (OpenCV uses BGR instead of RGB)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)

        for detection in detections:
            color = (0, 0, 0) if agnostic_mode else (0, 255, 0)
            ymin, xmin, ymax, xmax = (
                detection.bbox_.y1,
                detection.bbox_.x1,
                detection.bbox_.y2,
                detection.bbox_.x2,
            )
            if use_normalized_coordinates:
                (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
            else:
                (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
            (left, right, top, bottom) = (
                int(left * image.shape[1]),
                int(right * image.shape[1]),
                int(top * image.shape[0]),
                int(bottom * image.shape[0]),
            )
            cv.rectangle(image, (left, top), (right, bottom), color, 7)
            # draw label, name and score
            cv.putText(
                image,
                f"{detection.class_id_}: {detection.label_}: {detection.confidence_}",
                (left, top - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                2,
                color,
                2,
            )
            # draw centroid
            cv.circle(
                image,
                (int((left + right) / 2), int((top + bottom) / 2)),
                5,
                (0, 0, 255),
                -1,
            )

        # Convert image back to RGB format
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        return image

    def visualize_3d_detections(self, detections):
        marker_array = MarkerArray()

        for index in range(len(detections)):
            marker = Marker()
            marker.header.frame_id = self.object_detector_parameters.camera_frame
            marker.header.stamp = self.curr_clock
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = index
            marker.pose.position.x = detections[index].point_stamped_.point.x
            marker.pose.position.y = detections[index].point_stamped_.point.y
            marker.pose.position.z = detections[index].point_stamped_.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            marker_array.markers.append(marker)

        self.detections_3d.publish(marker_array)

    def run(self, frame):
        copy_frame = copy.deepcopy(frame)
        detected_objects, visual_detections, visual_image = (
            self.object_detector_2d.inference(
                copy_frame, self.depth_image, self.tfBuffer
            )
        )

        self.detections_frame = self.visualize_detections(
            visual_image,
            visual_detections,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            agnostic_mode=False,
        )

        if self.node_params.VERBOSE:
            for index in range(len(detected_objects)):
                self.get_logger().info(
                    f"Detection #{str(index)}:   {detected_objects[index].__str__()}"
                )

        self.visualize_3d_detections(detected_objects)

        self.detections_pose_publisher.publish(
            PoseArray(
                poses=[
                    Pose(
                        position=Point(
                            x=detection.point_stamped_.point.x,
                            y=detection.point_stamped_.point.y,
                            z=detection.point_stamped_.point.z,
                        )
                    )
                    for detection in detected_objects
                ]
            )
        )

        # update time
        for detection in self.object_detector_2d.getFridaDetections(detected_objects):
            detection.point3d.header.stamp = self.curr_clock
        self.detections_publisher.publish(
            ObjectDetectionArray(
                detections=self.object_detector_2d.getFridaDetections(detected_objects)
            )
        )


def main(args=None):
    rclpy.init(args=args)
    object_detector_2d = object_detector_node("object_detector_2D_node")
    rclpy.spin(object_detector_2d)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

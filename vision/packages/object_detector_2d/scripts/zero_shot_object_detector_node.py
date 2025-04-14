#! /usr/bin/env python3
import rclpy
import rclpy.duration
import rclpy.node
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray
from frida_interfaces.msg import ObjectDetectionArray
from frida_interfaces.srv import SetDetectorClasses
from dataclasses import dataclass
import pathlib
import threading
from detectors.YoloEObjectDetector import YoloEObjectDetector
from detectors.ObjectDetector import ObjectDectectorParams
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
    ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
    ZERO_SHOT_DETECTIONS_POSES_TOPIC,
    ZERO_SHOT_DETECTIONS_3D_TOPIC,
    ZERO_SHOT_DETECTIONS_ACTIVE_TOPIC,
    SET_DETECTOR_CLASSES_SERVICE,
    CAMERA_FRAME,
)
from object_detector_node import object_detector_node

MODELS_PATH = str(pathlib.Path(__file__).parent) + "/../models/"

ARGS = {
    "RGB_IMAGE_TOPIC": CAMERA_TOPIC,
    "DEPTH_IMAGE_TOPIC": DEPTH_IMAGE_TOPIC,
    "CAMERA_INFO_TOPIC": CAMERA_INFO_TOPIC,
    "DETECTIONS_TOPIC": ZERO_SHOT_DETECTIONS_TOPIC,
    "DETECTIONS_IMAGE_TOPIC": ZERO_SHOT_DETECTIONS_IMAGE_TOPIC,
    "DETECTIONS_POSES_TOPIC": ZERO_SHOT_DETECTIONS_POSES_TOPIC,
    "DETECTIONS_3D_TOPIC": ZERO_SHOT_DETECTIONS_3D_TOPIC,
    "DETECTIONS_ACTIVE_TOPIC": ZERO_SHOT_DETECTIONS_ACTIVE_TOPIC,
    "SET_DETECTOR_CLASSES_SERVICE": SET_DETECTOR_CLASSES_SERVICE,
    "CAMERA_FRAME": CAMERA_FRAME,
    "TARGET_FRAME": "base_link",
    "YOLO_MODEL_PATH": MODELS_PATH + "yoloe-11l-seg.pt",
    "USE_ACTIVE_FLAG": False,
    "DEPTH_ACTIVE": True,
    "VERBOSE": False,
    "USE_YOLO8": False,
    "FLIP_IMAGE": False,
    "USE_ZED_TRANSFORM": True,
    "MIN_SCORE_THRESH": 0.25,
    "CLASSES": [
        "handbag",
    ],
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
    SET_DETECTOR_CLASSES_SERVICE: str = None


# TODO DEFINE HOW TO GET params


class zero_shot_object_detector_node(object_detector_node):
    def __init__(self, node_name: str = "zero_shot_object_detector_2D_node"):
        super(object_detector_node, self).__init__(node_name)

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.camera_info = CameraInfo()
        self.detections_frame = []

        self.set_parameters()
        self.active_flag = not self.node_params.USE_ACTIVE_FLAG

        self.object_detector_2d = YoloEObjectDetector(
            self.node_params.YOLO_MODEL_PATH, self.object_detector_parameters
        )
        self.object_detector_2d.set_classes(self.active_classes)

        self.handleSubcriptions()
        self.handlePublishers()
        # self.handleServices()
        self.runThread = None

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Frames per second throughput estimator
        self.curr_clock = 0
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

        self.get_logger().info("Object Detector 2D Node has been started")

    def set_parameters(self):
        self.active_classes = (
            self.get_parameter("CLASSES").get_parameter_value().string_array_value
        )

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
        self.node_params.YOLO_MODEL_PATH = (
            MODELS_PATH
            + self.get_parameter("YOLO_MODEL_PATH").get_parameter_value().string_value
        )
        self.node_params.USE_ACTIVE_FLAG = (
            self.get_parameter("USE_ACTIVE_FLAG").get_parameter_value().bool_value
        )
        self.node_params.VERBOSE = (
            self.get_parameter("VERBOSE").get_parameter_value().bool_value
        )
        self.node_params.USE_YOLO8 = (
            self.get_parameter("USE_YOLO8").get_parameter_value().bool_value
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

        if self.node_params.VERBOSE:
            self.get_logger().info(
                "Publishers have been created with the following topics: "
            )
            self.get_logger().info(self.node_params.DETECTIONS_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_POSES_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_3D_TOPIC)
            self.get_logger().info(self.node_params.DETECTIONS_IMAGE_TOPIC)
            self.get_logger().info(self.node_params.DEBUG_IMAGE_TOPIC)

    def set_detector_classes_callback(self, request, response):
        """Set the classes for the object detector.

        Args:
            request (SetDetectorClasses.Request): Request object containing the classes.

        Returns:
            SetDetectorClasses.Response: Response object.
        """
        response = SetDetectorClasses.Response()
        self.object_detector_2d.set_classes(request.classes)
        return response

    def handleServices(self):
        object.set_detector_classes_service = self.create_service(
            SetDetectorClasses,
            self.node_params.SET_DETECTOR_CLASSES_SERVICE,
            self.set_detector_classes_callback,
        )

        if self.node_params.VERBOSE:
            self.get_logger().info(
                f"Service {self.node_params.SET_DETECTOR_CLASSES_SERVICE} has been created"
            )


def main(args=None):
    rclpy.init(args=args)
    object_detector_2d = zero_shot_object_detector_node()
    rclpy.spin(object_detector_2d)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

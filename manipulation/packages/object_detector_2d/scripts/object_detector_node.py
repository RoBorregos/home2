import rclpy
import rclpy.node
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray

from imutils.video import FPS
from YoloV5ObjectDetector import YoloV5ObjectDetector
from ObjectDetector import ObjectDectectorParams, Detection, BBOX
from dataclasses import dataclass
import pathlib
import threading
import copy
from typing import List
import cv2 as cv


ARGS = {
    "RGB_IMAGE_TOPIC" : "/raw_image",
    "DEPTH_IMAGE_TOPIC" :  "/camera/depth/image_raw",
    "CAMERA_INFO_TOPIC" :  "/camera/depth/camera_info",
    "DETECTIONS_TOPIC" : "/detections",
    "DETECTIONS_IMAGE_TOPIC" : "/detections_image",
    "DETECTIONS_POSES_TOPIC" : "/test/detection_poses",
    "DETECTIONS_3D_TOPIC" : "/detections_3d",
    "DETECTIONS_ACTIVE_TOPIC" : "/detections_active",
    "DEBUG_IMAGE_TOPIC" : "/debug_image",
    "CAMERA_FRAME" :  "xtion_rgb_optical_frame",
    "YOLO_MODEL_PATH" :  str(pathlib.Path(__file__).parent) + "/../models/yolov5s.pt",
    "USE_ACTIVE_FLAG" :  True,
    "DEPTH_ACTIVE" :  False,
    "VERBOSE" :  False,
    "USE_YOLO8" :  False,
    "FLIP_IMAGE" :  False,
    "MIN_SCORE_THRESH" :  0.75,
}

@dataclass
class NodeParams():
    RGB_IMAGE_TOPIC : str
    DEPTH_IMAGE_TOPIC : str
    CAMERA_INFO_TOPIC : str
    DETECTIONS_TOPIC : str
    DETECTIONS_POSES_TOPIC : str
    DETECTIONS_3D_TOPIC : str
    DETECTIONS_ACTIVE_TOPIC : str
    DETECTIONS_IMAGE_TOPIC : str
    DEBUG_IMAGE_TOPIC : str
    YOLO_MODEL_PATH : str
    USE_ACTIVE_FLAG : bool
    VERBOSE : bool
    USE_YOLO8 : bool


#TODO DEFINE HOW TO GET params

class object_detector_node(rclpy.node.Node):
    def __init__(self):
        super().__init__('object_detector_2D_node')

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.camera_info = CameraInfo()

        self.set_parameters()
        self.active_flag = not self.node_params.USE_ACTIVE_FLAG


        if self.node_params.USE_YOLO8:
            self.object_detector_2d = None
        else: 
            self.object_detector_2d = YoloV5ObjectDetector(self.node_params.YOLO_MODEL_PATH, self.object_detector_parameters)

        self.handleSubcriptions()
        self.handlePublishers()
        self.runThread = None

        # TFs
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

    def set_parameters(self): 
        self.object_detector_parameters = ObjectDectectorParams()
        self.object_detector_parameters.depth_active = self.get_parameter("DEPTH_ACTIVE").get_parameter_value().bool_value
        self.object_detector_parameters.camera_info = self.get_parameter("CAMERA_INFO").get_parameter_value().string_value
        self.object_detector_parameters.min_score_thresh = self.get_parameter("MIN_SCORE_THRESH").get_parameter_value().double_value
        self.object_detector_parameters.camera_frame = self.get_parameter("CAMERA_FRAME").get_parameter_value().string_value
        self.object_detector_parameters.flip_image = self.get_parameter("FLIP_IMAGE").get_parameter_value().bool_value

        self.node_params = NodeParams()
        self.node_params.RGB_IMAGE_TOPIC = self.get_parameter("RGB_IMAGE_TOPIC").get_parameter_value().string_value
        self.node_params.DEPTH_IMAGE_TOPIC = self.get_parameter("DEPTH_IMAGE_TOPIC").get_parameter_value().string_value
        self.node_params.CAMERA_INFO_TOPIC = self.get_parameter("CAMERA_INFO_TOPIC").get_parameter_value().string_value
        self.node_params.DETECTIONS_TOPIC = self.get_parameter("DETECTIONS_TOPIC").get_parameter_value().string_value
        self.node_params.DETECTIONS_POSES_TOPIC = self.get_parameter("DETECTIONS_POSES_TOPIC").get_parameter_value().string_value
        self.node_params.DETECTIONS_3D_TOPIC = self.get_parameter("DETECTIONS_3D_TOPIC").get_parameter_value().string_value
        self.node_params.DETECTIONS_ACTIVE_TOPIC = self.get_parameter("DETECTIONS_ACTIVE_TOPIC").get_parameter_value().string_value
        self.node_params.DETECTIONS_IMAGE_TOPIC = self.get_parameter("DETECTIONS_IMAGE_TOPIC").get_parameter_value().string_value
        self.node_params.DEBUG_IMAGE_TOPIC = self.get_parameter("DEBUG_IMAGE_TOPIC").get_parameter_value().string_value
        self.node_params.YOLO_MODEL_PATH = self.get_parameter("YOLO_MODEL_PATH").get_parameter_value().string_value
        self.node_params.USE_ACTIVE_FLAG = self.get_parameter("USE_ACTIVE_FLAG").get_parameter_value().bool_value
        self.node_params.VERBOSE = self.get_parameter("VERBOSE").get_parameter_value().bool_value
        self.node_params.USE_YOLO8 = self.get_parameter("USE_YOLO8").get_parameter_value().bool_value
        
    def handlePublishers(self):
        self.detections_publisher = self.create_publisher(
            self.node_params.DETECTIONS_TOPIC, ObjectDetectionArray, 5
        )
        self.detections_pose_publisher = self.create_publisher(
            self.node_params.DETECTIONS_POSES_TOPIC, PoseArray, 5
        )
        self.detections_3d = self.create_publisher(
            self.node_params.DETECTIONS_3D_TOPIC, MarkerArray, 5
        )
        self.detections_image_publisher = self.create_publisher(
            self.node_params.DETECTIONS_IMAGE_TOPIC, Image, 5
        )
        self.debug_image_publisher = self.create_publisher(
            self.node_params.DEBUG_IMAGE_TOPIC, Image, 5
        )
        
    def handleSubcriptions(self):
        self.rgb_image_sub = self.create_subscription(
            self.node_params.RGB_IMAGE_TOPIC, Image, self.rgbImageCallback
        )
        if self.object_detector_parameters.depth_active:
            self.depth_subscriber = self.create_subscription(
                self.node_params.DEPTH_IMAGE_TOPIC, Image, self.depthImageCallback
            )
            self.recieved_camera_info = False
            self.camera_info_subscriber = self.create_subscription(
                self.node_params.CAMERA_INFO_TOPIC, CameraInfo, self.cameraInfoCallback
            )
        
        if self.node_params.USE_ACTIVE_FLAG:
            self.create_subscription(
                self.node_params.DETECTIONS_ACTIVE_TOPIC, Bool, self.activeFlagCallback
            )

    # Callback for active flag
    def activeFlagCallback(self, msg):
        self.activeFlag = msg.data

    # Function to handle a ROS depth input.
    def depthImageCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle ROS camera info input.
    def cameraInfoCallback(self, data):
        if not self.recieved_camera_info:
            self.object_detector_2d.object_detector_params_.camera_info = data

    def rgbImageCallback(self, data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if not self.active_flag:
            self.detections_frame = self.rgb_image
        elif self.runThread == None or not self.runThread.is_alive():
            self.runThread = threading.Thread(target=self.run, args=(self.rgb_image,), daemon=True)
            self.runThread.start()

    # Handle FPS calculation.
    def callFps(self):
        if self.fps != None:
            self.fps.stop()
            if ARGS["VERBOSE"]:
                print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
            self.fpsValue = self.fps.fps()

        self.fps = FPS().start()

        callFpsThread = threading.Timer(2.0, self.callFps, args=())
        callFpsThread.start()

    def visualize_detections(
        self,
        image,
        detections : List[Detection],
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        agnostic_mode=False,
    ):
        """Visualize detections on an input image."""

        # Convert image to BGR format (OpenCV uses BGR instead of RGB)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)

        for detection in detections:
            color = (0,0,0) if agnostic_mode else (0, 255, 0)
            ymin, xmin, ymax, xmax = detection.bbox_.y1, detection.bbox_.x1, detection.bbox_.y2, detection.bbox_.x2
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
            cv.rectangle(image, (left, top), (right, bottom), color, 2)
                # draw label, name and score
            cv.putText(
                image,
                f"{detection.class_id_}: {detection.label_}: {detection.confidence_}",
                (left, top - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
            )
            # draw centroid
            cv.circle(
                image, (int((left + right) / 2), int((top + bottom) / 2)),
                5, (0, 0, 255), -1,
            )

        # Convert image back to RGB format
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        return image

    def run(self, frame):
        copy_frame = copy.deepcopy(frame)

        detected_objects, visual_detections, visual_image = self.object_detector_2d.inference(copy_frame)

        self.detections_frame = self.visualize_detections(
            visual_image,
            visual_detections,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            agnostic_mode=False,
        )
        
        self.detections_publisher.publish(ObjectDetectionArray(detections=detected_objects))
        self.fps.update()

def main(args=None):
    rclpy.init(args=args)
    object_detector_2d = object_detector_node()
    rclpy.spin(object_detector_2d)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

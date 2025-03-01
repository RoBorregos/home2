#!/usr/bin/env python3
# Published detections in 2D and 3D with a specified model.

import numpy as np

# import tensorflow as tf
import cv2
import pathlib
import rclpy
import threading
from imutils.video import FPS
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from frida_interfaces.manipulation.msg import ObjectDetection
import sys

sys.path.append(str(pathlib.Path(__file__).parent) + "/../include")
from vision_utils import *
import tf2_ros
import copy

SOURCES = {
    "VIDEO": str(pathlib.Path(__file__).parent) + "/../resources/test.mp4",
    "CAMERA": 0,
    "ROS_IMG": "/camaras/0",
}

ARGS = {
    "SOURCE": SOURCES["VIDEO"],
    "ROS_INPUT": False,
    "USE_ACTIVE_FLAG": True,
    "DEPTH_ACTIVE": False,
    "DEPTH_INPUT": "/camera/depth/image_raw",
    "CAMERA_INFO": "/camera/depth/camera_info",
    "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/",
    "LABELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/label_map.pbtxt",
    "MIN_SCORE_THRESH": 0.8,
    "VERBOSE": True,
    "CAMERA_FRAME": "xtion_rgb_optical_frame",
    "USE_YOLO8": False,
    "YOLO_MODEL_PATH": str(pathlib.Path(__file__).parent) + "/../models/yolov5s.pt",
    "FLIP_IMAGE": False,
}


class DetectionPicker:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.imageInfo = CameraInfo()

        cv2.namedWindow("Detection Selection", cv2.WINDOW_NORMAL)
        # left click to select object
        cv2.setMouseCallback("Detection Selection", self.select_object)
        print("STARTED WINDOW")

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

        self.frame_available = False

        self.subscriber = None
        self.handleSource()
        self.selected_point_pub = rclpy.Publisher(
            "/debug/selected_detection", ObjectDetection, queue_size=5
        )
        self.image_publisher = rclpy.Publisher("detections_image", Image, queue_size=5)
        # to visualize the 3d points of detected objects
        self.marker_3d_publisher = rclpy.Publisher(
            "/debug/selected_point_3d", MarkerArray, queue_size=5
        )

        # TFs
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        print("FINISHED INIT")

        self.run()

        # Show OpenCV window.

        # try:
        #     self.detections_frame = []
        #     rate = rclpy.Rate(60)
        #     while not rclpy.is_shutdown():
        #         if ARGS["VERBOSE"] and len(self.detections_frame) != 0:
        #             cv2.imshow("Detections", self.detections_frame)
        #             cv2.waitKey(1)

        #         if len(self.detections_frame) != 0:
        #             self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8"))

        #         rate.sleep()
        # except KeyboardInterrupt:
        #     pass
        # cv2.destroyAllWindows()

    # Function to handle either a cv2 image or a ROS image.
    def handleSource(self):
        if ARGS["ROS_INPUT"]:
            self.subscriber = rclpy.Subscriber(
                ARGS["SOURCE"], Image, self.imageRosCallback
            )
            if ARGS["DEPTH_ACTIVE"]:
                self.subscriberDepth = rclpy.Subscriber(
                    ARGS["DEPTH_INPUT"], Image, self.depthImageRosCallback
                )
                self.subscriberInfo = rclpy.Subscriber(
                    ARGS["CAMERA_INFO"], CameraInfo, self.infoImageRosCallback
                )
        else:
            cThread = threading.Thread(target=self.cameraThread, daemon=True)
            cThread.start()

    # Function to handle a cv2 input.
    def cameraThread(self):
        cap = cv2.VideoCapture(ARGS["SOURCE"])
        frame = []
        rate = rclpy.Rate(30)
        try:
            while not rclpy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    continue
                if len(frame) == 0:
                    continue
                self.imageCallback(frame)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cap.release()

    # Function to handle a ROS input.
    def imageRosCallback(self, data):
        try:
            self.imageCallback(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle ROS camera info input.
    def infoImageRosCallback(self, data):
        self.imageInfo = data
        self.subscriberInfo.unregister()

    # Function to handle Rate Neckbottle, TF object detection model frame rate (<10FPS) against camera input (>30FPS).
    # Process a frame only when the script finishes the process of the previous frame, rejecting frames to keep real-time idea.
    def imageCallback(self, img):
        self.rgb_image = copy.deepcopy(img)
        self.frame_available = True

    # Function to handle the selection of an object in the image.
    def select_object(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"Selected point: {x}, {y}")
            # get 3d point
            point2D = np.array([x, y])
            depth = get_depth(self.depth_image, point2D)
            point3d = deproject_pixel_to_point(self.imageInfo, point2D, depth)
            print(f"3D Point: {point3d}")
            detection = ObjectDetection(
                label=-2,
                label_text="Picked Point",
                score=1.0,
                ymin=0,
                xmin=0,
                ymax=0,
                xmax=0,
                point3d=PointStamped(
                    header=Header(
                        frame_id=ARGS["CAMERA_FRAME"], stamp=rclpy.Time.now()
                    ),
                    point=Point(x=point3d[0], y=point3d[1], z=point3d[2]),
                ),
            )
            # Publish marker
            marker = Marker()
            marker.header.frame_id = ARGS["CAMERA_FRAME"]
            marker.header.stamp = rclpy.Time.now()
            marker.ns = "selected_point"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=point3d[0], y=point3d[1], z=point3d[2])
            marker.pose.orientation.w = 1.0
            marker.scale = Point(x=0.3, y=0.3, z=0.3)
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.Duration(1)
            marker_array = MarkerArray()
            marker_array.markers.append(marker)
            self.marker_3d_publisher.publish(marker_array)
            self.selected_point_pub.publish(detection)

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

    def run(self):
        try:
            while True:
                if self.frame_available:
                    self.frame_available = False
                    self.fps.update()
                    cv2.imshow("Detection Selection", self.rgb_image)
                    cv2.waitKey(1)
                    if self.fps != None:
                        self.fps.update()
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init_node("Vision2D_Picker", anonymous=True)
    for key in ARGS:
        ARGS[key] = rclpy.get_param("~" + key, ARGS[key])
    DetectionPicker()
    rclpy.spin()


if __name__ == "__main__":
    main()

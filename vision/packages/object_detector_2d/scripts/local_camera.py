#! /usr/bin/env python3
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dataclasses import dataclass
from frida_constants.vision_constants import LOCAL_CAMERA_TOPIC

ARGS = {"ROS_TOPIC": LOCAL_CAMERA_TOPIC, "HZ": 30}


@dataclass
class NodeParams:
    ROS_TOPIC: str = ""
    HZ: int = 0


class Camera(Node):
    def __init__(self):
        super().__init__("local_camera")

        for key, value in ARGS.items():
            self.declare_parameter(key, value)

        self.get_params()

        self.publisher = self.create_publisher(Image, self.node_params.ROS_TOPIC, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / self.node_params.HZ, self.callback)
        self.cap = cv.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error opening camera")
            return

    def callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error reading frame")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(msg)

    def get_params(self):
        self.node_params = NodeParams()

        self.node_params.ROS_TOPIC = (
            self.get_parameter("ROS_TOPIC").get_parameter_value().string_value
        )
        self.node_params.HZ = (
            self.get_parameter("HZ").get_parameter_value().integer_value
        )


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#! /usr/bin/env python3
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

ROS_TOPIC = 'raw_image'
ROS_NODE = 'local_camera'
HZ = 30
class Camera(Node):
    def __init__(self):
        super().__init__(ROS_NODE)
        self.publisher = self.create_publisher(Image, ROS_TOPIC, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/HZ, self.callback)
        self.cap = cv.VideoCapture(0)

        if not self.cap.isOpened():
            print('Error opening camera')
            return

    def callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print('Error reading frame')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
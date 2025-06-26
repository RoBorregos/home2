#!/usr/bin/env python3
import sys
import cv2
from frida_interfaces.srv import CountPersons
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class client_persons(Node):
    def __init__(self):
        super().__init__("count_persons_async")
        self.client = self.create_client(CountPersons, "count_persons")
        self.req = CountPersons.Request()
        self.bridge = CvBridge()

    def send_request_cv2(self, img):
        self.req.img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        return self.client.call_async(self.req)


def main():
    rclpy.init()
    img = cv2.imread(
        "vision/packages/vision_general/Utils/pose_test_images/5persons.png"
    )
    per_client = client_persons()
    future = per_client.send_request_cv2(img)
    rclpy.spin_until_future_complete(per_client, future)
    response = future.result()
    print("Personas detectadas:")
    print(response.result.data)


if __name__ == "__main__":
    main()

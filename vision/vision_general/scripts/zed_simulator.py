#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

"""
    Node that simulates the Zed camera by capturing 
    frames from the webcam and publishing them.
"""

PUBLISHER_TOPIC = "/zed2/zed_node/rgb/image_rect_color"


class ZedSimulator(Node):
    def __init__(self):
        super().__init__("zed_simulator")
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Image, PUBLISHER_TOPIC, 10)

        self.get_logger().info("ZedSimulator has started.")
        self.cap = cv2.VideoCapture(0)
        self.run()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("No frame")
                continue

            image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ZedSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

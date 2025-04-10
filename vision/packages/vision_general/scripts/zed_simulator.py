#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from frida_constants.vision_constants import CAMERA_TOPIC

"""
    Node that simulates the Zed camera by capturing 
    frames from the webcam and publishing them.
"""


class ZedSimulator(Node):
    def __init__(self):
        """Initialize the node and the camera source."""
        super().__init__("zed_simulator")
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Image, CAMERA_TOPIC, 10)

        self.get_logger().info("ZedSimulator has started.")
        self.video_id = self.declare_parameter("video_id", 0)
        self.use_zed = self.declare_parameter("use_zed", False)
        self.cap = cv2.VideoCapture(self.video_id.value)
        self.run()

    def run(self):
        """Get frames from the webcam and publish them."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("No frame")
                continue

            if self.use_zed:
                frame = frame[:, : frame.shape[1] // 2]

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

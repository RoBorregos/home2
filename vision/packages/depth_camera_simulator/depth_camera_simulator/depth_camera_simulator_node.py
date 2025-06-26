#!/usr/bin/env python3

import cv2
import imutils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from frida_constants.vision_constants import CAMERA_TOPIC, DEPTH_IMAGE_TOPIC
from depth_camera_simulator.depth_models.DepthAnythingModel import (
    DepthAnythingModel as DepthModel,
)
import time

"""
    Node that simulates the Zed camera by capturing 
    frames from the webcam and publishing them.
"""


class DepthCameraSimulator(Node):
    def __init__(self):
        """Initialize the node and the camera source."""
        super().__init__("depth_camera_simulator")
        self.bridge = CvBridge()

        self.get_logger().info("DepthCameraSimulator has started.")
        self.video_id = self.declare_parameter("video_id", 0)
        self.declare_parameter("height", -1)
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.declare_parameter("width", -1)
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.declare_parameter("quality", 75)
        self.quality = self.get_parameter("quality").get_parameter_value().integer_value
        self.extract_depth = self.declare_parameter("extract_depth", False)
        self.extract_depth = (
            self.get_parameter("extract_depth").get_parameter_value().bool_value
        )

        self.declare_parameter("visualize", False)
        self.visualize = (
            self.get_parameter("visualize").get_parameter_value().bool_value
        )

        self.img_publisher = self.create_publisher(Image, CAMERA_TOPIC, 10)
        self.img_comp_publisher = self.create_publisher(
            CompressedImage, CAMERA_TOPIC + "_compressed", 10
        )
        if self.extract_depth:
            self.load_depth_model()
            self.depth_image_publisher = self.create_publisher(
                Image, DEPTH_IMAGE_TOPIC, 10
            )
        self.cap = cv2.VideoCapture(self.video_id.value)
        self.run()

    def load_depth_model(self):
        """Load the depth model if extract_depth is True."""
        self.get_logger().info("Loading depth model...")
        self.depth_model = DepthModel()
        self.depth_model.load_model()
        self.get_logger().info("Depth model loaded successfully.")

    def run(self):
        """Get frames from the webcam and publish them."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("No frame")
                continue

            image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # Resize frame if height and/or width are specified
            if self.height > 0 or self.width > 0:
                resize_kwargs = {}
                if self.width > 0:
                    resize_kwargs["width"] = self.width
                if self.height > 0:
                    resize_kwargs["height"] = self.height
                frame = imutils.resize(frame, **resize_kwargs)

            self.img_publisher.publish(image)

            # Compress the frame and publish as ImageCompressed
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), getattr(self, "quality", 90)]
            start_time = time.time()
            ret, buffer = cv2.imencode(".jpg", frame, encode_param)
            if ret:
                img_comp_msg = CompressedImage()
                img_comp_msg.header = image.header
                img_comp_msg.format = "jpeg"
                img_comp_msg.data = buffer.tobytes()
                self.img_comp_publisher.publish(img_comp_msg)
            print(f"Time encoding image = {time.time() - start_time:.4f} seconds")

            if self.extract_depth:
                # Extract depth from the frame
                depth_image = self.depth_model.inference(frame)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_msg.header = image.header
                self.depth_image_publisher.publish(depth_msg)

            if self.visualize:
                if self.extract_depth:
                    cv2.imshow("Depth Visualization", depth_image)
                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

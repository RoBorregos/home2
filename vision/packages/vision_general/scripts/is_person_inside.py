#!/usr/bin/env python3

"""
Node to send the 3D coordinates of a person detected in the camera feed.
"""

from vision_general.utils.calculations import (
    get_depth,
    deproject_pixel_to_point,
)
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    PERSON_POINT_TOPIC,
    CAMERA_FRAME
)

class IsPersonInside(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.person_inside = self.create_publisher(PointStamped, PERSON_POINT_TOPIC, 10)

        self.pose_detector = PoseDetection()

        self.create_timer(0.1, self.run)

        self.is_tracking_result = False

        self.image = None
        self.depth_image = []
        self.camera_frame = CAMERA_FRAME

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
        except Exception as e:
            print(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def run(self):
        """Main loop to run the tracker"""
        if self.image is None:
            return

        frame = self.image
        results = self.pose_detector.detect(frame)

        if not results.pose_landmarks:
            return

        landmarks = results.pose_landmarks.landmark

        left_shoulder = landmarks[11]
        right_shoulder = landmarks[12]

        h, w, _ = frame.shape

        # Coordinates in pixels
        left = (int(left_shoulder.x * w), int(left_shoulder.y * h))
        right = (int(right_shoulder.x * w), int(right_shoulder.y * h))

        # Midpoint between shoulders
        mid_neck = (int((left[0] + right[0]) / 2), int((left[1] + right[1]) / 2))

        # Get the depth at the midpoint and deproject to 3D point
        depth = get_depth(self.depth_image, mid_neck)
        point3D = deproject_pixel_to_point(self.imageInfo, mid_neck, depth)

        coords = PointStamped(header=Header(frame_id=self.camera_frame),
                point=Point(),)
        coords.x = float(point3D[0])
        coords.y = float(point3D[1])
        coords.z = float(point3D[2])

        self.get_logger().info(
            f"Person detected at coordinates: x={coords.x}, y={coords.y}, z={coords.z}"
        )

        self.person_inside.publish(coords)

def main(args=None):
    rclpy.init(args=args)
    node = IsPersonInside()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

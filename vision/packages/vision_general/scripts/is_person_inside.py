#!/usr/bin/env python3

"""
Node to send the 3D coordinates of a person detected in the camera feed.
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

from frida_interfaces.srv import PersonInsideReq

from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    PERSON_POINT_TOPIC,
    CAMERA_FRAME,
    PERSON_INSIDE_REQUEST_TOPIC,
)

from object_detector_2d.scripts.include.vision_3d_utils import estimate_3d_from_pose

class PersonInsideClient:
    def __init__(self, node: Node, service_name=PERSON_INSIDE_REQUEST_TOPIC):
        self.node = node
        self.client = node.create_client(PersonInside, service_name)

        self.node.get_logger().info(f'Initializing PersonInsideClient with service name: {service_name}')
        print(f'Initializing PersonInsideClient with service name: {service_name}')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info(f'Waiting for {service_name}...')
            print(f'Waiting for {service_name}...')

        self.node.get_logger().info(f'Service {service_name} is now available.')
        print(f'Service {service_name} is now available.')

    def call(self, ymin, xmin, ymax, xmax, callback=None):
        self.node.get_logger().info(f'Preparing request with coordinates: ymin={ymin}, xmin={xmin}, ymax={ymax}, xmax={xmax}')
        print(f'Preparing request with coordinates: ymin={ymin}, xmin={xmin}, ymax={ymax}, xmax={xmax}')

        request = PersonInsideReq.Request()
        request.ymin = float(ymin)
        request.xmin = float(xmin)
        request.ymax = float(ymax)
        request.xmax = float(xmax)

        future = self.client.call_async(request)
        self.node.get_logger().info('Request sent asynchronously.')
        print('Request sent asynchronously.')

        if callback:
            self.node.get_logger().info('Adding custom callback to future.')
            print('Adding custom callback to future.')
            future.add_done_callback(lambda fut: callback(fut.result()))
        else:
            self.node.get_logger().info('Adding default callback to future.')
            print('Adding default callback to future.')
            future.add_done_callback(self._default_callback)

    def _default_callback(self, response):
        if response.success:
            self.node.get_logger().info('Person detected and coordinates published.')
            print('Person detected and coordinates published.')
        else:
            self.node.get_logger().warn('No person detected.')
            print('No person detected.')


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

        self.request_person_inside = self.create_service(
            PersonInsideReq, PERSON_INSIDE_REQUEST_TOPIC, self.handle_person_inside_request
        )

        self.person_inside = self.create_publisher(PointStamped, PERSON_POINT_TOPIC, 10)

        self.pose_detector = PoseDetection()

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

    def handle_person_inside_request(self, request, response):
        """Handle the request to check if a person is inside"""
        if self.image is None or self.depth_image is None:
            response.success = False
            return response
        
        # Get the coordinates of the person in the image
        y1 = request.ymin
        x1 = request.xmin
        y2 = request.ymax
        x2 = request.xmax

        cropped_frame = self.image[y1:y2, x1:x2]

        # Run the main loop to detect person and get coordinates
        self.run(cropped_frame)

        response.success = True
        response.message = "Person coordinates published successfully."
        return response

    def run(self, frame=None):
        """Main loop to run the tracker"""
        results = self.pose_detector.detect(frame)

        if not results.pose_landmarks or len(results.pose_landmarks.landmark) <= 12:
            return

        point3D = estimate_3d_from_pose(
            frame, results.pose_landmarks.landmark, self.imageInfo, self.depth_image
        )
        if point3D is None:
            return

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

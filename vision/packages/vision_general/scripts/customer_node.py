#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2
import time
from ultralytics import YOLO
import tqdm
from vision_general.utils.calculations import (
    get2DCentroid,
    get_depth,
    deproject_pixel_to_point,
)

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
import copy

from frida_interfaces.srv import CropQuery, Customer, IsSitting
from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOIC,
    CROP_QUERY_TOPIC,
    IS_SITTING_TOPIC,
    CUSTOMER,
    GET_CUSTOMER_TOPIC,
)

CONF_THRESHOLD = 0.8
DEPTH_THRESHOLD = 100


class CustomerNode(Node):
    def __init__(self):
        super().__init__("customer_node")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.get_customer_service = self.create_service(
            Customer, GET_CUSTOMER_TOPIC, self.get_customer_callback
        )

        self.is_tracking_result = False

        self.results_publisher = self.create_publisher(PointStamped, RESULTS_TOPIC, 10)

        self.image_publisher = self.create_publisher(Image, TRACKER_IMAGE_TOPIC, 10)

        self.customer_publisher = self.create_publisher(Image, CUSTOMER, 10)

        self.centroid_publisher = self.create_publisher(Point, CENTROID_TOIC, 10)

        self.moondream_client = self.create_client(
            CropQuery, CROP_QUERY_TOPIC, callback_group=self.callback_group
        )
        self.moondream_sitting_client = self.create_client(
            IsSitting, IS_SITTING_TOPIC, callback_group=self.callback_group
        )

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.create_timer(0.1, self.run)
        self.create_timer(0.01, self.publish_image)

    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        self.image = None
        self.image_time = None
        self.frame_id = "zed_left_camera_optical_frame"

        self.depth_image_time = None
        pbar = tqdm.tqdm(total=1, desc="Loading models")

        self.model = YOLO("yolov8n.pt")
        self.pose_detection = PoseDetection(self.is_sitting_moondream)

        self.output_image = []
        self.depth_image = []
        self.customer_image = []

        pbar.close()
        self.get_logger().info("Single Tracker Ready")

    def image_callback(self, data):
        """Callback to receive image from camera"""
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image_time = data.header.stamp

    def depth_callback(self, data):
        """Callback to receive depth image from camera"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_image = depth_image
            self.depth_image_time = data.header.stamp
        except Exception as e:
            print(f"Error: {e}")

    def image_info_callback(self, data):
        """Callback to receive camera info"""
        self.imageInfo = data

    def publish_image(self):
        """Publish the image to the camera topic"""
        if len(self.output_image) != 0:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(self.output_image, "bgr8")
            )

        if len(self.customer_image) != 0:
            h, w = self.customer_image.shape[:2]

            square_size = max(h, w)

            square_img = cv2.copyMakeBorder(
                self.customer_image,
                top=(square_size - h) // 2,
                bottom=(square_size - h + 1) // 2,  # Handles odd differences
                left=(square_size - w) // 2,
                right=(square_size - w + 1) // 2,
                borderType=cv2.BORDER_CONSTANT,
                value=(0, 0, 0),  # Black padding (BGR)
            )
            self.customer_publisher.publish(
                self.bridge.cv2_to_imgmsg(square_img, "bgr8")
            )

    def success(self, message) -> None:
        """Print success message"""
        self.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    def get_customer_callback(self, req, res):
        """Set the target to track (Default: Largest person in frame)"""
        res.found = False
        res.point = PointStamped()
        print("running")
        if self.image is None:
            self.get_logger().warn("No image available")

            return res

        self.frame = copy.deepcopy(self.image)

        self.output_image = self.frame.copy()
        results = copy.deepcopy(self.results)

        for out in results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                # Get confidence
                prob = round(box.conf[0].item(), 2)

                if prob < CONF_THRESHOLD:
                    continue

                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                bbox = box.xyxy[0].tolist()

                cropped_image = self.frame[y1:y2, x1:x2]
                self.customer_image = copy.deepcopy(cropped_image)
                raising = self.pose_detection.is_waving_customer(cropped_image)

                if raising:
                    print("IS RAISING HANDDDDDDDDDDDDDDDDDD")
                    cv2.rectangle(
                        self.output_image,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2,
                    )
                    if len(self.depth_image) > 0 and (
                        (
                            self.depth_image_time.nanosec - self.image_time.nanosec
                            > -DEPTH_THRESHOLD
                        )
                        and (
                            self.depth_image_time.nanosec - self.image_time.nanosec
                            < DEPTH_THRESHOLD
                        )
                    ):
                        coords = PointStamped()
                        coords.header.frame_id = self.frame_id
                        coords.header.stamp = self.depth_image_time
                        point2D = get2DCentroid(bbox, self.depth_image)
                        point2D_x_coord = float(point2D[1])
                        point2D_x_coord_normalized = (
                            point2D_x_coord / (self.frame.shape[1] / 2)
                        ) - 1
                        point2Dpoint = Point()
                        point2Dpoint.x = float(point2D_x_coord_normalized)
                        point2Dpoint.y = 0.0
                        point2Dpoint.z = 0.0
                        # self.get_logger().info(f"frame_shape: {self.frame.shape[1]} Point2D: {point2D[1]} normalized_point2D: {point2D_x_coord_normalized}")
                        self.centroid_publisher.publish(point2Dpoint)
                        depth = get_depth(self.depth_image, point2D)
                        point_2d_temp = (point2D[1], point2D[0])
                        point3D = deproject_pixel_to_point(
                            self.imageInfo, point_2d_temp, depth
                        )
                        # print(point3D)
                        point3D = (
                            float(point3D[0]),
                            float(point3D[1]),
                            float(point3D[2]),
                        )
                        coords.point.x = point3D[0]
                        coords.point.y = point3D[1]
                        coords.point.z = point3D[2]
                        # self.point_pub.publish(coords)
                        self.results_publisher.publish(coords)
                        res.point = coords
                        res.found = True
                        self.success("Customer found")
                        return res
        self.get_logger().warn("No customer raising hand")
        return res

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            #print("Waiting for future to complete...")
            print(".", end="", flush=True)
            time.sleep(0.5)
        return future

    def is_sitting_moondream(self, image):
        if image is None:
            return False

        if not self.moondream_sitting_client.wait_for_service(timeout_sec=0.1):
            print("Moondream sitting service not available")
            return False

        request = IsSitting.Request()
        request.image = self.bridge.cv2_to_imgmsg(image, "bgr8")

        future = self.moondream_sitting_client.call_async(request)
        future = self.wait_for_future(future, timeout=2)
        if not future:
            return False

        result = future.result()
        if result is None:
            return False

        if not result.success:
            return False

        return bool(result.answer)

    def run(self):
        """Main loop to run the tracker"""

        if True:  # self.target_set:
            self.frame = self.image
            # image_time = copy.deepcopy(self.image_time)
            # depth_image_time = copy.deepcopy(self.depth_image_time)
            if self.frame is None:
                self.get_logger().error("No image available")
                return

            if self.frame is None:
                return

            self.output_image = self.frame.copy()

            self.results = self.model.track(
                self.frame,
                persist=True,
                tracker="bytetrack.yaml",
                classes=0,
                verbose=False,
            )

            # Check each detection
            for out in self.results:
                for box in out.boxes:
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]

                    cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    # Get confidence
                    prob = round(box.conf[0].item(), 2)

                    if prob < CONF_THRESHOLD:
                        continue


def main(args=None):
    rclpy.init(args=args)
    node = CustomerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

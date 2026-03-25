#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

import cv2
from vision_general.utils.trt_utils import load_yolo_trt
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

from frida_interfaces.srv import CropQuery, Customer
from frida_interfaces.msg import PersonList, Person
from pose_detection import PoseDetection
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    TRACKER_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    RESULTS_TOPIC,
    CAMERA_INFO_TOPIC,
    CENTROID_TOIC,
    CROP_QUERY_TOPIC,
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

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
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

        self.verbose = self.declare_parameter("verbose", True)
        self.setup()
        self.create_timer(0.1, self.run)
        self.create_timer(0.1, self.publish_image)

    def setup(self):
        """Load models and initial variables"""
        self.target_set = False
        self.image = None
        self.image_time = None
        self.frame_id = "zed_left_camera_optical_frame"

        self.depth_image_time = None
        pbar = tqdm.tqdm(total=1, desc="Loading models")

        # Load YOLO with TensorRT for Orin AGX
        self.model = load_yolo_trt("yolov8n.pt")
        self.pose_detection = PoseDetection()

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
        res.people = PersonList()
        res.people.list = []
        print("running")
        if self.image is None:
            self.get_logger().warn("No image available")

            return res

        self.frame = self.image.copy()
        self.output_image = self.frame.copy()

        for out in self.results:
            for box in out.boxes:
                if box.conf[0].item() < CONF_THRESHOLD:
                    continue

                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                cropped_image = self.frame[y1:y2, x1:x2]
                self.customer_image = cropped_image.copy()

                raising = self.pose_detection.is_waving_customer(cropped_image)
                sitting = self.pose_detection.is_sitting_yolo(cropped_image)

                if not sitting:
                    self.get_logger().info("Checking sitting with moondream")
                    sitting = self.is_sitting_moondream([x1, y1, x2, y2])

                if raising and sitting:
                    self.get_logger().info("Customer raising hand and sitting detected")
                    cv2.rectangle(self.output_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # 2D Centroid and Depth
                    point2D = get2DCentroid((x1, y1, x2, y2), self.depth_image)
                    point_2d_temp = (point2D[1], point2D[0])
                    depth = get_depth(self.depth_image, point2D)
                    point3D_raw = deproject_pixel_to_point(
                        self.imageInfo, point_2d_temp, depth
                    )

                    # Transform to ROS frame
                    coords = PointStamped()
                    coords.header.frame_id = self.frame_id
                    coords.header.stamp = self.depth_image_time
                    coords.point.x = float(point3D_raw[0])
                    coords.point.y = float(point3D_raw[1])
                    coords.point.z = float(point3D_raw[2])
                    self.results_publisher.publish(coords)

                    # Normalize X coordinate for basic tracking
                    pt_x_norm = float(point2D[1]) / (self.frame.shape[1] / 2) - 1.0
                    self.centroid_publisher.publish(Point(x=pt_x_norm, y=0.0, z=0.0))

                    # Append to results
                    person = Person()
                    person.x = (x1 + x2) // 2
                    person.y = (y1 + y2) // 2
                    person.point3d = coords
                    res.people.list.append(person)

                    res.found = True
                    self.success("Customer found")
                    self.get_logger().info(
                        f"Customer position (3D): x={coords.point.x:.3f}  y={coords.point.y:.3f}  z={coords.point.z:.3f}"
                    )

        self.get_logger().info(f"Customers detected: {len(res.people.list)}")
        return res

    def is_sitting_moondream(self, bbox):
        if self.frame is None or bbox is None:
            return False

        if not self.moondream_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("Moondream crop query service not available")
            return False

        height, width = self.frame.shape[:2]
        x1, y1, x2, y2 = bbox
        if width <= 0 or height <= 0:
            return False

        xmin = max(0.0, min(1.0, float(x1) / float(width)))
        ymin = max(0.0, min(1.0, float(y1) / float(height)))
        xmax = max(0.0, min(1.0, float(x2) / float(width)))
        ymax = max(0.0, min(1.0, float(y2) / float(height)))

        request = CropQuery.Request()
        request.xmin = xmin
        request.ymin = ymin
        request.xmax = xmax
        request.ymax = ymax
        request.query = (
            "Is the person in this image sitting? Answer only with yes or no."
        )

        future = self.moondream_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=7)

        if not future.done():
            self.get_logger().warn("Service call timed out")
            return False

        if future.exception() is not None:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return False

        result = future.result()
        if result is None or not result.success:
            return False

        answer = result.result.strip().lower()
        if answer.startswith("yes"):
            return True
        if answer.startswith("no"):
            return False

        self.get_logger().warn(
            f"Unexpected answer from Moondream crop query: '{answer}'. Expected 'yes' or 'no'."
        )
        return False

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

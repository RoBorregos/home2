#!/usr/bin/env python3

"""
Node to track a single person and
re-id them if necessary
"""

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
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_point
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)

TEST_TOPIC = "/vision/detections_image"


class SingleTracker(Node):
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

        self.point_pub = self.create_publisher(PointStamped, "point_visualize", 10)

        self.tf_buffer = Buffer()
        self.setup()
        self.create_timer(0.1, self.run)
        self.create_timer(0.1, self.publish_image)

    def setup(self):
        self.image = None
        self.depth_image = None

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

    def run(self):
        if self.image is None:
            self.get_logger().warn("No image available")
            return

        self.frame = self.image
        if len(self.depth_image) > 0:
            coords = Point()
            point2D = get2DCentroid(self.object_coords, self.frame)
            depth = get_depth(self.depth_image, point2D)
            point3D = deproject_pixel_to_point(self.imageInfo, point2D, depth)
            point3D = float(point3D[0]), float(point3D[1]), float(point3D[2])
            coords.x = point3D[0]
            coords.y = point3D[1]
            coords.z = point3D[2]
            stamped_point = PointStamped()
            stamped_point.header.frame_id = (
                "zed_camera_link"  # Adjust to match your actual camera frame
            )
            stamped_point.header.stamp = self.get_clock().now().to_msg()
            stamped_point.point.x = coords.x
            stamped_point.point.y = coords.y
            stamped_point.point.z = coords.z
            transform = self.tf_buffer.lookup_transform(
                "base_link", stamped_point.header.frame_id, rclpy.time.Time()
            )
            transformed_point = do_transform_point(stamped_point, transform)
            self.point_pub.publish(transformed_point)

        else:
            self.get_logger().warn("Depth image not available")


"""
    self.person_data["coordinates"] = (
                                    person["x1"],
                                    person["y1"],
                                    person["x2"],
                                    person["y2"],
                                )
"""


def main(args=None):
    rclpy.init(args=args)
    node = SingleTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

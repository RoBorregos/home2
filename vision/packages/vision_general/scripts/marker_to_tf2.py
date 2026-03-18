#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray
from frida_constants.vision_constants import DETECTIONS_POSES_TOPIC, TRASH_DEPTH_TOPIC
import tf2_ros
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_point

DEFAULT_CLASSES = ["black_trashcan", "trashcan"]


class TrashDetectionNodeDemo(Node):
    """
    Class for trash detection
    **classes to detect are customizable via zero shot /vision/set_detector_classes service
    zero sot requires the camera, this node only requires the poses**
    """

    def __init__(self):
        super().__init__("trash_detection_demo")

        self.tf_buffer = Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.class_names = DEFAULT_CLASSES

        self.latest_poses = PoseArray()
        self.image = None

        self.detections_sub = self.create_subscription(
            PoseArray,
            DETECTIONS_POSES_TOPIC,
            self.detections_callback,
            10,
        )

        self.trash_depth_publisher = self.create_publisher(
            PointStamped, TRASH_DEPTH_TOPIC, 10
        )

        self.get_logger().info("Trash Detection Node initialized.")

    def detections_callback(self, data):
        """
        Store latest yolo-e pose detections and publish 3D coordinates
        """
        self.latest_poses = data
        self.get_logger().info(f"Received {len(data.detections)} detections")

        if self.latest_poses is None or len(self.latest_poses) == 0:
            self.get_logger().warn("No pose messages")

        for pose in self.latest_poses:
            try:
                point3D = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
                stamped_point = PointStamped()
                stamped_point.header.frame_id = "zed_camera_link"
                stamped_point.header.stamp = self.get_clock().now().to_msg()
                stamped_point.point.x = point3D[0]
                stamped_point.point.y = point3D[1]
                stamped_point.point.z = point3D[2]

                transform = self.tf_buffer.lookup_transform(
                    "base_link", stamped_point.header.frame_id, rclpy.time.Time()
                )
                transformed_point = do_transform_point(stamped_point, transform)
                self.trash_depth_publisher.publish(transformed_point)

                self.get_logger().info(
                    f"Detection at: x={stamped_point.point.x:.2f}, "
                    f"y={stamped_point.point.y:.2f}, z={stamped_point.point.z:.2f}"
                )

            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNodeDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

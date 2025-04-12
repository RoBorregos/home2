#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for visualization marker
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Example point in the ZED camera frame
        self.original_point = PointStamped()
        self.original_point.header.frame_id = 'zed2_left_camera_frame'  # Replace with your ZED frame
        self.original_point.point.x = -0.35303948620751946
        self.original_point.point.y = 0.8954556367524048
        self.original_point.point.z = 1.0139923095703125

        # Timer to periodically transform and visualize the point
        self.timer = self.create_timer(1.0, self.transform_and_publish)

    def transform_and_publish(self):
        self.original_point.header.stamp = self.get_clock().now().to_msg()

        try:
            # Lookup transform from ZED frame to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.original_point.header.frame_id,
                rclpy.time.Time()
            )
            transformed_point = do_transform_point(self.original_point, transform)
            self.get_logger().info(f"Transformed point: {transformed_point.point}")

            # Publish the marker at the transformed position
            self.publish_marker(transformed_point.point)

        except Exception as e:
            self.get_logger().warn(f'Could not transform point: {e}')

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'zed_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

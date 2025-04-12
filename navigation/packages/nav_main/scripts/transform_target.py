#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
import math as m
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.point_pub = self.create_publisher(PointStamped, 'point_visualize', 10)
        self.accumulated_points = []
        # Subscribe to the vision/tracking_results topic
        self.point_sub = self.create_subscription(
            Point,
            '/vision/tracking_results',
            self.point_callback,
            10
        )

    def median_filter(self, data):
        '''From a given list of data, return the median value'''
        # Sort the data
        sorted_data = sorted(data)
        # Calculate the median
        n = len(sorted_data)
        mid = n // 2
        if n % 2 == 0:
            median = (sorted_data[mid - 1] + sorted_data[mid]) / 2
        else:
            median = sorted_data[mid]
        return median
    

    def point_callback(self, msg: Point):
        # Wrap the raw Point into a PointStamped
        
        point_to_be_filtered = Point()
    
        original_point_x = msg.x
        original_point_y = msg.y

        original_orientation = m.atan2(original_point_y, original_point_x)
        original_distance = m.sqrt(original_point_x ** 2 + original_point_y ** 2)

        print(f"Original Point: x={original_point_x}, y={original_point_y}")
        print(f"Original Orientation: {original_orientation}")
        print(f"Original Distance: {original_distance}")

        # Transform the point

        transformed_orientation = original_orientation + m.pi*(1.25)
        print(f"Transformed Orientation: {transformed_orientation}")
        transformed_x = original_distance * m.cos(transformed_orientation)
        transformed_y = original_distance * m.sin(transformed_orientation)
        print(f"Transformed Point: x={transformed_x}, y={transformed_y}")
        
        point_to_be_filtered.x = transformed_x
        point_to_be_filtered.y = transformed_y
        point_to_be_filtered.z = 0.0

        # Append the new point to the list
        self.accumulated_points.append(point_to_be_filtered)
        # Limit the number of points to the last 5
        if len(self.accumulated_points) > 5:
            self.accumulated_points.pop(0)
        # Calculate the median of the accumulated points
        median_x = self.median_filter([p.x for p in self.accumulated_points])
        median_y = self.median_filter([p.y for p in self.accumulated_points])
        print(f"Median Point: x={median_x}, y={median_y}")
        # Publish the median point

        stamped_point = PointStamped()
        stamped_point.header.frame_id = 'base_link'  # Adjust to match your actual camera frame
        stamped_point.header.stamp = self.get_clock().now().to_msg()
        stamped_point.point.x = median_x
        stamped_point.point.y = median_y
        stamped_point.point.z = 0.0
        self.point_pub.publish(stamped_point)


        # try:
        #     # Transform the point to base_link
        #     transform = self.tf_buffer.lookup_transform(
        #         'base_link',
        #         stamped_point.header.frame_id,
        #         rclpy.time.Time()
        #     )
        #     transformed_point = do_transform_point(stamped_point, transform)
        #     self.get_logger().info(f'Transformed point: {transformed_point.point}')
        #     self.publish_marker(transformed_point.point)

        # except Exception as e:
        #     self.get_logger().warn(f'Failed to transform point: {e}')

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
        marker.pose.position.z = 0.0  # Force it to lie flat on the RViz grid
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
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

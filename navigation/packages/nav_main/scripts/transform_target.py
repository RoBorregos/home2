#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped,PoseStamped
from visualization_msgs.msg import Marker
import math as m
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from std_srvs.srv import SetBool


ACTIVATE_FOLLOWING_TOPIC = "/navigation/activate_following"


class PointTransformer(Node):
    def __init__(self):
        super().__init__('goal_transformer')

        # Initialize flags
        self.is_following = False
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.point_pub = self.create_publisher(PointStamped, 'point_visualize', 10)
        self.gual_pub = self.create_publisher(PoseStamped, 'goal_update', 10)
        self.set_target_service = self.create_service(
            SetBool, ACTIVATE_FOLLOWING_TOPIC, self.set_target_callback
        )

        self.accumulated_points = []
        # Subscribe to the vision/tracking_results topic
        self.point_sub = self.create_subscription(
            Point,
            '/navigation/activate_following',
            self.point_callback,
            10
        )
        #send initial goal 
        # self.initial_goal = PoseStamped()
        # self.initial_goal.header.frame_id = 'map'
        # self.initial_goal.header.stamp = self.get_clock().now().to_msg()
        # self.initial_goal.pose.position.x = 0.0
        # self.initial_goal.pose.position.y = 0.0
        # self.initial_goal.pose.position.z = 0.0
        # self.initial_goal.pose.orientation.w = 1.0
        # self.initial_goal.pose.orientation.x = 0.0
        # self.initial_goal.pose.orientation.y = 0.0
        # self.initial_goal.pose.orientation.z = 0.0
        # self.gual_pub.publish(self.initial_goal)
        # self.get_logger().info('Point Transformer Node Initialized')

    def set_target_callback(self, request, response):
        """Callback to set the target to track"""
        self.is_following = request.data
        if self.is_following:
            self.get_logger().info("following enabled")
        else:
            self.get_logger().info("following disabled")
        return response
    
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
        if not self.is_following:
            return
        
        point_to_be_filtered = Point()
    
        original_point_x = msg.x
        original_point_y = msg.y

        original_orientation = m.atan2(original_point_y, original_point_x)
        original_distance = m.sqrt(original_point_x ** 2 + original_point_y ** 2)

        # Transform the point

        transformed_orientation = original_orientation + m.pi*(1.25)

        transformed_x = original_distance * m.cos(transformed_orientation)
        transformed_y = original_distance * m.sin(transformed_orientation)
        
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

        stamped_point = PointStamped()
        stamped_point.header.frame_id = 'base_link'  # Adjust to match your actual camera frame
        stamped_point.header.stamp = self.get_clock().now().to_msg()
        stamped_point.point.x = median_x
        stamped_point.point.y = median_y
        stamped_point.point.z = 0.0
        

        try:
            # Transform the point to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                stamped_point.header.frame_id,
                rclpy.time.Time()
            )
            transformed_point = do_transform_point(stamped_point, transform)
            # Publish on goal update

            self.point_pub.publish(transformed_point)
            
            #publish on goal update
            goal_update = PoseStamped()
            goal_update.header.frame_id = 'map'
            goal_update.header.stamp = self.get_clock().now().to_msg()
            goal_update.pose.position.x = transformed_point.point.x
            goal_update.pose.position.y = transformed_point.point.y
            goal_update.pose.position.z = 0.0
            goal_update.pose.orientation.w = 1.0
            goal_update.pose.orientation.x = 0.0
            goal_update.pose.orientation.y = 0.0
            goal_update.pose.orientation.z = 0.0
            self.gual_pub.publish(goal_update)

        except Exception as e:
            self.get_logger().warn(f'Failed to transform point: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

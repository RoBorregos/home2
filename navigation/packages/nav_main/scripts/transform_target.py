#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from visualization_msgs.msg import Marker
import math as m
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from std_srvs.srv import SetBool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from frida_constants.navigation_constants import (
    FOLLOWING_SERVICE,
)

class PointTransformer(Node):
    def __init__(self):
        super().__init__('goal_transformer')
        ##TEST##
        self.goal_test = PoseStamped()
        self.goal_test.header.frame_id = 'map'
        self.goal_test.header.stamp = self.get_clock().now().to_msg()
        self.goal_test.pose.position.x = 0.0
        self.goal_test.pose.position.y = 0.0
        self.goal_test.pose.position.z = 0.0
        self.goal_test.pose.orientation.w = 1.0
        self.goal_test.pose.orientation.x = 0.0
        self.goal_test.pose.orientation.y = 0.0
        self.goal_test.pose.orientation.z = 0.0
        # Initialize flags
        self.actual_goal_handle = None
        self.is_following = False
        self.navigation_in_progress = False
        self.accumulated_points = []
        self.acc_sum = [0.0, 0.0]
        
        # Create a callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker publisher
        self.point_pub = self.create_publisher(PointStamped, 'point_visualize', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_update', 10)
        
        # Navigation action client
        self.nav_action_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose', 
            callback_group=self.callback_group
        )
        
        self.set_target_service = self.create_service(
            SetBool, FOLLOWING_SERVICE, self.set_target_callback,
            callback_group=self.callback_group
        )

        # Subscribe to the vision/tracking_results topic
        self.point_sub = self.create_subscription(
            Point,
            '/vision/tracking_results',
            self.point_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("PointTransformer with Navigation Action Client initialized")

    def set_target_callback(self, request, response):
        """Callback to set the target to track"""
        self.is_following = request.data
        
        if request.data:
            self.get_logger().info("Following enabled")
            ###### TEST ##########
            # goal_update = PoseStamped()
            # goal_update.header.frame_id = 'map'
            # goal_update.header.stamp = self.get_clock().now().to_msg()
            # goal_update.pose.position.x = 0.0
            # goal_update.pose.position.y = 0.0
            # goal_update.pose.position.z = 0.0
            # goal_update.pose.orientation.w = 1.0
            # goal_update.pose.orientation.x = 0.0
            # goal_update.pose.orientation.y = 0.0
            # goal_update.pose.orientation.z = 0.0
            # self.get_logger().info("sending test_goal")
            # self.send_navigation_goal(goal_update)

        else:
            self.cancel_navigation()
            self.get_logger().info("Following disabled")

        response.success = True
        response.message = "Following state changed"
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
    
    def send_navigation_goal(self, pose_stamped):
        """Send a navigation goal to the action server"""
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        self.get_logger().info(f'Sending navigation goal: x={pose_stamped.pose.position.x:.2f}, y={pose_stamped.pose.position.y:.2f}')
        
        self.navigation_in_progress = True
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback for when the action server responds to our goal request"""
        goal_handle = future.result()
        self.get_logger().warn(f"function_papu? = {goal_handle}")
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.navigation_in_progress = False
            return
        self.navigation_in_progress = True
        self.get_logger().info('Navigation goal accepted')
        self.actual_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback for when the action server returns a result"""
        self.navigation_in_progress = False
        self.actual_goal_handle = None
    
    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback"""
        feedback = feedback_msg.feedback
        # You can use this to monitor progress if needed
        pass
    
    def cancel_navigation(self):
        """Cancel the current navigation goal"""
        if self.navigation_in_progress and self.actual_goal_handle is not None:
            self.get_logger().info('Canceling navigation goal')
            self.actual_goal_handle.cancel_goal_async()
            self.navigation_in_progress = False

    def point_callback(self, msg: Point):
        # Skip processing if not following
        if not self.is_following:
            return
        

        # Process the point as before
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
        self.acc_sum[0] += point_to_be_filtered.x
        self.acc_sum[1] += point_to_be_filtered.y
        # Limit the number of points to the last 5
        if len(self.accumulated_points) > 5:
            self.acc_sum[0] -= self.accumulated_points[0].x
            self.acc_sum[1] -= self.accumulated_points[0].y
            self.accumulated_points.pop(0)
        # Calculate the median of the accumulated points
        # median_x = self.median_filter([p.x for p in self.accumulated_points])
        # median_y = self.median_filter([p.y for p in self.accumulated_points])
        median_x = self.acc_sum[0] / len(self.accumulated_points)
        median_y = self.acc_sum[1] / len(self.accumulated_points)


        stamped_point = PointStamped()
        stamped_point.header.frame_id = 'zed_camera_link'  # Adjust to match your actual camera frame
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
            # Publish for visualization
            self.point_pub.publish(transformed_point)
            
            # Create goal pose message
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
            if self.navigation_in_progress == False:
                self.send_navigation_goal(goal_update)
            # Publish for debugging/visualization
            self.goal_pub.publish(goal_update)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to transform point: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    
    # Use MultiThreadedExecutor to handle concurrent callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
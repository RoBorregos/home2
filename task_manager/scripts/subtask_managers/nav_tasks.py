#!/usr/bin/env python3

"""
Node to move to a place.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from utils.decorators import mockable, service_check
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, PointStamped,PoseStamped
import math as m
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import os
import json
from rclpy.task import Future

MOVE_TOPIC = "/navigate_to_pose"

TIMEOUT = 5.0


class NavigationTasks:
    """Class to manage the navigation tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    def __init__(self, task_manager, mock_data=False) -> None:
        self.node = task_manager
        self.mock_data = mock_data
        self.goal_state = None
        if not self.mock_data:
            self.setup_services()

    def setup_services(self):
        self.pose_client = ActionClient(self.node, NavigateToPose, MOVE_TOPIC)
       
        if not self.pose_client.wait_for_server(timeout_sec=TIMEOUT):
            self.node.get_logger().warn("Move service not initialized.")

    @mockable(return_value=True, delay=2)
    @service_check("pose_client", False, TIMEOUT)
    def move_to_location(self, location: str, sublocation: str) -> Future:
        """Attempts to move to the given location and returns a Future that completes when the action finishes.
        Call the function on this way

        future = self.subtask_manager["navigation"].move_to_location("living_room", "couches")
        # Wait for the action result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        """
        future = Future()
        try:
            package_share_directory = get_package_share_directory("frida_constants")
            file_path = os.path.join(package_share_directory, "map_areas/areas.json")
            with open(file_path, "r") as file:
                data = json.load(file)
            if sublocation != "":
                coordinates = data[location][sublocation]
            else:
                coordinates = data[location]["safe_place"]
                sublocation = "safe_place"
            self.node.get_logger().info(f"{coordinates}")
        except Exception as e:
            self.node.get_logger().error(f"Error fetching coordinates: {e}")
            future.set_result(self.STATE["EXECUTION_ERROR"])
            return future

        try:
            self.node.get_logger().info(f"Sending move request to: {location} {sublocation}")
            client_goal = NavigateToPose.Goal()
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = coordinates[0]
            goal.pose.position.y = coordinates[1]
            goal.pose.position.z = coordinates[2]
            goal.pose.orientation.x = coordinates[3]
            goal.pose.orientation.y = coordinates[4]
            goal.pose.orientation.z = coordinates[5]
            goal.pose.orientation.w = coordinates[6]

            client_goal.pose = goal
            self.goal_state = None
            self._send_goal_future = self.pose_client.send_goal_async(client_goal)

            self._send_goal_future.add_done_callback(
                lambda future_goal: self.goal_response_callback(future_goal, future)
            )

            return future
        except Exception as e:
            self.node.get_logger().error(f"Error moving to location: {e}")
            future.set_result(self.STATE["EXECUTION_ERROR"])
            return future

    def goal_response_callback(self, future, result_future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected.")
            result_future.set_result(self.STATE["EXECUTION_ERROR"])
            return

        self.node.get_logger().info("Goal accepted! Waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future_result: self.result_callback(result_future)
        )

    def result_callback(self, result_future):
        self.node.get_logger().info("Goal execution completed!")
        result_future.set_result(self.STATE["EXECUTION_SUCCESS"])

    def follow_person(self, activate: bool, pose: Point):
        """Follow a person using the given pose"""
        if activate:
            self.node.get_logger().info(f"Following person at pose: {pose}")
            # Here you would implement the logic to follow the person
            point_to_be_filtered = Point()
            original_point_x = pose.x
            original_point_y = pose.y
            original_orientation = m.atan2(original_point_y, original_point_x)
            original_distance = m.sqrt(original_point_x ** 2 + original_point_y ** 2)
            transformed_orientation = original_orientation + m.pi * (1.25)
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
            print(f"Median Point: x={median_x}, y={median_y}")
            # Publish the median point

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
                self.node.get_logger().warn(f'Failed to transform point: {e}')

        else:
            self.node.get_logger().info("Stopping following person.")

if __name__ == "__main__":
    rclpy.init()
    node = Node("navigation_tasks")
    navigation_tasks = NavigationTasks(node)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")

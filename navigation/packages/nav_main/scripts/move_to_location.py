#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import json
from frida_interfaces.action import Move
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import frida_constants
import importlib.resources as pkg_resources

MOVE_TOPIC = "/navigation/move"


class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_action_server')

        self.callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self, Move, MOVE_TOPIC, self.move_callback,
            callback_group=self.callback_group
        )

        self.nav2_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        try:
            with pkg_resources.files(frida_constants).joinpath("areas.json").open("r") as f:
                self.locations = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON file: {e}")
            self.locations = {}

        self.get_logger().info("MoveActionServer ready")

    def get_location_pose(self, location):
        """Get pose array [x, y, z, qx, qy, qz, qw] for a location name.
        Searches area names (returns safe_place) and location names within areas."""
        for area_name, area_locations in self.locations.items():
            if location == area_name:
                if "safe_place" in area_locations:
                    return area_locations["safe_place"]
                first_key = next(iter(area_locations))
                if isinstance(area_locations[first_key], list):
                    return area_locations[first_key]
                return None
            if isinstance(area_locations, dict) and location in area_locations:
                pose = area_locations[location]
                if isinstance(pose, list) and len(pose) >= 7:
                    return pose
        return None

    async def move_callback(self, goal_handle):
        """Navigate the robot to the requested location using Nav2."""
        target_location = goal_handle.request.location
        self.get_logger().info(f"Received move request to: {target_location}")

        pose = self.get_location_pose(target_location)
        if pose is None:
            self.get_logger().warn(f"Invalid location: {target_location}")
            feedback = Move.Feedback()
            feedback.feedback = f"Unknown location: {target_location}"
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result = Move.Result()
            result.success = False
            return result

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            goal_handle.succeed()
            result = Move.Result()
            result.success = False
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(pose[0])
        nav_goal.pose.pose.position.y = float(pose[1])
        nav_goal.pose.pose.position.z = float(pose[2])
        nav_goal.pose.pose.orientation.x = float(pose[3])
        nav_goal.pose.pose.orientation.y = float(pose[4])
        nav_goal.pose.pose.orientation.z = float(pose[5])
        nav_goal.pose.pose.orientation.w = float(pose[6])

        self.get_logger().info(
            f"Navigating to {target_location} at ({pose[0]:.2f}, {pose[1]:.2f})")

        feedback_msg = Move.Feedback()
        feedback_msg.feedback = f"Navigating to {target_location}..."
        goal_handle.publish_feedback(feedback_msg)

        send_goal_future = await self.nav2_client.send_goal_async(nav_goal)

        if not send_goal_future.accepted:
            self.get_logger().warn("Nav2 goal was rejected")
            goal_handle.succeed()
            result = Move.Result()
            result.success = False
            return result

        self.get_logger().info("Nav2 goal accepted, waiting for result...")
        nav_result_future = await send_goal_future.get_result_async()
        nav_status = nav_result_future.status

        result = Move.Result()
        if nav_status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"\033[92mSUCCESS:\033[0m Arrived at {target_location}")
            feedback_msg.feedback = f"Arrived at {target_location}"
            goal_handle.publish_feedback(feedback_msg)
            result.success = True
        else:
            self.get_logger().warn(
                f"Navigation to {target_location} failed with status: {nav_status}")
            feedback_msg.feedback = f"Navigation failed (status: {nav_status})"
            goal_handle.publish_feedback(feedback_msg)
            result.success = False

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(move_action_server)
    executor.spin()


if __name__ == '__main__':
    main()

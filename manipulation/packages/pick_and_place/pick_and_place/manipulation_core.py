#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickMotion, PickTask
from frida_interfaces.srv import ClusterObjectFromPoint
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PICK_ACTION_SERVER,
    CLUSTER_OBJECT_SERVICE,
)
import copy
from geometry_msgs.msg import PoseStamped


class ManipulationCore(Node):
    def __init__(self):
        super().__init__("manipulation_core")
        self.callback_group = ReentrantCallbackGroup()

        self._pick_action_client = ActionServer(
            self,
            PickTask,
            PICK_ACTION_SERVER,
            self.pick_execute_callback,
            callback_group=self.callback_group,
        )

        self._pick_motion_action_client = ActionClient(
            self,
            PickMotion,
            PICK_MOTION_ACTION_SERVER,
        )

        self._cluster_object_client = self.create_client(
            ClusterObjectFromPoint, CLUSTER_OBJECT_SERVICE
        )

        # self._gpd_client = self.create_client(self, GPDCloudRequest, GPD_CLOUD_SERVICE)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute_callback(self, goal_handle):
        self.get_logger().info("Received object pose")
        self.get_logger().info(f"Goal: {goal_handle.request.object_point}")
        self.get_logger().info("Extracting object cloud")

        # Call cloud extractor
        request = ClusterObjectFromPoint.Request()
        request.point = goal_handle.request.object_point
        self._cluster_object_client.wait_for_service()
        future = self._cluster_object_client.call_async(request)
        future = wait_for_future(future)

        self.get_logger().info(f"Object Cloud extracted: {future.result()}")

        # Save to PCD??
        # Call Grasp Pose Detection
        ##### FAKE #####
        object_point = goal_handle.request.object_point
        grasp_pose1 = PoseStamped()
        grasp_pose1.header.frame_id = object_point.header.frame_id
        grasp_pose1.pose.position.x = object_point.point.x
        grasp_pose1.pose.position.y = object_point.point.y
        grasp_pose1.pose.position.z = object_point.point.z + 0.10
        grasp_pose1.pose.orientation.x = 1.0
        grasp_pose1.pose.orientation.y = 0.0
        grasp_pose1.pose.orientation.z = 0.0
        grasp_pose1.pose.orientation.w = 0.0

        grasp_pose2 = copy.deepcopy(grasp_pose1)
        grasp_pose2.pose.position.z = object_point.point.z + 0.25

        grasp_poses = [grasp_pose1, grasp_pose2]

        # Call Pick Motion Action
        # Create goal
        goal_msg = PickMotion.Goal()
        goal_msg.grasping_poses = grasp_poses

        # Send goal
        self.get_logger().info("Sending pick motion goal...")
        future = self._pick_motion_action_client.send_goal_async(goal_msg)
        future = wait_for_future(future)

        # Check result
        result = future.result()
        self.get_logger().info(f"Pick Motion Result: {result}")

        if result:
            # Initialize result
            result = PickTask.Result()
            result.success = True
            goal_handle.succeed()
            return result

        self.get_logger().error("Pick Motion failed")
        goal_handle.abort()
        result = PickTask.Result()
        result.success = False
        return result


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

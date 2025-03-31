#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickMotion, PickTask
from frida_interfaces.srv import PerceptionService, GraspDetection
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PICK_ACTION_SERVER,
    PERCEPTION_SERVICE,
    GRASP_DETECTION_SERVICE,
)
import copy
from geometry_msgs.msg import PoseStamped

CFG_PATH = (
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params.cfg"
)


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

        self.perception_3d_client = self.create_client(
            PerceptionService, PERCEPTION_SERVICE
        )

        self.grasp_detection_service = self.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE
        )

        # self._gpd_client = self.create_client(self, GPDCloudRequest, GPD_CLOUD_SERVICE)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute_callback(self, goal_handle):
        self.get_logger().info("Received object pose")
        self.get_logger().info(f"Goal: {goal_handle.request.object_point}")
        self.get_logger().info("Extracting object cloud")

        # Call cloud extractor
        request = PerceptionService.Request()
        request.point = goal_handle.request.object_point
        request.add_collision_objects = True
        self.perception_3d_client.wait_for_service()
        future = self.perception_3d_client.call_async(request)
        future = wait_for_future(future)

        pcl_result = future.result().cluster_result
        if len(pcl_result.data) == 0:
            self.get_logger().error("No object cluster detected")
            goal_handle.abort()
            result = PickTask.Result()
            result.success = False
            return result
        self.get_logger().info(
            f"Object cluster detected: {len(pcl_result.data)} points"
        )
        # Save to PCD??
        # Call Grasp Pose Detection
        grasp_poses, grasp_scores = self.get_grasps(pcl_result)
        ##### FAKE #####
        # grasp_poses = self.fake_grasps(goal_handle.request.object_point)

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

    def get_grasps(self, object_cloud):
        request = GraspDetection.Request()
        request.input_cloud = object_cloud
        request.cfg_path = CFG_PATH
        self.get_logger().info("Sending Grasp Detection Request")
        self.grasp_detection_service.wait_for_service()
        future = self.grasp_detection_service.call_async(request)
        future = wait_for_future(future)

        response = future.result()
        if len(response.grasp_poses) == 0:
            self.get_logger().error("No grasps found")
            return []
        for i, (pose, score) in enumerate(
            zip(response.grasp_poses, response.grasp_scores)
        ):
            self.get_logger().info(f"\nGrip {i + 1} - Score: {score:.3f}")
        return response.grasp_poses, response.grasp_scores

    def fake_grasps(object_point):
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

        return grasp_poses


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickMotion, PickTask
from frida_interfaces.srv import ClusterObjectFromPoint, GPDCloudRequest
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PICK_ACTION_SERVER,
    CLUSTER_OBJECT_SERVICE,
    GPD_CLOUD_SERVICE,
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

        self._cluster_object_client = self.create_client(
            self, ClusterObjectFromPoint, CLUSTER_OBJECT_SERVICE
        )

        self._gpd_client = self.create_client(self, GPDCloudRequest, GPD_CLOUD_SERVICE)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute_callback(self, goal_handle):
        self.get_logger().info("Received object pose")
        self.get_logger().info(f"Goal: {goal_handle.request.object_pose}")
        self.get_logger().info("Extracting object cloud")

        # Call cloud extractor
        request = ClusterObjectFromPoint.Request()
        request.point = goal_handle.request.object_point
        self._cluster_object_client.call_async(request)

        # Initialize result
        result = PickTask.Result()
        result.success = True
        goal_handle.succeed(result)
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

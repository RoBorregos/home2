#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickMotion, PickTask
from frida_interfaces.srv import PerceptionService, GraspDetection
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PICK_ACTION_SERVER,
    PERCEPTION_SERVICE,
    GRASP_DETECTION_SERVICE,
)
from pick_and_place.managers.PickManager import PickManager


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

        self.grasp_detection_client = self.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE
        )

        self.pick_manager = PickManager(self)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute_callback(self, goal_handle):
        self.get_logger().info("Received object pose")
        self.get_logger().info(f"Goal: {goal_handle.request.object_point}")
        self.get_logger().info("Extracting object cloud")

        # Call cloud extractor

        result = self.pick_manager.execute(
            object_name=None,
            point=goal_handle.request.object_point,
        )

        action_result = PickTask.Result()
        action_result.success = result.success
        return action_result


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

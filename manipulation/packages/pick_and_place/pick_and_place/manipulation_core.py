#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.action import PickMotion, ManipulationAction
from frida_interfaces.srv import PerceptionService, GraspDetection, DetectionHandler
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    MANIPULATION_ACTION_SERVER,
    PERCEPTION_SERVICE,
    GRASP_DETECTION_SERVICE,
)
from frida_constants.vision_constants import (
    DETECTION_HANDLER_TOPIC_SRV,
)
from pick_and_place.managers.PickManager import PickManager


class ManipulationCore(Node):
    def __init__(self):
        super().__init__("manipulation_core")
        self.callback_group = ReentrantCallbackGroup()

        self._manipulation_server = ActionServer(
            self,
            ManipulationAction,
            MANIPULATION_ACTION_SERVER,
            self.manipulation_server_callback,
            callback_group=self.callback_group,
        )

        self._pick_motion_action_client = ActionClient(
            self,
            PickMotion,
            PICK_MOTION_ACTION_SERVER,
        )

        self.detection_handler_client = self.create_client(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV
        )

        self.perception_3d_client = self.create_client(
            PerceptionService, PERCEPTION_SERVICE
        )

        self.grasp_detection_client = self.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE
        )

        self.pick_manager = PickManager(self)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute(self, object_name=None, object_point=None):
        self.get_logger().info(f"Goal: {object_point}")
        self.get_logger().info("Extracting object cloud")

        result = self.pick_manager.execute(
            object_name=object_name,
            point=object_point,
        )

        return result

    def manipulation_server_callback(self, goal_handle):
        task_type = goal_handle.request.task_type
        object_name = goal_handle.request.object_name
        object_point = goal_handle.request.object_point
        self.get_logger().info(f"Task Type: {task_type}")
        self.get_logger().info(f"Object Name: {object_name}")
        response = ManipulationAction.Result()
        if task_type == ManipulationTask.PICK:
            self.get_logger().info("Executing Pick Task")
            result = self.pick_execute(
                object_name=object_name, object_point=object_point
            )
            goal_handle.succeed()
            response.success = result
        elif task_type == ManipulationTask.PICK_CLOSEST:
            self.get_logger().info("Executing Pick Closest Task")
            goal_handle.succeed()
            response.success = result
        elif task_type == ManipulationTask.PLACE:
            self.get_logger().info("Executing Place Task")
            goal_handle.succeed()
            response.success = result.success
        else:
            self.get_logger().error("Unknown task type")
            goal_handle.abort()
            response = ManipulationAction.Result()
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

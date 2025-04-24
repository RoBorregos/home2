#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_motion_planning.utils.ros_utils import wait_for_future
from std_srvs.srv import SetBool
from frida_interfaces.action import MoveJoints
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.action import PickMotion, PlaceMotion, ManipulationAction
from frida_interfaces.srv import (
    PickPerceptionService,
    GraspDetection,
    DetectionHandler,
    RemoveCollisionObject,
    PlacePerceptionService,
    HeatmapPlace,
)
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PLACE_MOTION_ACTION_SERVER,
    MANIPULATION_ACTION_SERVER,
    PICK_PERCEPTION_SERVICE,
    GRASP_DETECTION_SERVICE,
    MOVE_JOINTS_ACTION_SERVER,
    REMOVE_COLLISION_OBJECT_SERVICE,
    GRIPPER_SET_STATE_SERVICE,
    PLACE_PERCEPTION_SERVICE,
    HEATMAP_PLACE_SERVICE,
)
from frida_constants.vision_constants import (
    DETECTION_HANDLER_TOPIC_SRV,
)
from pick_and_place.managers.PickManager import PickManager
from pick_and_place.managers.PlaceManager import PlaceManager
from frida_interfaces.msg import PickResult


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

        self._place_motion_action_client = ActionClient(
            self,
            PlaceMotion,
            PLACE_MOTION_ACTION_SERVER,
        )

        self._move_joints_client = ActionClient(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
        )

        self.detection_handler_client = self.create_client(
            DetectionHandler, DETECTION_HANDLER_TOPIC_SRV
        )

        self.pick_perception_3d_client = self.create_client(
            PickPerceptionService, PICK_PERCEPTION_SERVICE
        )

        self.place_perception_3d_client = self.create_client(
            PlacePerceptionService, PLACE_PERCEPTION_SERVICE
        )

        self.place_pose_client = self.create_client(HeatmapPlace, HEATMAP_PLACE_SERVICE)

        self.grasp_detection_client = self.create_client(
            GraspDetection, GRASP_DETECTION_SERVICE
        )

        self._remove_collision_object_client = self.create_client(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
        )

        self._gripper_set_state_client = self.create_client(
            SetBool,
            GRIPPER_SET_STATE_SERVICE,
        )

        self.pick_manager = PickManager(self)
        self.pick_result = PickResult()

        self.place_manager = PlaceManager(self)

        self.get_logger().info("Manipulation Core has been started")

    def pick_execute(self, object_name=None, object_point=None):
        self.get_logger().info(f"Goal: {object_point}")
        self.get_logger().info("Extracting object cloud")

        # self.remove_all_collision_object(attached=True)
        # result, pick_result = self.pick_manager.execute(
        #     object_name=object_name,
        #     point=object_point,
        # )

        # if not result:
        #     self.get_logger().error("Pick failed")
        #     self.remove_all_collision_object(attached=True)
        #     return False

        # self.remove_all_collision_object(attached=False)
        # self.get_logger().info("Pick succeeded")
        # return result, pick_result

        try:
            self.remove_all_collision_object(attached=True)
            result, pick_result = self.pick_manager.execute(
                object_name=object_name,
                point=object_point,
            )
            if not result:
                self.get_logger().error("Pick failed")
                self.remove_all_collision_object(attached=True)
                return (False, PickResult())
            return (result, pick_result)
        except Exception:
            return (False, PickResult())

    def manipulation_server_callback(self, goal_handle):
        task_type = goal_handle.request.task_type
        object_name = goal_handle.request.pick_params.object_name
        object_point = goal_handle.request.pick_params.object_point
        self.get_logger().info(f"Task Type: {task_type}")
        self.get_logger().info(f"Object Name: {object_name}")
        response = ManipulationAction.Result()
        if task_type == ManipulationTask.PICK:
            self.get_logger().info("Executing Pick Task")
            result, self.pick_result = self.pick_execute(
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
            result = self.place_manager.execute(
                place_params=goal_handle.request.place_params,
                pick_result=self.pick_result,
            )
            response.success = result
        else:
            self.get_logger().error("Unknown task type")
            goal_handle.abort()
            response = ManipulationAction.Result()
            response.success = False

        return response

    def remove_all_collision_object(self, attached=False):
        """Remove the collision object from the scene."""
        request = RemoveCollisionObject.Request()
        request.id = "all"
        request.include_attached = attached
        self._remove_collision_object_client.wait_for_service()
        future = self._remove_collision_object_client.call_async(request)
        wait_for_future(future)
        return future.result().success


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

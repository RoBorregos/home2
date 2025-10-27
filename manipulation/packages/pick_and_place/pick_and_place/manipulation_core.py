#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.service_utils import (
    get_joint_positions,
    move_joint_positions,
)
from std_srvs.srv import SetBool, Empty
from frida_interfaces.action import MoveJoints
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.action import (
    PickMotion,
    PlaceMotion,
    ManipulationAction,
    PourMotion,
)
from frida_interfaces.srv import (
    PickPerceptionService,
    GraspDetection,
    DetectionHandler,
    RemoveCollisionObject,
    PlacePerceptionService,
    HeatmapPlace,
    GetJoints,
    GetCollisionObjects,
)
from frida_constants.manipulation_constants import (
    PICK_MOTION_ACTION_SERVER,
    PLACE_MOTION_ACTION_SERVER,
    POUR_MOTION_ACTION_SERVER,
    MANIPULATION_ACTION_SERVER,
    PICK_PERCEPTION_SERVICE,
    GRASP_DETECTION_SERVICE,
    MOVE_JOINTS_ACTION_SERVER,
    REMOVE_COLLISION_OBJECT_SERVICE,
    GRIPPER_SET_STATE_SERVICE,
    PLACE_PERCEPTION_SERVICE,
    HEATMAP_PLACE_SERVICE,
    GET_JOINT_SERVICE,
    SCAN_ANGLE_VERTICAL,
    SCAN_ANGLE_HORIZONTAL,
    GET_COLLISION_OBJECTS_SERVICE,
)
from frida_constants.vision_constants import (
    DETECTION_HANDLER_TOPIC_SRV,
)
from pick_and_place.managers.PickManager import PickManager
from pick_and_place.managers.PlaceManager import PlaceManager
from pick_and_place.managers.PourManager import PourManager
from frida_interfaces.msg import PickResult
import time

# tf buffer
from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import PoseStamped
# from frida_pymoveit2.robots import xarm6
# from sensor_msgs.msg import JointState


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

        self._pour_motion_action_client = ActionClient(
            self,
            PourMotion,  # Make sure to import PourMotion
            POUR_MOTION_ACTION_SERVER,
        )

        self._move_joints_client = ActionClient(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
        )

        self._get_joints_client = self.create_client(GetJoints, GET_JOINT_SERVICE)

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

        self._get_collision_objects_client = self.create_client(
            GetCollisionObjects,
            GET_COLLISION_OBJECTS_SERVICE,
        )

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))

        self.tf_listener = TransformListener(
            self.tf_buffer, self, qos=qos, spin_thread=True
        )

        self.pick_manager = PickManager(self)
        self.pick_result = PickResult()
        self.pour_manager = PourManager(self)

        self.place_manager = PlaceManager(self)

        self.octomap_clear_client = self.create_client(Empty, "/clear_octomap")

        self.declare_parameter("octomap_update_during_motion", False)
        self.declare_parameter("octomap_update_rate_slow", 0.2)
        self.declare_parameter("octomap_update_rate_normal", 1.0)

        self.octomap_paused = False

        self.get_logger().info("Manipulation Core has been started")

    def pause_octomap(self):
        if not self.octomap_paused:
            self.get_logger().info("Pausing octomap")

            self.octomap_paused = True

    def resume_octomap_service(self):
        if self.octomap_paused:
            self.get_logger().info("Resuming octomap updates")

            self.octomap_paused = False

    def clear_octomap(self):
        if self.octomap_clear_client.wait_for_service(timeout_sec=1.0):
            request = Empty.Request()
            future = self.octomap_clear_client.call_async(request)
            future = wait_for_future(future, timeout=1.0)
            if future:
                self.get_logger().info("Octomap cleared successfully")
            else:
                self.get_logger().warn("Octomap clear service timeout")
        else:
            self.get_logger().warn("Octomap clear service not available")

    def pick_execute(self, object_name=None, object_point=None, pick_params=None):
        self.get_logger().info(f"Goal: {object_point}")
        self.get_logger().info("Extracting object cloud")

        try:
            self.remove_all_collision_object(attached=True)
            result, pick_result = self.pick_manager.execute(
                object_name=object_name,
                point=object_point,
                pick_params=pick_params,
            )
            if not result:
                self.get_logger().error("Pick failed")
                self.remove_all_collision_object(attached=True)
                return (False, PickResult())

            self.remove_all_collision_object(attached=False)
            self.get_logger().info("[SUCCESS] Pick executed successfully")
            return (result, pick_result)
        except Exception as e:
            self.get_logger().error(f"Pick exception: {e}")
            return (False, PickResult())

    def place_execute(self, place_params=None):
        self.get_logger().info("Executing place")
        try:
            result = self.place_manager.execute(
                place_params=place_params,
                pick_result=self.pick_result,
            )
            if not result:
                self.get_logger().error("Place failed")
                return False
            return result
        except Exception as e:
            self.get_logger().error(f"Place exception: {e}")
            return False

    def pour_execute(self, object_name=None, bowl_name=None):
        # self.get_logger().info(f"Goal: {object_point}")
        self.get_logger().info(f"ObjectName: {object_name}")
        self.get_logger().info(f"BowlName: {bowl_name}")
        self.get_logger().info("Extracting object cloud")

        try:
            self.remove_all_collision_object(attached=True)
            self.get_logger().info("Executing pour100")
            result, pick_result = self.pour_manager.execute(
                object_name=object_name,
                # object_point=object_point,
                container_object_name=bowl_name,
            )
            if not result:
                self.get_logger().error("Pour failed")
                return (False, PickResult())
            self.remove_all_collision_object(attached=False)
            return (result, pick_result)
        except Exception as e:
            self.get_logger().error(f"Pour exeption: {e}")
            return (False, PickResult())

    def manipulation_server_callback(self, goal_handle):
        task_type = goal_handle.request.task_type
        if task_type == ManipulationTask.POUR:
            object_name = goal_handle.request.pour_params.object_name
            bowl_name = goal_handle.request.pour_params.bowl_name
            # object_point = goal_handle.request.pick_params.object_point
        else:
            object_name = goal_handle.request.pick_params.object_name
            object_point = goal_handle.request.pick_params.object_point

        # object_name = goal_handle.request.pick_params.object_name
        # object_point = goal_handle.request.pick_params.object_point
        # bowl_name = goal_handle.request.pour_params.bowl_name
        self.get_logger().info(f"Task Type: {task_type}")
        self.get_logger().info(f"Object Name: {object_name}")

        response = ManipulationAction.Result()

        self.clear_octomap()

        if goal_handle.request.scan_environment:
            self.get_logger().info("Scanning environment")
            self.scan_environment()
            # give time to see
            time.sleep(2.0)

        if task_type == ManipulationTask.PICK:
            self.get_logger().info("Executing Pick Task")
            result, self.pick_result = self.pick_execute(
                object_name=object_name,
                object_point=object_point,
                pick_params=goal_handle.request.pick_params,
            )
            goal_handle.succeed()
            response.success = result
        elif task_type == ManipulationTask.PICK_CLOSEST:
            self.get_logger().info("Executing Pick Closest Task")
            goal_handle.succeed()
            response.success = result
        elif task_type == ManipulationTask.PLACE:
            self.get_logger().info("Executing Place Task")
            result = self.place_execute(place_params=goal_handle.request.place_params)
            goal_handle.succeed()
            if result:
                self.remove_all_collision_object(attached=True)
            response.success = result
        elif task_type == ManipulationTask.POUR:
            self.get_logger().info("Executing Pour Task")
            result, self.pick_result = self.pour_execute(
                object_name=object_name,
                bowl_name=bowl_name,
                # , object_point=object_point
            )
            goal_handle.succeed()
            if result:
                self.remove_all_collision_object(attached=True)
            response.success = result
        else:
            self.get_logger().error("Unknown task type")
            goal_handle.succeed()
            response = ManipulationAction.Result()
            response.success = False
        self.get_logger().info(f"[DONE] Manipulation Task Result: {response}")
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

    # def clear_octomap(self):
    #     """Clear the octomap."""
    #     request = Empty.Request()
    #     self.get_logger().info("Clearing octomap")
    #     future = self._clear_octomap_client.call_async(request)
    #     wait_for_future(future)
    #     return True

    def scan_environment(self):
        """Scan the environment and update the octomap."""
        curr_joint_positions = get_joint_positions(
            self._get_joints_client, degrees=True
        )
        self.get_logger().info(f"Current joint positions: {curr_joint_positions}")
        curr_joint_positions["joints"]["joint1"] += SCAN_ANGLE_HORIZONTAL
        curr_joint_positions["joints"]["joint5"] -= SCAN_ANGLE_VERTICAL
        self.get_logger().info(f"Moving to: {curr_joint_positions}")
        move_joint_positions(
            move_joints_action_client=self._move_joints_client,
            joint_positions=curr_joint_positions,
            velocity=0.2,
        )
        curr_joint_positions["joints"]["joint1"] -= SCAN_ANGLE_HORIZONTAL * 2
        move_joint_positions(
            move_joints_action_client=self._move_joints_client,
            joint_positions=curr_joint_positions,
            velocity=0.2,
        )
        curr_joint_positions["joints"]["joint1"] += SCAN_ANGLE_HORIZONTAL
        curr_joint_positions["joints"]["joint5"] += SCAN_ANGLE_VERTICAL
        move_joint_positions(
            move_joints_action_client=self._move_joints_client,
            joint_positions=curr_joint_positions,
            velocity=0.2,
        )


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = ManipulationCore()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

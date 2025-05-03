#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs import Pose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool

from frida_constants.manipulation_constants import (
    ATTACH_COLLISION_OBJECT_SERVICE,
    GET_COLLISION_OBJECTS_SERVICE,
    # PICK_OBJECT_NAMESPACE,
    # PLANE_NAMESPACE,
    # EEF_LINK_NAME,
    # EEF_CONTACT_LINKS,
    # PICK_MOTION_ACTION_SERVER,
    # PLANE_OBJECT_COLLISION_TOLERANCE,
    # SAFETY_HEIGHT,
    # PICK_MIN_HEIGHT,
    GRASP_LINK_FRAME,
    GRIPPER_SET_STATE_SERVICE,
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_PLANNER,
    POUR_ACCELERATION,
    POUR_MOTION_ACTION_SERVER,
    POUR_VELOCITY,
    REMOVE_COLLISION_OBJECT_SERVICE,
)
from frida_interfaces.action import MoveToPose, PourMotion
from frida_interfaces.msg import Constraint
from frida_interfaces.srv import (
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)

# from frida_interfaces.msg import PickResult
# import copy
# import numpy as np
# from transforms3d.quaternions import quat2mat
# from frida_motion_planning.utils.service_utils import (
#     close_gripper,
# )
# import time


class PourMotionServer(Node):
    def __init__(self):
        super().__init__("pour_server")
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PourMotion,
            POUR_MOTION_ACTION_SERVER,
            self.execute_callback,
            callback_group=self.callback_group,
        )

        self._move_to_pose_action_client = ActionClient(
            self,
            MoveToPose,
            MOVE_TO_POSE_ACTION_SERVER,
        )

        self._attach_collision_object_client = self.create_client(
            AttachCollisionObject,
            ATTACH_COLLISION_OBJECT_SERVICE,
        )

        self._get_collision_objects_client = self.create_client(
            GetCollisionObjects,
            GET_COLLISION_OBJECTS_SERVICE,
        )

        self._remove_collision_object_client = self.create_client(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
        )

        self._gripper_set_state_client = self.create_client(
            SetBool,
            GRIPPER_SET_STATE_SERVICE,
        )

        self._move_to_pose_action_client.wait_for_server()

        self.get_logger().info("Pour Action Server has been started")

    # // Add Primitive false for the goal
    async def execute_callback(self, goal_handle):
        """Execute the pour action when a goal is received."""
        self.get_logger().info("Executing pour goal...")

        # Initialize result
        feedback = PourMotion.Feedback()
        result = PourMotion.Result()
        try:
            (result.success,) = self.pour(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pour failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    # TODO: APPLY LOGIC TO THIS FUNCTION
    def pour(self, goal_handle, feedback):
        """Perform the pour operation."""
        self.get_logger().info(
            f"Trying to pour object: {goal_handle.request.object_name}"
        )
        # Obtain the desired pose
        bowl_position = goal_handle.request.bowl_position.point
        abs_object_height = abs(
            goal_handle.request.object_top_height
            - goal_handle.request.object_centroid_height
        )
        abs_bowl_height = abs(
            goal_handle.request.bowl_top_height
            - goal_handle.request.bowl_position.point.z
        )

        pour_pose = Pose()
        # Set position
        pour_pose.pose.position.z += bowl_position.point.z + abs_bowl_height + 0.03
        pour_pose.pose.position.x += bowl_position.x
        pour_pose.pose.position.y += bowl_position.y + (abs_object_height) * (
            np.sin(45)
        )
        # Set orientation
        pour_pose.pose.orientation.w = 1.0
        pour_pose.pose.orientation.x = 0.0
        pour_pose.pose.orientation.y = 0.0
        pour_pose.pose.orientation.z = 0.0

        # Set Frame ID
        # pour_pose.header.frame_id = goal_handle.request.pour_pose.header.frame_id
        self.get_logger().info(f"Pour pose: {pour_pose}")

        # Move to pour pose
        pour_pose_handler, pour_pose_result = self.move_to_pose(pour_pose, True)
        if not pour_pose_result.result.success:
            self.get_logger().error("Failed to reach pour pose")
            result = False
            return False, result
        self.get_logger().info("Pour pose reached")

        # Set the new orientation without constraint
        pour_pose.pose.orientation.w = 1.0
        pour_pose.pose.orientation.x = 0.0
        pour_pose.pose.orientation.y = 0.0
        pour_pose.pose.orientation.z = 0.0

        self.get_logger().info(f"Pour final orientation: {pour_pose.orientation}")

        pour_pose_handler, pour_pose_result = self.move_to_pose(pour_pose, True)
        if not pour_pose_result.result.success:
            self.get_logger().error("Failed to reach pour final orientation")
            result = False
            return False, result
        self.get_logger().info("Pour final orientation reached")

        return True

    def move_to_pose(self, pose, useConstraint: bool = False):
        """Move the robot to the given pose."""
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = POUR_VELOCITY
        request.acceleration = POUR_ACCELERATION
        request.planner_id = PICK_PLANNER
        request.apply_constraint = useConstraint
        request.target_link = GRASP_LINK_FRAME

        # Set the constraint
        if useConstraint:
            constraint_msg = Constraint()
            constraint_msg.orientation = pose.orientation
            constraint_msg.frame_id = PICK_PLANNER
            constraint_msg.target_link = GRASP_LINK_FRAME
            # Modify in case need to modify the limits for the path
            constraint_msg.tolerance_orientation = [3.14159, 0.5, 0.5]
            constraint_msg.weight = 0.5
            constraint_msg.parameterization = 1
            request.constraint = constraint_msg

        future = self._move_to_pose_action_client.send_goal_async(request)
        self.wait_for_future(future)
        action_result = future.result().get_result()
        return future.result(), action_result

    def wait_for_future(self, future):
        if future is None:
            self.get_logger().error("Service call failed: future is None")
            return False
        while not future.done():
            pass
        # self.get_logger().info("Execution done with status: " + str(future.result()))
        return future  # 4 is the status for success

    def get_collision_objects(self):
        """Get the collision objects in the scene."""
        request = GetCollisionObjects.Request()
        future = self._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)
        return future.result().collision_objects

    def remove_collision_object(self, id):
        """Remove the collision object from the scene."""
        request = RemoveCollisionObject.Request()
        request.id = id
        self._remove_collision_object_client.wait_for_service()
        future = self._remove_collision_object_client.call_async(request)
        self.wait_for_future(future)
        return future.result().success


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PourMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

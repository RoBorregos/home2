#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from frida_constants.manipulation_constants import (
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_VELOCITY,
    PICK_ACCELERATION,
    PICK_PLANNER,
    ATTACH_COLLISION_OBJECT_SERVICE,
    REMOVE_COLLISION_OBJECT_SERVICE,
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
    POUR_MOTION_ACTION_SERVER,
)
from frida_interfaces.srv import (
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)
from frida_interfaces.action import PourMotion, MoveToPose

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
        pour_pose = goal_handle.request.pour_pose
        pour_pose.pose.position.z += 0.1
        pour_pose.pose.position.x += 0.1
        pour_pose.pose.position.y += 0.1
        pour_pose.pose.orientation.w = 1.0
        pour_pose.pose.orientation.x = 0.0
        pour_pose.pose.orientation.y = 0.0
        pour_pose.pose.orientation.z = 0.0
        pour_pose.header.frame_id = goal_handle.request.pour_pose.header.frame_id
        self.get_logger().info(f"Pour pose: {pour_pose}")
        # Move to pour pose
        pour_pose_handler, pour_pose_result = self.move_to_pose(pour_pose)
        if not pour_pose_result.result.success:
            self.get_logger().error("Failed to reach pour pose")
            result = False
            return False, result
        self.get_logger().info("Pour pose reached")

        return True

    def move_to_pose(self, pose):
        """Move the robot to the given pose."""
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = PICK_VELOCITY
        request.acceleration = PICK_ACCELERATION
        request.planner_id = PICK_PLANNER
        request.target_link = GRASP_LINK_FRAME
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

    # def save_collision_objects(self):
    #     self.collision_objects = self.get_collision_objects()

    # def find_plane(self):
    #     # find plane object
    #     self.plane = None
    #     for obj in self.collision_objects:
    #         if PLANE_NAMESPACE in obj.id:
    #             self.get_logger().info(f"Plane object found: {obj.id}")
    #             self.plane = obj

    # def attach_pick_object(self):
    #     """Attach the pick object to the robot."""
    #     obj_lowest = None
    #     obj_highest = None
    #     for obj in self.collision_objects:
    #         if PICK_OBJECT_NAMESPACE in obj.id:
    #             request = AttachCollisionObject.Request()
    #             request.id = obj.id
    #             if obj_lowest is None:
    #                 obj_lowest = obj
    #             else:
    #                 if obj.pose.pose.position.z < obj_lowest.pose.pose.position.z:
    #                     obj_lowest = obj
    #             if obj_highest is None:
    #                 obj_highest = obj
    #             else:
    #                 if obj.pose.pose.position.z > obj_highest.pose.pose.position.z:
    #                     obj_highest = obj
    #             if self.plane is not None and self.object_in_plane(obj, self.plane):
    #                 self.remove_collision_object(obj.id)
    #                 continue
    #             request.attached_link = EEF_LINK_NAME
    #             request.touch_links = EEF_CONTACT_LINKS
    #             request.detach = False
    #             self._attach_collision_object_client.wait_for_service()
    #             future = self._attach_collision_object_client.call_async(request)
    #             self.wait_for_future(future)
    #     return True, obj_lowest, obj_highest

    def get_collision_objects(self):
        """Get the collision objects in the scene."""
        request = GetCollisionObjects.Request()
        future = self._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)
        return future.result().collision_objects

    # def object_in_plane(self, obj, plane):
    #     """Check if the object is in the plane."""
    #     plane_top_height = plane.pose.pose.position.z + plane.dimensions.z / 2
    #     return (
    #         obj.pose.pose.position.z
    #         < plane_top_height + PLANE_OBJECT_COLLISION_TOLERANCE
    #     )

    def remove_collision_object(self, id):
        """Remove the collision object from the scene."""
        request = RemoveCollisionObject.Request()
        request.id = id
        self._remove_collision_object_client.wait_for_service()
        future = self._remove_collision_object_client.call_async(request)
        self.wait_for_future(future)
        return future.result().success

    # def calculate_object_pick_height(self, obj, pose):
    #     """Calculate the height of the object, measured from where it was picked
    #     e.g. if a 30cm tall object is picked at 10cm, the height is 10cm
    #     -> Reason for this is to know how high we should be to place the object, basically repeat same height"""
    #     if obj.pose.header.frame_id != pose.header.frame_id:
    #         self.get_logger().error(
    #             "Object and pose frames do not match, cannot calculate height"
    #         )
    #         return 0.0
    #     obj_z = obj.pose.pose.position.z
    #     obj_radius = obj.dimensions.x
    #     grasp_height = pose.pose.position.z
    #     # assume lowest part of the object is lowest bound of lowest sphere
    #     height = grasp_height - (obj_z - obj_radius) + SAFETY_HEIGHT
    #     self.get_logger().info(f"Object pick height: {height}")
    #     return height

    # def calculate_object_height(self, obj_lowest, obj_highest):
    #     """Calculate the height of the object, measured from the lowest point to the highest point"""
    #     if obj_lowest.pose.header.frame_id != obj_highest.pose.header.frame_id:
    #         self.get_logger().error(
    #             "Object and pose frames do not match, cannot calculate height"
    #         )
    #         return 0.0
    #     obj_lowest_z = obj_lowest.pose.pose.position.z
    #     obj_highest_z = obj_highest.pose.pose.position.z
    #     obj_radius = obj_lowest.dimensions.x
    #     height = (obj_highest_z + obj_radius) - (obj_lowest_z - obj_radius)
    #     self.get_logger().info(f"Object height: {height}")
    #     return height

    # def check_feasibility(self, pose):
    #     """Check if the pose is feasible."""
    #     pick_height = pose.pose.position.z
    #     plane_height = self.plane.pose.pose.position.z + self.plane.dimensions.z / 2
    #     if pick_height < plane_height + PICK_MIN_HEIGHT:
    #         self.get_logger().warn(
    #             f"Pick height {pick_height} is below acceptable height, plane height is {plane_height}"
    #         )
    #         return False
    #     return True


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PourMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

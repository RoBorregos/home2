#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_constants.manipulation_constants import (
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_VELOCITY,
    PICK_ACCELERATION,
    PICK_PLANNER,
    ATTACH_COLLISION_OBJECT_SERVICE,
    REMOVE_COLLISION_OBJECT_SERVICE,
    GET_COLLISION_OBJECTS_SERVICE,
    PICK_OBJECT_NAMESPACE,
    EEF_LINK_NAME,
    EEF_CONTACT_LINKS,
    PLACE_MOTION_ACTION_SERVER,
    AIM_STRAIGHT_FRONT_QUAT,
    SHELF_POSITION_PREPLACE_POSE,
)
from frida_interfaces.srv import (
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)
from frida_interfaces.action import PlaceMotion, MoveToPose
import copy
import numpy as np
from transforms3d.quaternions import quat2mat


class PlaceMotionServer(Node):
    def __init__(self):
        super().__init__("place_server")
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PlaceMotion,
            PLACE_MOTION_ACTION_SERVER,
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

        self._move_to_pose_action_client.wait_for_server()

        self.get_logger().info("Place Action Server has been started")

    async def execute_callback(self, goal_handle):
        """Execute the place action when a goal is received."""
        self.get_logger().info("Executing place goal...")

        # Initialize result
        feedback = PlaceMotion.Feedback()
        result = PlaceMotion.Result()
        try:
            result.success = self.place(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Place failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    def place(self, goal_handle, feedback):
        """Perform the place operation."""
        self.get_logger().info(
            f"Trying to place up object: {goal_handle.request.object_name}"
        )
        place_pose = goal_handle.request.place_pose
        is_shelf = goal_handle.request.place_params.is_shelf

        # generate several place_poses, so try every time higher from plane
        n_poses = 5
        poses_dist = 0.03
        place_poses = []
        for i in range(n_poses):
            ee_link_pose = copy.deepcopy(place_pose)
            ee_link_pose.pose.position.z += i * poses_dist

            ee_link_pre_pose = copy.deepcopy(ee_link_pose)

            if is_shelf:
                # send it a little bit back, then forward
                offset_distance = SHELF_POSITION_PREPLACE_POSE  # Desired distance in meters along the local z-axis

                # Compute the offset along the local z-axis
                quat = [
                    ee_link_pre_pose.pose.orientation.w,
                    ee_link_pre_pose.pose.orientation.x,
                    ee_link_pre_pose.pose.orientation.y,
                    ee_link_pre_pose.pose.orientation.z,
                ]
                rotation_matrix = quat2mat(quat)
                # Extract local Z-axis (third column of the rotation matrix)
                z_axis = rotation_matrix[:, 2]

                # Translate along the local Z-axis
                new_position = (
                    np.array(
                        [
                            ee_link_pre_pose.pose.position.x,
                            ee_link_pre_pose.pose.position.y,
                            ee_link_pre_pose.pose.position.z,
                        ]
                    )
                    + z_axis * offset_distance
                )
                ee_link_pre_pose.pose.position.x = new_position[0]
                ee_link_pre_pose.pose.position.y = new_position[1]
                ee_link_pre_pose.pose.position.z = new_position[2]

            place_poses.append([ee_link_pre_pose, ee_link_pose])

        for i, poses in enumerate(place_poses):
            # Move to pre-grasp pose

            ee_link_pre_pose = poses[0]
            ee_link_pose = poses[1]
            if is_shelf:
                ee_link_pose.pose.orientation.x = AIM_STRAIGHT_FRONT_QUAT[0]
                ee_link_pose.pose.orientation.y = AIM_STRAIGHT_FRONT_QUAT[1]
                ee_link_pose.pose.orientation.z = AIM_STRAIGHT_FRONT_QUAT[2]
                ee_link_pose.pose.orientation.w = AIM_STRAIGHT_FRONT_QUAT[3]
                place_pose_handler, place_pose_result = self.move_to_pose(
                    ee_link_pre_pose
                )
                if not place_pose_result.result.success:
                    self.get_logger().error("Failed to reach pre-place pose")
                    continue
            place_pose_handler, place_pose_result = self.move_to_pose(ee_link_pose)
            print(f"Grasp Pose {i} result: {place_pose_result}")
            if place_pose_result.result.success:
                self.get_logger().info("Grasp pose reached")
                self.deattach_pick_object()

                # close gripper
                # gripper_request = SetBool.Request()
                # gripper_request.data = False
                # self.node.get_logger().info(
                #     "Closing gripper :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
                # )
                # future = self.node._gripper_set_state_client.call_async(gripper_request)
                # future = wait_for_future(future)
                # result = future.result()
                # self.node.get_logger().info(f"Gripper Result: {str(gripper_request.data)}")

                # self.node.get_logger().info(
                #     "Returning to position............................................."
                # )

                return True
        self.get_logger().error("Failed to reach any grasp pose")
        return False

    def move_to_pose(self, pose):
        """Move the robot to the given pose."""
        self.get_logger().info(f"Moving to pose: {pose}")
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = PICK_VELOCITY
        request.acceleration = PICK_ACCELERATION
        request.planner_id = PICK_PLANNER
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

    def deattach_pick_object(self):
        """Attach the pick object to the robot."""
        collision_objects = self.get_collision_objects()
        for obj in collision_objects:
            if PICK_OBJECT_NAMESPACE in obj.id:
                request = AttachCollisionObject.Request()
                request.id = obj.id
                request.attached_link = EEF_LINK_NAME
                request.touch_links = EEF_CONTACT_LINKS
                request.detach = True
                self._attach_collision_object_client.wait_for_service()
                future = self._attach_collision_object_client.call_async(request)
                self.wait_for_future(future)
        return True

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
    place_server = PlaceMotionServer()
    executor.add_node(place_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

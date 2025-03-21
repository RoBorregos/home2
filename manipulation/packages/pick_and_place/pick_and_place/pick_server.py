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
    GET_COLLISION_OBJECTS_SERVICE,
    PICK_OBJECT_NAMESPACE,
    EEF_LINK_NAME,
    EEF_CONTACT_LINKS,
    PICK_MOTION_ACTION_SERVER,
)
from frida_interfaces.srv import AttachCollisionObject, GetCollisionObjects
from frida_interfaces.action import PickMotion, MoveToPose


class PickMotionServer(Node):
    def __init__(self):
        super().__init__("pick_server")
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PickMotion,
            PICK_MOTION_ACTION_SERVER,
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

        self._move_to_pose_action_client.wait_for_server()

        self.get_logger().info("Pick Action Server has been started")

    async def execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pick goal...")

        # Initialize result
        feedback = PickMotion.Feedback()
        result = PickMotion.Result()
        try:
            result.success = self.pick(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    def pick(self, goal_handle, feedback):
        """Perform the pick operation."""
        self.get_logger().info(
            f"Trying to pick up object: {goal_handle.request.object_name}"
        )
        grasping_poses = goal_handle.request.grasping_poses
        for i, pose in enumerate(grasping_poses):
            # Move to pre-grasp pose
            grasp_pose_handler, grasp_pose_result = self.move_to_pose(pose)
            print(f"Grasp Pose {i} result: {grasp_pose_result}")
            if grasp_pose_result.result.success:
                self.get_logger().info("Grasp pose reached")
                self.attach_pick_object()
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
        print("Waiting future not none")
        if future is None:
            self.get_logger().error("Service call failed: future is None")
            return False
        print("Waiting future done")
        while not future.done():
            pass
        # self.get_logger().info("Execution done with status: " + str(future.result()))
        return future  # 4 is the status for success

    def attach_pick_object(self):
        """Attach the pick object to the robot."""
        collision_objects = self.get_collision_objects()
        print(collision_objects)
        for obj in collision_objects:
            print("Found object: ", obj.id)
            if PICK_OBJECT_NAMESPACE in obj.id:
                request = AttachCollisionObject.Request()
                request.id = obj.id
                request.attached_link = EEF_LINK_NAME
                request.touch_links = EEF_CONTACT_LINKS
                request.detach = False
                self._attach_collision_object_client.wait_for_service()
                future = self._attach_collision_object_client.call_async(request)
                self.wait_for_future(future)
                if future.result().success:
                    self.get_logger().info(f"Object {obj.id} attached to robot")
                else:
                    self.get_logger().error(f"Failed to attach object {obj.id}")
        return True

    def get_collision_objects(self):
        """Get the collision objects in the scene."""
        request = GetCollisionObjects.Request()
        future = self._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)
        return future.result().collision_objects


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PickMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

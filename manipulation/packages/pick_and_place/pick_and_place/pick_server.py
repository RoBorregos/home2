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
    PLANE_NAMESPACE,
    EEF_LINK_NAME,
    EEF_CONTACT_LINKS,
    PICK_MOTION_ACTION_SERVER,
    PLANE_OBJECT_COLLISION_TOLERANCE,
    SAFETY_HEIGHT,
    PICK_MIN_HEIGHT,
    GRASP_LINK_FRAME,
)
from frida_interfaces.srv import (
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)
from frida_interfaces.action import PickMotion, MoveToPose
from frida_interfaces.msg import PickResult
import copy
import numpy as np
from transforms3d.quaternions import quat2mat


class PickMotionServer(Node):
    def __init__(self):
        super().__init__("pick_server")
        self.callback_group = ReentrantCallbackGroup()

        # Declare and retrieve the parameter for the end-effector link offset
        self.declare_parameter("ee_link_offset", -0.125)
        self.ee_link_offset = self.get_parameter("ee_link_offset").value
        self.get_logger().info(f"End-effector link offset: {self.ee_link_offset} m")

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

        self._remove_collision_object_client = self.create_client(
            RemoveCollisionObject,
            REMOVE_COLLISION_OBJECT_SERVICE,
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
            result.success, result.pick_result = self.pick(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            goal_handle.succeed()
            result.success = False
            return result

    def pick(self, goal_handle, feedback):
        """Perform the pick operation."""
        self.get_logger().info(
            f"Trying to pick up object: {goal_handle.request.object_name}"
        )
        pick_result = PickResult()
        grasping_poses = goal_handle.request.grasping_poses

        # Apparently GPD always sends me somewhat at the same distance to the object, so lets try different distances
        num_grasping_alternatives = (
            3  # for each grasping pose, try 4 alternatives from closest to farthest
        )
        grasping_alternative_distance = -0.02  # 5mm distance

        self.save_collision_objects()
        self.find_plane()
        if self.plane is None:
            self.get_logger().error("No plane found, cannot pick object")
            return False, pick_result

        for i, pose in enumerate(grasping_poses):
            # Move to pre-grasp pose

            for j in range(num_grasping_alternatives):
                ee_link_pose = copy.deepcopy(pose)

                offset_distance = (
                    self.ee_link_offset
                )  # Desired distance in meters along the local z-axis

                offset_distance += j * grasping_alternative_distance

                # Compute the offset along the local z-axis
                quat = [
                    ee_link_pose.pose.orientation.w,
                    ee_link_pose.pose.orientation.x,
                    ee_link_pose.pose.orientation.y,
                    ee_link_pose.pose.orientation.z,
                ]
                rotation_matrix = quat2mat(quat)
                # Extract local Z-axis (third column of the rotation matrix)
                z_axis = rotation_matrix[:, 2]

                # Translate along the local Z-axis
                new_position = (
                    np.array(
                        [
                            ee_link_pose.pose.position.x,
                            ee_link_pose.pose.position.y,
                            ee_link_pose.pose.position.z,
                        ]
                    )
                    + z_axis * offset_distance
                )
                ee_link_pose.pose.position.x = new_position[0]
                ee_link_pose.pose.position.y = new_position[1]
                ee_link_pose.pose.position.z = new_position[2]

                if not self.check_feasibility(ee_link_pose):
                    self.get_logger().warn(
                        f"Grasping alternative {j} is not feasible, skipping"
                    )
                    continue

                grasp_pose_handler, grasp_pose_result = self.move_to_pose(ee_link_pose)

                print(f"Grasp Pose {i} result: {grasp_pose_result}")
                if grasp_pose_result.result.success:
                    self.get_logger().info("Grasp pose reached")
                    result, lowest_obj = self.attach_pick_object()
                    if result:
                        self.get_logger().info("Object attached")
                        # Save object details
                        pick_result.pick_pose = ee_link_pose
                        pick_result.grasp_score = goal_handle.request.grasping_scores[i]
                        pick_result.object_pick_height = (
                            self.calculate_object_pick_height(lowest_obj, ee_link_pose)
                        )
                    else:
                        self.get_logger().error("Failed to attach object")
                    return True, pick_result

        self.get_logger().error("Failed to reach any grasp pose")
        return False, pick_result

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

    def save_collision_objects(self):
        self.collision_objects = self.get_collision_objects()

    def find_plane(self):
        # find plane object
        self.plane = None
        for obj in self.collision_objects:
            if PLANE_NAMESPACE in obj.id:
                self.get_logger().info(f"Plane object found: {obj.id}")
                self.plane = obj

    def attach_pick_object(self):
        """Attach the pick object to the robot."""
        obj_lowest = None
        for obj in self.collision_objects:
            if PICK_OBJECT_NAMESPACE in obj.id:
                request = AttachCollisionObject.Request()
                request.id = obj.id
                if obj_lowest is None:
                    obj_lowest = obj
                else:
                    if obj.pose.pose.position.z < obj_lowest.pose.pose.position.z:
                        obj_lowest = obj
                if self.plane is not None and self.object_in_plane(obj, self.plane):
                    self.remove_collision_object(obj.id)
                    continue
                request.attached_link = EEF_LINK_NAME
                request.touch_links = EEF_CONTACT_LINKS
                request.detach = False
                self._attach_collision_object_client.wait_for_service()
                future = self._attach_collision_object_client.call_async(request)
                self.wait_for_future(future)
        return True, obj_lowest

    def get_collision_objects(self):
        """Get the collision objects in the scene."""
        request = GetCollisionObjects.Request()
        future = self._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)
        return future.result().collision_objects

    def object_in_plane(self, obj, plane):
        """Check if the object is in the plane."""
        plane_top_height = plane.pose.pose.position.z + plane.dimensions.z / 2
        return (
            obj.pose.pose.position.z
            < plane_top_height + PLANE_OBJECT_COLLISION_TOLERANCE
        )

    def remove_collision_object(self, id):
        """Remove the collision object from the scene."""
        request = RemoveCollisionObject.Request()
        request.id = id
        self._remove_collision_object_client.wait_for_service()
        future = self._remove_collision_object_client.call_async(request)
        self.wait_for_future(future)
        return future.result().success

    def calculate_object_pick_height(self, obj, pose):
        """Calculate the height of the object, measured from where it was picked
        e.g. if a 30cm tall object is picked at 10cm, the height is 10cm
        -> Reason for this is to know how high we should be to place the object, basically repeat same height"""
        if obj.pose.header.frame_id != pose.header.frame_id:
            self.get_logger().error(
                "Object and pose frames do not match, cannot calculate height"
            )
            return 0.0
        obj_z = obj.pose.pose.position.z
        obj_radius = obj.dimensions.x
        grasp_height = pose.pose.position.z
        # assume lowest part of the object is lowest bound of lowest sphere
        height = grasp_height - (obj_z - obj_radius) + SAFETY_HEIGHT
        self.get_logger().info(f"Object pick height: {height}")
        return height

    def check_feasibility(self, pose):
        """Check if the pose is feasible."""
        pick_height = pose.pose.position.z
        plane_height = self.plane.pose.pose.position.z + self.plane.dimensions.z / 2
        if pick_height < plane_height + PICK_MIN_HEIGHT:
            self.get_logger().warn(
                f"Pick height {pick_height} is below acceptable height, plane height is {plane_height}"
            )
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PickMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

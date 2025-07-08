#!/usr/bin/env python3
import time

# import numpy as np
import rclpy

# from geometry_msgs.msg import Pose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from frida_pymoveit2.robots import xarm6
from frida_motion_planning.utils.service_utils import (
    move_joint_positions, get_joint_positions
)
from transforms3d.quaternions import quat2mat

import numpy as np
# from frida_interfaces.srv import GetJoints

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
    MOVE_JOINTS_ACTION_SERVER,
    GRASP_LINK_FRAME,
    GRIPPER_SET_STATE_SERVICE,
    MOVE_TO_POSE_ACTION_SERVER,
    PICK_PLANNER,
    POUR_ACCELERATION,
    POUR_MOTION_ACTION_SERVER,
    POUR_VELOCITY,
    REMOVE_COLLISION_OBJECT_SERVICE,
)
from frida_interfaces.action import MoveToPose, PourMotion, MoveJoints
from frida_interfaces.msg import Constraint
from frida_interfaces.srv import (
    # GetJoints,
    AttachCollisionObject,
    GetCollisionObjects,
    RemoveCollisionObject,
)
from frida_interfaces.srv import GetJoints

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
        self.joint_states = None  # Aquí se almacenarán las posiciones

        self.create_subscription(
            JointState,
            "/joint_states",  # Asegúrate de que este sea el tópico correcto
            self.joint_states_callback,
            10,
        )

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
        
        self._move_joints_action_client = ActionClient(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
        )
        
        self._get_joints_client = self.create_client(GetJoints, "/manipulation/get_joints")

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
            self.get_logger().info("Pouring...........................")
            success, _ = self.pour(goal_handle, feedback)
            result.success = int(success)

            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pour failed: {str(e)}")
            goal_handle.abort()
            result.success = 0
            return result


    def pour(self, goal_handle, feedback):
        """Perform the pour operation."""
        self.get_logger().info("Trying to pour object")
        isConstrained = (
            True  # THE MOTION PLANNER IS CONSTRAINED ------------------------
        )

        pose = self.receive_pose(goal_handle.request.pour_pose)
        
        pose = self.transform_pose_to_gripper_center(pose)

        tries = 5
        distance_between_tries=0.025
        for i in range(tries):
            # self.get_logger().warn(f"Trying to pour object: {self.node.pour.request.object_name}")
            self.get_logger().warn("Trying to pour object")
            pose.pose.position.z += distance_between_tries
            self.get_logger().info(f"Pour pose: {pose.pose}")
            # call the move_to_pose function
            try: 
                goal_handle_result, action_result = self.move_to_pose(pose, isConstrained, planning_time=5.0, planning_attempts=20)
            except Exception as e:
                self.get_logger().error(f"Failed to move to pour pose: {e}")
                return False, None
            
            # self.get_logger().info(
            #     f"Pour pose result: ({goal_handle_result}, {action_result})"
            # )
            if action_result.result.success:
                self.get_logger().info("Pour pose reached")
                break 
            else:
                self.get_logger().error("Failed to reach pour pose")
                if i == tries - 1:
                    self.get_logger().error("Max tries reached")
                    return False, None
                else:
                    self.get_logger().info(
                        f"Retrying pour pose: {pose.pose}"
                    )
        time.sleep(2.0)
        pour_angle = 2.5
        
        current_joints = get_joint_positions(
            self._get_joints_client,
            degrees=False,  # set to true to return in degrees
        )
        current_joints["joints"]["joint6"] += pour_angle
        
        action_result = move_joint_positions(
            move_joints_action_client=self._move_joints_action_client,
            joint_positions=current_joints,
            velocity=POUR_VELOCITY,
            wait=True,
        )
        
        if action_result is None:
            self.get_logger().error("Failed to get action result")
            return False, action_result.result.success
        if not action_result.result.success:
            self.get_logger().error("Failed to move to pour pose")
            self.get_logger().warning(
                f"Action result: {action_result.result.error_code}"
            )
            return False, action_result.result.success
        self.get_logger().info("Moved to pour pose successfully")
        # Move to the updated pose
        return True, action_result.result.success
    
    def transform_pose_to_gripper_center(self, pose):
        offset_distance = (
            -0.15
        )  # Desired distance in meters along the local z-axis

        offset_distance

        # Compute the offset along the local z-axis
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        rotation_matrix = quat2mat(quat)
        # Extract local Z-axis (third column of the rotation matrix)
        z_axis = rotation_matrix[:, 2]

        # Translate along the local Z-axis
        new_position = (
            np.array(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
            )
            + z_axis * offset_distance
        )
        # Update the pose with the new position
        pose.pose.position.x = new_position[0]
        pose.pose.position.y = new_position[1]
        pose.pose.position.z = new_position[2]
        
        return pose
    
    def move_to_pose(self, pose, useConstraint: bool = False, planning_time: float = 0.5, planning_attempts: int = 5):
        """Move the robot to the given pose."""
        self.get_logger().warn(f"Moving to pose: {pose}")
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = POUR_VELOCITY
        request.acceleration = POUR_ACCELERATION
        request.planner_id = PICK_PLANNER
        request.apply_constraint = useConstraint
        request.target_link = GRASP_LINK_FRAME
        request.planning_time = planning_time  # Set the planning time for the action
        request.planning_attempts = planning_attempts  # Set the number of planning attempts

        # Set the constraint
        if useConstraint:
            constraint_msg = Constraint()
            constraint_msg.orientation = pose.pose.orientation
            constraint_msg.frame_id = pose.header.frame_id
            constraint_msg.target_link = GRASP_LINK_FRAME
            # Modify in case need to modify the limits for the path
            constraint_msg.tolerance_orientation = [1.0, 1.0, 1.0]
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

    def receive_pose(self, pose):
        """Receive the pose from the client."""
        self.get_logger().info(f"Received pose: {pose}")
        return pose

    def joint_states_callback(self, msg: JointState):
        # Filtra solo los joints relevantes del xarm6
        relevant_indices = [
            i
            for i, name in enumerate(msg.name)
            if name in xarm6.joint_names()  # usa esta función del paquete
        ]
        if not relevant_indices:
            return
        self.joint_states = JointState()
        self.joint_states.name = [msg.name[i] for i in relevant_indices]
        self.joint_states.position = [msg.position[i] for i in relevant_indices]


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PourMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

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
    # GetJoints,
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
        self.get_logger().info("Pouring...")
        try:
            self.get_logger().info("Pouring...........................")
            success, _ = self.pour(goal_handle, feedback)
            result.success = int(success)
            
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pour failed: {str(e)}")
            goal_handle.abort()
            result.success =  0
            return result

    # TODO: APPLY LOGIC TO THIS FUNCTION
    def pour(self, goal_handle, feedback):
        """Perform the pour operation."""
        self.get_logger().info(
            "Trying to pour object"
        )
        isConstrained = (
            False  # THE MOTION PLANNER IS CONSTRAINED ------------------------
        )

        pose = self.receive_pose(goal_handle.request.pour_pose)
        

        # call the move_to_pose function
        goal_handle_result, action_result = self.move_to_pose(pose, isConstrained)
        self.get_logger().info(f"Pour pose result: ({goal_handle_result}, {action_result})")
        # self.get_logger().info(f"Pour pose: {pose}")
        if not action_result.result.success:
            self.get_logger().error("Failed to reach pour pose")
            return False, action_result.result.success

        # Now we can only rotate the joint 6 to pour
        # Rotate joint 6 to perform the pour motion

        # pour_angle = 3.14159
        pour_angle = 0.0

        # get the current pose
        current_pose = self.get_joint_positions()
        if not current_pose:
            self.get_logger().error("Failed to get current pose")
            
            return True, action_result.result.success

        # Rotate joint 6
        current_pose["joint6"] = pour_angle
        self.get_logger().info(f"Rotating joint 6 to {pour_angle} radians")
        # Move to the new joint position
        future = self._move_to_pose_action_client.send_goal_async(
            MoveToPose.Goal(
                pose=pose,
                velocity=POUR_VELOCITY, 
                acceleration=POUR_ACCELERATION,
                planner_id=PICK_PLANNER,
                apply_constraint=isConstrained,
                target_link=GRASP_LINK_FRAME,
            )
        )
        self.wait_for_future(future)
        action_result = future.result()
        if action_result is None:
            self.get_logger().error("Failed to get action result")
            return False, action_result.result.success
        
        if not action_result.result.success:
            self.get_logger().error("Failed to move to pour pose")
            return False, action_result.result.success
        
        self.get_logger().info("Moved to pour pose successfully")

        # Move to the updated pose
        # pour_motion_result = self.move_to_pose(pose, isConstrained)
        # if not pour_motion_result.result.success:
        #     self.get_logger().error("Failed to perform pour motion")
        #     result = False
        #     return False, result

        self.get_logger().info("Pour motion completed successfully")
        
        return False, action_result.result.success

        # isConstrained = (
        #     False  # THE MOTION PLANNER IS CONSTRAINED ------------------------
        # )
        # """Perform the pour operation."""
        # self.get_logger().info(
        #     f"Trying to pour object: {goal_handle.request.object_name}"
        # )
        # # Obtain the desired pose
        # bowl_position = goal_handle.request.bowl_position.point
        # abs_object_height = abs(
        #     goal_handle.request.object_top_height
        #     - goal_handle.request.object_centroid_height
        # )
        # abs_bowl_height = abs(
        #     goal_handle.request.bowl_top_height
        #     - goal_handle.request.bowl_position.point.z
        # )

        # pour_pose = Pose()
        # # Set position
        # pour_pose.pose.position.z += bowl_position.point.z + abs_bowl_height + 0.03
        # pour_pose.pose.position.x += bowl_position.x
        # pour_pose.pose.position.y += bowl_position.y + (abs_object_height) * (
        #     np.sin(45)
        # )
        # # Set orientation
        # pour_pose.pose.orientation.w = 1.0
        # pour_pose.pose.orientation.x = 0.0
        # pour_pose.pose.orientation.y = 0.0
        # pour_pose.pose.orientation.z = 0.0

        # # Set Frame ID
        # # pour_pose.header.frame_id = goal_handle.request.pour_pose.header.frame_id
        # self.get_logger().info(f"Pour pose: {pour_pose}")

        # # Move to pour pose
        # pour_pose_handler, pour_pose_result = self.move_to_pose(
        #     pour_pose, isConstrained
        # )
        # if not pour_pose_result.result.success:
        #     self.get_logger().error("Failed to reach pour pose")
        #     result = False
        #     return False, result
        # self.get_logger().info("Pour pose reached")

        # # Set the new orientation without constraint
        # pour_pose.pose.orientation.w = 1.0
        # pour_pose.pose.orientation.x = 0.0
        # pour_pose.pose.orientation.y = 0.0
        # pour_pose.pose.orientation.z = 0.0

        # self.get_logger().info(f"Pour final orientation: {pour_pose.orientation}")

        # pour_pose_handler, pour_pose_result = self.move_to_pose(
        #     pour_pose, isConstrained
        # )
        # if not pour_pose_result.result.success:
        #     self.get_logger().error("Failed to reach pour final orientation")
        #     result = False
        #     return False, result
        # self.get_logger().info("Pour final orientation reached")

        # return True

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

    def get_joint_positions(self, timeout=1) -> dict:
        start_time = time.time()
        while self.joint_states is None:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for joint states")
                return {}
            rclpy.spin_once(self, timeout_sec=0.1)
        return dict(zip(self.joint_states.name, self.joint_states.position))


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PourMotionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

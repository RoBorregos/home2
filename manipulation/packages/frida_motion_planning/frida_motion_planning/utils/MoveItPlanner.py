import time
from concurrent.futures import Future
from typing import List, Union

import rclpy
from controller_manager_msgs.srv import SwitchController
from frida_motion_planning.utils.Planner import Planner
from frida_pymoveit2.robots import xarm6
from geometry_msgs.msg import PoseStamped
from pymoveit2 import GripperInterface, MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from xarm_msgs.srv import SetInt16

from frida_constants.manipulation_constants import (
    MOVEIT_MODE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
)

PYMOVEIT_FUTURE_TIMEOUT = 3


# A class to handle planning with MoveIt
# Abstracts from pymoveit2, in case we want to change the library in the future
class MoveItPlanner(Planner):
    def __init__(
        self,
        node: Node,
        callback_group: ReentrantCallbackGroup,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
        planner_id: str = "RRTConnect",
    ):
        # Create callback group
        self.node = node
        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=xarm6.joint_names(),
            base_link_name=xarm6.base_link_name(),
            end_effector_name=xarm6.end_effector_name(),
            group_name=xarm6.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.joint_states_topic = "/joint_states"
        # create listener
        self.joint_states_sub = self.node.create_subscription(
            JointState, self.joint_states_topic, self.joint_states_callback, 10
        )

        """ 
        For some reason, controller may stop working when using moveit2
        This ensures the controller is active before sending a new plan
        ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController {"activate_controllers : [xarm6_traj_controller]}"
        
        """

        self.mode_enabled = True

        self.joint_states = None
        # Set initial parameters
        self.moveit2.max_velocity = max_velocity
        self.moveit2.max_acceleration = max_acceleration
        self.moveit2.planner_id = planner_id

    def set_velocity(self, velocity: float) -> None:
        self.moveit2.max_velocity = velocity

    def set_acceleration(self, acceleration: float) -> None:
        self.moveit2.max_acceleration = acceleration

    def set_planner(self, planner_id: str) -> None:
        self.moveit2.planner_id = planner_id

    def set_planning_time(self, planning_time: float) -> None:
        self.moveit2.allowed_planning_time = planning_time

    def set_planning_attempts(self, planning_attempts: int) -> None:
        self.moveit2.num_planning_attempts = planning_attempts
        
    def plan_joint_goal(self, joint_positions: List[float], joint_names: List[str]):
        self.node.get_logger().info("Generating a plan for a joint goal...")

        # Calls the internal function that uses pymoveit2 to get the plan
        trajectory_plan = self._plan_joint_goal(joint_positions, joint_names)

        # Checks if a valid plan was found (if not None)
        if trajectory_plan:
            self.node.get_logger().info("Plan for joint goal generated successfully.")
            # Returns a tuple: (success, trajectory_plan)
            return True, trajectory_plan
        else:
            self.node.get_logger().warn("MoveIt failed to generate a plan for the joint goal.")
            # Returns a tuple consistent also in case of failure
            return False, None
        
    def execute_plan(self, trajectory, wait: bool = True):
        """
        Takes a pre-planned trajectory and executes it.
        Monitors the execution and returns True if completed successfully.
        """
        self.node.get_logger().info("Executing the provided trajectory...")

        # --- 1. Send the Execution Command ---
        # This call is asynchronous, it does not block. It simply tells MoveIt to start.
        self.moveit2.execute(trajectory)

        # --- 2. Get the "Future" to Monitor Progress ---
        # A "future" is like a "receipt" or a "promise" that allows us to check
        # the status of the execution later.
        future = None
        start_time = time.time()
        while future is None and time.time() - start_time < PYMOVEIT_FUTURE_TIMEOUT:
            future = self.moveit2.get_execution_future()

        # If we couldn't get the 'future' in time, something went wrong.
        if future is None:
            self.node.get_logger().error(
                "Failed to get execution future, timeout occurred. Check if controllers are running."
            )
            return False

        # --- 3. Wait for Execution to Complete (if requested) ---
        if wait:
            self.node.get_logger().info("Waiting for execution to complete...")
            # This loop blocks the code until the action ends (success, failure, or cancellation).
            while not future.done():
                pass  # Active wait. In a real node, this would be handled more elegantly.

            # --- 4. Check the Final Result ---
            # Once 'future.done()' is True, we can see the result.
            # Status code '4' in pymoveit2 means SUCCESS.
            execution_success = future.result().status == 4
            if execution_success:
                self.node.get_logger().info("Execution completed successfully.")
            else:
                self.node.get_logger().error(f"Execution failed with status code: {future.result().status}")
            
            return execution_success

        # If not requested to wait, simply return True to indicate that execution started.
        return True
        
    # Past implementation, now split into plan and execute (using joint trajectory)
    # def plan_joint_goal(
    #     self,
    #     joint_positions: List[float],
    #     joint_names: List[str],
    #     wait: bool = True,
    #     set_mode: bool = True,
    # ) -> Union[bool, Future]:
    #     if set_mode:
    #         self.set_mode(MOVEIT_MODE)
    #     if joint_names is None or len(joint_names) == 0:
    #         joint_names = xarm6.joint_names()
    #     trajectory = self._plan_joint_goal(joint_positions, joint_names)
    #     if not trajectory:
    #         self.node.get_logger().error(
    #             "Failed to plan joint goal, \
    #                     probably due to timeout, check if MoveIt is running"
    #         )
    #         return False
    #     self.moveit2.execute(trajectory)
    #     future = None
    #     start_time = time.time()
    #     while future is None and time.time() - start_time < PYMOVEIT_FUTURE_TIMEOUT:
    #         future = self.moveit2.get_execution_future()
    #     if future is None:
    #         self.node.get_logger().error(
    #             "Failed to get execution future, \
    #                     probably due to timeout, check if MoveIt is running"
    #         )
    #         return False
    #     if wait:
    #         while not future.done():
    #             pass
    #         return future.result().status == 4
    #     return True
    
    def plan_pose_goal(
        self,
        pose: PoseStamped,
        target_link: str = xarm6.end_effector_name(),
        cartesian: bool = False,
        tolerance_position: float = 0.015,
        tolerance_orientation: float = 0.02,):
        
        self.node.get_logger().info("Generating a plan for a pose goal...")

        # Calls the internal function that uses pymoveit2 to get the plan
        trajectory_plan = self._plan(
            pose=pose,
            cartesian=cartesian,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )

        # Check if a valid plan was found (if not None)
        if trajectory_plan:
            self.node.get_logger().info("Plan for pose goal generated successfully.")
            # Return a tuple: (success, trajectory_plan)
            return True, trajectory_plan
        else:
            self.node.get_logger().warn("MoveIt failed to generate a plan for the pose goal.")
            # Return a consistent tuple also in case of failure
            return False, None

    # Past implementation, now split into plan and execute (using joint trajectory)
    # def plan_pose_goal(
    #     self,
    #     pose: PoseStamped,
    #     target_link: str = xarm6.end_effector_name(),
    #     cartesian: bool = False,
    #     tolerance_position: float = 0.015,
    #     tolerance_orientation: float = 0.02,
    #     wait: bool = True,
    #     set_mode: bool = True,
    # ) -> Union[bool, Future]:
    #     if set_mode:
    #         self.set_mode(MOVEIT_MODE)
    #     self.node.get_logger().info("Planning pose goal")
    #     trajectory = self._plan(
    #         pose=pose,
    #         cartesian=cartesian,
    #         target_link=target_link,
    #         tolerance_position=tolerance_position,
    #         tolerance_orientation=tolerance_orientation,
    #     )
    #     if not trajectory:
    #         return False
    #     self.node.get_logger().info("Executing trajectory")
    #     self.moveit2.execute(trajectory)
    #     future = None
    #     start_time = time.time()
    #     while future is None and time.time() - start_time < PYMOVEIT_FUTURE_TIMEOUT:
    #         future = self.moveit2.get_execution_future()
    #     if future is None:
    #         self.node.get_logger().error(
    #             "Failed to get execution future, \
    #                     probably due to timeout, check if MoveIt is running"
    #         )
    #         return False
    #     if wait:
    #         while not future.done():
    #             pass
    #         self.node.get_logger().info(
    #             "Execution done witth status: " + str(future.result().status)
    #         )
    #         return future.result().status == 4  # 4 is the status for success

    #     return self.moveit2.get_execution_future()

    def _plan(
        self,
        pose: PoseStamped,
        cartesian: bool = False,
        target_link: str = xarm6.end_effector_name(),
        tolerance_position: float = 0.01,
        tolerance_orientation: float = 0.05,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
        cartesian_max_step: float = 0.05,
        cartesian_fraction_threshold: float = 0.8,
    ) -> Union[bool, Future]:
        return self.moveit2.plan(
            position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            target_link=target_link,
            frame_id=pose.header.frame_id,
            cartesian=cartesian,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
            weight_position=weight_position,
            weight_orientation=weight_orientation,
            max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )

    def _plan_joint_goal(
        self,
        joint_positions: List[float],
        joint_names: List[str],
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        return self.moveit2.plan(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance_joint_position=tolerance,
            weight_joint_position=weight,
        )

    def get_current_operation_state(self) -> MoveIt2State:
        return self.moveit2.query_state()

    def get_joint_positions(self, timeout=1) -> dict[float]:
        # Get current joint positions
        # wait for message to be received
        start_time = time.time()
        while self.joint_states is None:
            if time.time() - start_time > timeout:
                return {}  # Return empty dict if timeout occurs
            rclpy.spin_once(self.node)
        return dict(zip(self.joint_states.name, self.joint_states.position))

    def joint_states_callback(self, msg: JointState):
        relevant_indices = []
        for i, joint_name in enumerate(msg.name):
            if joint_name in xarm6.joint_names():
                relevant_indices.append(i)
        # Filtra solo los joints del brazo
        if len(relevant_indices) == 0:
            return
        self.joint_states = JointState()
        self.joint_states.name = [msg.name[i] for i in relevant_indices]
        self.joint_states.position = [msg.position[i] for i in relevant_indices]

    def get_fk(self, links: List[str]) -> List[PoseStamped]:
        # Get forward kinematics for current joint state and specified links
        if self.moveit2.joint_state is None:
            return []
        return self.moveit2.compute_fk(fk_link_names=links)

    def compute_fk(self, joint_positions: List[float]) -> PoseStamped:
        # Compute forward kinematics for specified joint positions
        return self.moveit2.compute_fk(joint_state=joint_positions)

    def set_joint_constraints(
        self, joint_positions: List[float], tolerance: float = 0.1
    ) -> None:
        self.moveit2.set_joint_goal(
            joint_positions=joint_positions, tolerance=tolerance
        )

    def set_position_constraints(
        self, position: List[float], tolerance: float = 0.1
    ) -> None:
        self.moveit2.set_position_goal(position=position, tolerance=tolerance)

    def set_orientation_constraints(self, goal_handle) -> None:
        self.moveit2.set_path_orientation_constraint(
            quat_xyzw=goal_handle.constraint.orientation,
            frame_id=goal_handle.constraint.frame_id,
            target_link=goal_handle.constraint.target_link,
            tolerance=goal_handle.constraint.tolerance_orientation,
            weight=goal_handle.constraint.weight,
            parameterization=goal_handle.constraint.parameterization,  # 0: Euler Angles, 1: Rotation Vector
        )
        # self.moveit2.set_orientation_goal(quat_xyzw=quat_xyzw, tolerance=tolerance)
        # self.moveit2.set_path_orientation_constraint(quat_xyzw=quat_xyzw, tolerance=tolerance, parameterization=parameterization)

    def delete_all_constraints(self) -> None:
        self.moveit2.clear_goal_constraints()
        self.moveit2.clear_path_constraints()

    def add_collision_box(self, id: str, size: List[float], pose: PoseStamped) -> None:
        self.moveit2.add_collision_box(
            id=id,
            size=tuple(size),
            position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            frame_id=pose.header.frame_id,
        )

    def add_collision_cylinder(
        self, id: str, height: float, radius: float, pose: PoseStamped
    ) -> None:
        self.moveit2.add_collision_cylinder(
            id=id,
            height=height,
            radius=radius,
            position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            frame_id=pose.header.frame_id,
        )

    def add_collision_sphere(self, id: str, radius: float, pose: PoseStamped) -> None:
        self.moveit2.add_collision_sphere(
            id=id,
            radius=radius,
            position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            frame_id=pose.header.frame_id,
        )

    def add_collision_mesh(
        self,
        id: str,
        filepath: str,
        pose: PoseStamped,
        scale: float = 1.0,
    ) -> None:
        self.moveit2.add_collision_mesh(
            id=id,
            filepath=filepath,
            position=[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            frame_id=pose.header.frame_id,
            scale=scale,
        )

        """self,
        id: str,
        link_name: Optional[str] = None,
        touch_links: List[str] = [],
        weight: float = 0.0,"""

    def attach_collision_object(
        self,
        id: str,
        link_name: str,
        touch_links: List[str] = [],
        weight: float = 0.0,
    ) -> bool:
        self.moveit2.attach_collision_object(
            id=id,
            link_name=link_name,
            touch_links=touch_links,
            weight=weight,
        )
        return True

    def detach_collision_object(self, id: str) -> bool:
        self.moveit2.detach_collision_object(id)
        return True

    def update_planning_scene(self) -> None:
        self.moveit2.update_planning_scene()

    def get_planning_scene(self) -> None:
        self.update_planning_scene()  # check the pymoveit2 library
        return (
            self.moveit2.planning_scene
        )  # Return the current planning scene after the update

    def remove_collision_object(self, id: str) -> None:
        self.moveit2.remove_collision_object(id)

    def remove_all_collision_objects(self, include_attached: bool = False) -> None:
        if include_attached:
            self.moveit2.detach_all_collision_objects()
        time.sleep(1e-03)
        planning_scene = self.get_planning_scene()
        for collision_object in planning_scene.world.collision_objects:
            print("Found collision object: ", collision_object.id)
            time.sleep(1e-09)
            self.remove_collision_object(collision_object.id)

    def cancel_execution(self) -> None:
        self.moveit2.cancel_execution()

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.executor_thread.join()

    def open_gripper(self, wait: bool = True) -> Union[bool, Future]:
        # Create gripper interface
        gripper = GripperInterface(
            node=self.moveit2._node,
            gripper_joint_names=xarm6.gripper_joint_names(),
            open_gripper_joint_positions=xarm6.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=xarm6.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=xarm6.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )
        gripper.open()
        if wait:
            gripper.wait_until_executed()
            return True
        return gripper.get_execution_future()

    def close_gripper(self, wait: bool = True) -> Union[bool, Future]:
        # Create gripper interface
        gripper = GripperInterface(
            node=self.moveit2._node,
            gripper_joint_names=xarm6.gripper_joint_names(),
            open_gripper_joint_positions=xarm6.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=xarm6.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=xarm6.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )
        gripper.close()
        if wait:
            gripper.wait_until_executed()
            return True
        return gripper.get_execution_future()
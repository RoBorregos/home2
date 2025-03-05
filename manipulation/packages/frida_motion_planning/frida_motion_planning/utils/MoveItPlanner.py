from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State, GripperInterface
from frida_pymoveit2.robots import xarm6
from typing import List, Union
from concurrent.futures import Future
from pick_and_place.utils.Planner import Planner
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import rclpy


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

    def plan_joint_goal(
        self, joint_positions: List[float], joint_names: List[str], wait: bool = True
    ) -> Union[bool, Future]:
        if joint_names is None or len(joint_names) == 0:
            joint_names = xarm6.joint_names()
        trajectory = self._plan_joint_goal(joint_positions, joint_names)
        if not trajectory:
            return False
        self.moveit2.execute(trajectory)
        future = None
        while future is None:
            future = self.moveit2.get_execution_future()
        if wait:
            while not future.done():
                pass
            return future.result().status == 4
        return future

    def plan_pose_goal(
        self, pose: PoseStamped, cartesian: bool = False, wait: bool = True
    ) -> Union[bool, Future]:
        print("calling plan")
        trajectory = self._plan(pose, cartesian)
        if not trajectory:
            return False
        print("calling execute")
        self.moveit2.execute(trajectory)
        future = None
        while future is None:
            future = self.moveit2.get_execution_future()
        if wait:
            while not future.done():
                pass
            print("future result", future.result())
            print("future result status", future.result().status)
            return future.result().status == 4  # 4 is the status for success

        return self.moveit2.get_execution_future()

    def _plan(
        self,
        pose: PoseStamped,
        cartesian: bool = False,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
        cartesian_max_step: float = 0.01,
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
        self.joint_states = msg

    def get_fk(self, links: List[str]) -> List[PoseStamped]:
        # Get forward kinematics for current joint state and specified links
        if self.moveit2.joint_state is None:
            return []
        return self.moveit2.compute_fk(fk_link_names=links)

    def compute_fk(self, joint_positions: List[float]) -> PoseStamped:
        # Compute forward kinematics for specified joint positions
        return self.moveit2.compute_fk(joint_state=joint_positions)

    def set_joint_constraints(
        self, joint_positions: List[float], tolerance: float = 0.001
    ) -> None:
        self.moveit2.set_joint_goal(
            joint_positions=joint_positions, tolerance=tolerance
        )

    def set_position_constraints(
        self, position: List[float], tolerance: float = 0.001
    ) -> None:
        self.moveit2.set_position_goal(position=position, tolerance=tolerance)

    def set_orientation_constraints(
        self, quat_xyzw: List[float], tolerance: float = 0.001
    ) -> None:
        self.moveit2.set_orientation_goal(quat_xyzw=quat_xyzw, tolerance=tolerance)

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

    def remove_collision_object(self, id: str) -> None:
        self.moveit2.remove_collision_object(id)

    def clear_all_collision_objects(self) -> None:
        self.moveit2.clear_all_collision_objects()

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

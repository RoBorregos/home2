from abc import ABC, abstractmethod
from typing import List, Union, Any
from rclpy.node import Node
from concurrent.futures import Future
from geometry_msgs.msg import PoseStamped


class Planner(ABC):
    """Abstract base class for motion planners"""

    @abstractmethod
    def __init__(
        self,
        node: Node,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
        planner_id: str = "",
    ):
        """Initialize the planner"""
        pass

    @abstractmethod
    def set_velocity(self, velocity: float) -> None:
        """Set the maximum velocity for planning"""
        pass

    @abstractmethod
    def set_acceleration(self, acceleration: float) -> None:
        """Set the maximum acceleration for planning"""
        pass

    @abstractmethod
    def set_planner(self, planner_id: str) -> None:
        """Set the planner algorithm"""
        pass

    @abstractmethod
    def plan_joint_goal(
        self, joint_positions: List[float], wait: bool = True
    ) -> Union[bool, Future]:
        """Plan and execute a trajectory to reach joint positions"""
        pass

    @abstractmethod
    def plan_pose_goal(
        self,
        pose: PoseStamped,
        cartesian: bool = False,
        wait: bool = True,
    ) -> Union[bool, Future]:
        """Plan and execute a trajectory to reach a pose goal"""
        pass

    def get_joint_positions(self) -> dict[float]:
        """Get the current joint positions"""
        pass

    def get_joint_velocities(self) -> dict[float]:
        """Get the current joint velocities"""
        pass

    def get_joint_efforts(self) -> dict[float]:
        """Get the current joint efforts"""
        pass

    def get_joint_states(self) -> dict[dict[float]]:
        """Get the current joint states (positions, velocities, efforts)"""
        pass

    @abstractmethod
    def get_current_operation_state(self) -> Any:
        """Get the current robot state -> Planning, Moving, Idle..."""
        pass

    @abstractmethod
    def get_fk(self, links: List[str]) -> List[PoseStamped]:
        """Get the forward kinematics for a list of links based on current joint positions"""
        pass

    @abstractmethod
    def compute_fk(self, joint_positions: List[float]) -> PoseStamped:
        """Compute the forward kinematics for a robot state (set of joint positions)"""
        pass

    @abstractmethod
    def set_joint_constraints(
        self, joint_positions: List[float], tolerance: float = 0.001
    ) -> None:
        """Set joint constraints during planning"""
        pass

    @abstractmethod
    def set_position_constraints(
        self, position: List[float], tolerance: float = 0.001
    ) -> None:
        """Set position constraints during planning"""
        pass

    @abstractmethod
    def set_orientation_constraints(
        self, quat_xyzw: List[float], tolerance: float = 0.001
    ) -> None:
        """Set orientation constraints during planning"""
        pass

    @abstractmethod
    def delete_all_constraints(self) -> None:
        """Clear all path and goal constraints"""
        pass

    @abstractmethod
    def add_collision_box(
        self, id: str, size: List[float], position: List[float], quat_xyzw: List[float]
    ) -> None:
        """Add box collision object to planning scene"""
        pass

    @abstractmethod
    def add_collision_cylinder(
        self,
        id: str,
        height: float,
        radius: float,
        position: List[float],
        quat_xyzw: List[float],
    ) -> None:
        """Add cylinder collision object to planning scene"""
        pass

    @abstractmethod
    def add_collision_mesh(
        self,
        id: str,
        filepath: str,
        position: List[float],
        quat_xyzw: List[float],
        scale: float = 1.0,
    ) -> None:
        """Add mesh collision object to planning scene"""
        pass

    @abstractmethod
    def remove_collision_object(self, id: str) -> None:
        """Remove collision object from planning scene"""
        pass

    @abstractmethod
    def clear_all_collision_objects(self) -> None:
        """Remove all collision objects from planning scene"""
        pass

    @abstractmethod
    def cancel_execution(self) -> None:
        """Cancel the current execution"""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Shutdown the planner"""
        pass

    @abstractmethod
    def open_gripper(self, wait: bool = True) -> Union[bool, Future]:
        """Open the gripper"""
        pass

    @abstractmethod
    def close_gripper(self, wait: bool = True) -> Union[bool, Future]:
        """Close the gripper"""
        pass

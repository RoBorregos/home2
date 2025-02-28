from abc import ABC, abstractmethod
from typing import List, Union, Any
from rclpy.node import Node
from concurrent.futures import Future


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
        position: List[float],
        quat_xyzw: List[float],
        cartesian: bool = False,
        wait: bool = True,
    ) -> Union[bool, Future]:
        """Plan and execute a trajectory to reach a pose goal"""
        pass

    @abstractmethod
    def get_current_state(self) -> Any:
        """Get the current robot state"""
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

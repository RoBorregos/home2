from abc import ABC, abstractmethod
from typing import Tuple
from rclpy.node import Node


class Servo(ABC):
    """Abstract base class for motion planners doing servoing in Cartesian space"""

    @abstractmethod
    def __init__(
        self,
        node: Node,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
    ):
        """Initialize the planner"""
        pass

    @abstractmethod
    def enable_servo(self) -> None:
        """Enable servoing"""
        pass

    @abstractmethod
    def disable_servo(self) -> None:
        """Disable servoing"""
        pass

    @abstractmethod
    def update_servo(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        pass

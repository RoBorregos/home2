"""Defines BBOX, Detection, and DetectorModel ABC — the shared types for all model implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class BBOX:
    x: float = 0
    y: float = 0
    w: float = 0
    h: float = 0
    x1: float = 0
    x2: float = 0
    y1: float = 0
    y2: float = 0


class Detection:
    def __init__(self, label: str, class_id: int, confidence: float):
        self.bbox_ = BBOX()
        self.label_: str = label
        self.class_id_: int = class_id
        self.confidence_: float = confidence
        self.point_stamped_ = None
        self.mask = None


class DetectorModel(ABC):
    def __init__(self, name: str):
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    @abstractmethod
    def load(self, config: dict): ...

    @abstractmethod
    def detect(self, image) -> list[Detection]: ...

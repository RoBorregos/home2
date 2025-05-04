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
    distance: float = 0
    pz: float = 0
    px: float = 0
    py: float = 0
    classname: str = ""


@dataclass
class ShelfDetection:
    level: int = 0
    x1: float = 0
    y1: float = 0
    x2: float = 0
    y2: float = 0

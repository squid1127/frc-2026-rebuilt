"""Represents state across the project"""

from enum import Enum, auto

class RobotState(Enum):
    DISABLED = auto()
    TELEOP = auto()
    AUTO = auto()
    PRACT = auto()
    TEST = auto()

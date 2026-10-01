"""Represents state across the project"""

from enum import Enum, auto


class RobotState(Enum):
    """Represents the state of the robot."""
    DISABLED = auto()
    TELEOP = auto()
    AUTO = auto()
    PRACTICE = auto()
    TEST = auto()

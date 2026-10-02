"""Top-level robot configuration dataclasses."""

from dataclasses import dataclass

from .subsystems.swerve.config import SwerveConfig


@dataclass(frozen=True, slots=True)
class IntakeConfig:
    """Configuration for the intake subsystem."""

    motor_id: int


@dataclass(frozen=True, slots=True)
class LauncherConfig:
    """Configuration for the launcher subsystem."""

    motor_id: int


@dataclass(frozen=True, slots=True)
class RobotConfig:
    """Configuration for all robot subsystems."""

    intake: IntakeConfig
    launcher: LauncherConfig
    swerve: SwerveConfig
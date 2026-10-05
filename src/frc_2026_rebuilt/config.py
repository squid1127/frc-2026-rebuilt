"""Top-level robot configuration dataclasses."""

from dataclasses import dataclass

from .subsystems.swerve.config import SwerveConfig


@dataclass(frozen=True, slots=True)
class IntakeConfig:
    """Configuration for the intake subsystem.
    
    Attributes:
        motor_id: The CAN ID of the intake motor
    """

    motor_id: int


@dataclass(frozen=True, slots=True)
class LauncherConfig:
    """Configuration for the launcher subsystem.
    
    Attributes:
        motor_id: The CAN ID of the launcher motor
    """

    motor_id: int


@dataclass(frozen=True, slots=True)
class RobotConfig:
    """Configuration for all robot subsystems.
    
    Attributes:
        intake: Configuration for the intake subsystem
        launcher: Configuration for the launcher subsystem
        swerve: Configuration for the swerve subsystem
    """

    intake: IntakeConfig
    launcher: LauncherConfig
    swerve: SwerveConfig
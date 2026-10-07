"""Top-level robot configuration dataclasses."""

from dataclasses import dataclass

from .subsystems.feeder import FeederConfig
from .subsystems.launcher import LauncherConfig
from .subsystems.swerve import SwerveConfig


@dataclass(frozen=True, slots=True)
class RobotConfig:
    """Configuration for all robot subsystems.

    Attributes:
        driver_controller: Index for the driver's controller
        operator_controller: Index for the operator's controller
        feeder: Configuration for the feeder subsystem
        launcher: Configuration for the launcher subsystem
        swerve: Configuration for the swerve subsystem
    """

    driver_controller: int
    operator_controller: int

    feeder: FeederConfig
    launcher: LauncherConfig
    swerve: SwerveConfig

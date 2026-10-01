"""Main robot class"""

import wpilib
import commands2
from phoenix6.canbus import CANBus as PhCANBus
from typing import Any, Callable

from .models.state import RobotState
from .subsystems.launcher import LauncherSubsystem
from .subsystems.intake import IntakeSubsystem
from .logger import get_logger

logger = get_logger(__name__)


class RobotContainer(commands2.TimedCommandRobot):
    """Main robot class"""

    def __init__(self):
        logger.info("Initializing...")

        super().__init__()

        self.controller = commands2.button.CommandXboxController(0)

        self.bus = PhCANBus()
        self.launcher = LauncherSubsystem(17, self.bus)
        self.intake = IntakeSubsystem(14)

        self.set_bindings()

    def set_bindings(self) -> None:
        """Sets the bindings for each subsystem"""
        self.controller.a().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.launcher.set(0.3),
                lambda: self.launcher.stop(),
                self.launcher
            )
        )
        self.controller.rightBumper().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.intake.set(0.3),
                lambda: self.intake.stop(),
                self.intake
            )
        )
        self.controller.rightTrigger().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.intake.set(-0.3),
                lambda: self.intake.stop(),
                self.intake
            )
        )
"""Joystick command for driving the swerve drivetrain."""

from typing import Protocol

from commands2 import Command

from ..logger import get_logger
from ..subsystems.swerve import DrivetrainSubsystem

logger = get_logger(__name__)


class Controller(Protocol):
    """Controller device that implements getLeftY, getLeftX, and getRightX"""

    def getLeftX(self) -> float: ...
    def getRightX(self) -> float: ...


class SwerveTuneYaw(Command):
    """Tune the yaw axis by adding offsets based on joystick input"""

    def __init__(
        self,
        drivetrain: DrivetrainSubsystem,
        controller: Controller,
        deadband: float,
        l_sensitivity: float,
        r_sensitivity: float,
    ):
        super().__init__()
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.controller: Controller = controller
        self.deadband = deadband
        self.l_sensitivity = l_sensitivity
        self.r_sensitivity = r_sensitivity
        self.addRequirements(drivetrain)

    def execute(self) -> None:

        l_value = self.controller.getLeftX()
        if abs(l_value) >= self.deadband:
            self.drivetrain.offset_yaw(l_value * self.l_sensitivity)

        r_value = self.controller.getRightX()
        if abs(r_value) >= self.deadband:
            self.drivetrain.offset_yaw(r_value * self.r_sensitivity)

    def initialize(self):
        super().initialize()

        self.drivetrain.set(1, align_only=True)

    def end(self, interrupted: bool):
        super().end(interrupted)

        self.drivetrain.stop()

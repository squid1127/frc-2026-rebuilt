"""Joystick command for driving the swerve drivetrain."""

from typing import Protocol

from commands2 import Command

from ..subsystems.swerve import DrivetrainSubsystem
from ..logger import get_logger

logger = get_logger(__name__)


class Controller(Protocol):
    """Controller device that implements getLeftY, getLeftX, and getRightX"""

    def getLeftY(self) -> float: ...
    def getLeftX(self) -> float: ...
    def getRightX(self) -> float: ...


class SwerveTuneYaw(Command):
    """Tune the yaw axis by adding offsets"""

    def __init__(
        self,
        drivetrain: DrivetrainSubsystem,
        controller: Controller,
        deadband: float,
        sensitivity: float,
        *,
        field_relative: bool = True,
    ):
        super().__init__()
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.controller = controller
        self.field_relative = field_relative
        self.deadband = deadband
        self.sensitivity = sensitivity
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        drive_y = self.controller.getLeftY()
        drive_x = self.controller.getLeftX()
        if abs(drive_y) >= self.deadband or abs(drive_x) >= self.deadband:
            self.drivetrain.set(-drive_y, -drive_x, 0, field_relative=True)
        else:
            self.drivetrain.set(1, field_relative=True, align_only=True)

        value = self.controller.getRightX()
        if abs(value) >= self.deadband:
            self.drivetrain.offset_yaw(value * self.sensitivity)

    def end(self, interrupted: bool):
        super().end(interrupted)

        self.drivetrain.stop()

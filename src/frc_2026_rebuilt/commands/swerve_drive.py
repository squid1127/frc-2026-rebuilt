"""Joystick command for driving the swerve drivetrain."""

from collections.abc import Callable
from typing import Protocol

from commands2 import Command

from ..subsystems.swerve import DrivetrainSubsystem


class Controller(Protocol):
    """Controller device that implements getLeftY, getLeftX, and getRightX"""

    def getLeftY(self) -> float: ...
    def getLeftX(self) -> float: ...
    def getRightX(self) -> float: ...
class SwerveDriveCommand(Command):
    """Drive from Xbox sticks, optionally relative to the field or robot."""

    def __init__(
        self,
        drivetrain: DrivetrainSubsystem,
        controller: Controller,
        *,
        field_relative: bool = True,
        field_relative_inverted: Callable[[], bool] | None = None,
    ):
        super().__init__()
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.controller = controller
        self.field_relative = field_relative
        self.field_relative_inverted = field_relative_inverted
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        self.drivetrain.set(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            -self.controller.getRightX(),
            field_relative=self.field_relative,
            field_relative_inverted=(
                self.field_relative_inverted()
                if self.field_relative_inverted is not None
                else False
            ),
        )
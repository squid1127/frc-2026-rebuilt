"""Joystick command for driving the swerve drivetrain."""

from commands2 import Command

from ..subsystems.swerve.drivetrain import DrivetrainSubsystem


class SwerveDriveCommand(Command):
    """Drive from Xbox sticks, optionally relative to the field or robot."""

    def __init__(self, drivetrain, controller, *, field_relative: bool = True):
        super().__init__()
        self.drivetrain: DrivetrainSubsystem = drivetrain
        self.controller = controller
        self.field_relative = field_relative
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        self.drivetrain.set(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            -self.controller.getRightX(),
            field_relative=self.field_relative,
        )
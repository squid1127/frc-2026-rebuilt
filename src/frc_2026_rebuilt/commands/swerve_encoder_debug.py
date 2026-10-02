"""Command that logs the three swerve encoder positions per module."""

from commands2 import Command

from ..logger import get_logger
from ..subsystems.swerve.drivetrain import DrivetrainSubsystem

logger = get_logger(__name__)


class SwerveEncoderDebugCommand(Command):
    """Log drive, steer, and CANcoder positions once when scheduled."""

    def __init__(self, drivetrain: DrivetrainSubsystem):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        for module in self.drivetrain.modules:
            drive_position = module.drive_motor.get_position()
            steer_position = module.steer_motor.get_position()
            encoder_position = module.encoder.get_absolute_position()

            drive_position.refresh()
            steer_position.refresh()
            encoder_position.refresh()

            logger.info(
                "%s encoder positions (rotations): drive=%.3f steer=%.3f CANcoder=%.3f",
                module.name,
                drive_position.value,
                steer_position.value,
                encoder_position.value,
            )

    def isFinished(self) -> bool:
        return True
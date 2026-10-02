"""Main robot class"""

import commands2
from phoenix6.canbus import CANBus as PhCANBus

from .commands import SwerveDriveCommand, SwerveEncoderDebugCommand
from .config import RobotConfig
from .current_config import CurrentBotConfig
from .logger import get_logger
from .subsystems.intake import IntakeSubsystem
from .subsystems.launcher import LauncherSubsystem
from .subsystems.swerve import DrivetrainSubsystem

logger = get_logger(__name__)


class RobotContainer(commands2.TimedCommandRobot):
    """Main robot class"""

    CRAWL_SPEED = 0.15

    def __init__(self, config: RobotConfig = CurrentBotConfig.CONFIG):
        logger.info("Initializing...")

        super().__init__()
        self.config = config

        self.controller = commands2.button.CommandXboxController(0)

        self.bus = PhCANBus()
        self.launcher = LauncherSubsystem(config.launcher.motor_id, self.bus)
        self.intake = IntakeSubsystem(config.intake.motor_id)
        self.swerve = DrivetrainSubsystem(config.swerve, bus=self.bus)
        self.field_relative_drive = SwerveDriveCommand(
            self.swerve, self.controller, field_relative=True
        )
        self.robot_relative_drive = SwerveDriveCommand(
            self.swerve, self.controller, field_relative=False
        )
        self.encoder_debug_command = SwerveEncoderDebugCommand(self.swerve)

        self.set_bindings()

    def set_bindings(self) -> None:
        """Sets the bindings for each subsystem"""
        self.swerve.setDefaultCommand(self.field_relative_drive)
        self.controller.y().whileTrue(
            commands2.cmd.run(
                lambda: self.swerve.set(
                    self.CRAWL_SPEED,
                    field_relative=False,
                ),
                self.swerve,
            )
        )
        self.controller.leftTrigger().whileTrue(
            commands2.cmd.run(self.swerve.lock, self.swerve)
        )
        self.controller.leftBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: (
                    self.swerve.reset_yaw(),
                    logger.info("Reset swerve yaw"),
                ),
                self.swerve,
            )
        )


        # self.controller.x().onTrue(self.encoder_debug_command)
        self.controller.a().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.launcher.set(0.6),
                lambda: self.launcher.stop(),
                self.launcher,
            )
        )
        self.controller.rightBumper().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.intake.set(0.5), lambda: self.intake.stop(), self.intake
            )
        )
        self.controller.rightTrigger().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.intake.set(-0.5), lambda: self.intake.stop(), self.intake
            )
        )

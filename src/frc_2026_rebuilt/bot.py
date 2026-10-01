"""Main robot class"""

import commands2
from phoenix6.canbus import CANBus as PhCANBus

from .logger import get_logger
from .subsystems.intake import IntakeSubsystem
from .subsystems.launcher import LauncherSubsystem
from .subsystems.swerve import SwerveConfig, SwerveModuleConfig, DrivetrainSubsystem

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
        swerve_config = SwerveConfig(
            imu_usb=1,
            fr=SwerveModuleConfig(
                drive_motor_id=9, steer_motor_id=8, encoder_id=10, encoder_offset=0
            ),
            fl=SwerveModuleConfig(
                drive_motor_id=6, steer_motor_id=5, encoder_id=7, encoder_offset=0
            ),
            br=SwerveModuleConfig(
                drive_motor_id=3, steer_motor_id=2, encoder_id=4, encoder_offset=0
            ),
            bl=SwerveModuleConfig(
                drive_motor_id=6, steer_motor_id=5, encoder_id=7, encoder_offset=0
            ),
        )
        self.swerve = DrivetrainSubsystem(swerve_config, bus=self.bus)

        self.set_bindings()

    def set_bindings(self) -> None:
        """Sets the bindings for each subsystem"""
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

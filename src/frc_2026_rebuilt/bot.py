"""Main robot class"""

import commands2
from phoenix6.canbus import CANBus as PhCANBus

from .commands import SwerveDriveCommand, SwerveTuneYaw
from .config import RobotConfig
from .current_config import CurrentBotConfig
from .logger import get_logger
from .subsystems.feeder import FeederSubsystem
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
        self.last_control_state: None | tuple[bool, bool, bool] = None

        # Controllers
        self.driver_controller = commands2.button.CommandXboxController(
            config.driver_controller
        )
        self.operator_controller = commands2.button.CommandXboxController(
            config.operator_controller
        )
        if not self.driver_controller.isConnected():
            logger.error(
                "Driver controller is not connected (USB port %s)",
                config.driver_controller,
            )
        if not self.operator_controller.isConnected():
            logger.error(
                "Operator controller is not connected (USB port %s)",
                config.operator_controller,
            )
            
        # Subsystems
        self.bus = PhCANBus()
        self.launcher = LauncherSubsystem(config.launcher, self.bus)
        self.feeder = FeederSubsystem(config.feeder)
        self.swerve = DrivetrainSubsystem(config.swerve, bus=self.bus)

        # Commands
        self.field_relative_drive = SwerveDriveCommand(
            self.swerve, self.driver_controller, field_relative=True
        )
        self.tune_yaw = SwerveTuneYaw(self.swerve, self.driver_controller, 0.05, 10, 1.5)

        self.set_bindings()

        logger.info("Bot initialization complete...")

    def set_bindings(self) -> None:
        """Sets the controller bindings for each subsystem"""

        # Driver controller
        self.swerve.setDefaultCommand(self.field_relative_drive)
        self.driver_controller.leftBumper().whileTrue(self.tune_yaw)
        self.driver_controller.povUp().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.swerve.set(
                    1,
                    field_relative=False,
                    align_only=True,
                ),
                lambda: self.swerve.stop(),
                self.swerve,
            )
        )
        self.driver_controller.leftTrigger().whileTrue(
            commands2.cmd.run(self.swerve.lock, self.swerve)
        )

        # Operator controller
        self.operator_controller.a().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.launcher.launch(),
                self.launcher,
            )
        )
        self.operator_controller.b().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.launcher.stop(),
                self.launcher,
            )
        )
        self.operator_controller.y().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.feeder.set(0.5), self.feeder
            )
        )
        self.operator_controller.rightBumper().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.feeder.set(0.5), lambda: self.feeder.stop(), self.feeder
            )
        )
        self.operator_controller.rightTrigger().whileTrue(
            commands2.cmd.startEnd(
                lambda: self.feeder.set(-0.5), lambda: self.feeder.stop(), self.feeder
            )
        )

    def robotPeriodic(self) -> None:
        """Detect state transitions"""
        super().robotPeriodic()

        state = self.getControlState()
        if state != self.last_control_state:
            flags = []
            if state[0]:
                flags.append("ENABLED")
            if state[1]:
                flags.append("AUTO")
            if state[2]:
                flags.append("TEST")
            if not flags:
                flags.append("NONE")

            logger.info(f"State transition: {', '.join(flags)}")
            self.last_control_state = state

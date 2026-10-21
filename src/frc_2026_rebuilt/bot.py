"""Main robot class"""

import commands2
from phoenix6.canbus import CANBus as PhCANBus

from .commands import SwerveDriveCommand, SwerveTuneYaw
from .config import RobotConfig
from .current_config import CurrentBotConfig
from .logger import get_logger
from .network import NTBool
from .subsystems.feeder import FeederSubsystem
from .subsystems.launcher import LauncherSubsystem
from .subsystems.swerve import DrivetrainSubsystem

logger = get_logger(__name__)


class RobotContainer(commands2.TimedCommandRobot):
    """Main robot class"""

    def __init__(self, config: RobotConfig = CurrentBotConfig.CONFIG):
        logger.info("Initializing...")

        super().__init__()
        self.config = config
        self.last_control_state: None | tuple[bool, bool, bool] = None
        self.nt_competition_mode = NTBool("Bot/Competition Mode", True)

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
        self.tune_yaw = SwerveTuneYaw(
            self.swerve, self.driver_controller, 0.075, 10, 1.5
        )

        self.set_bindings()

        logger.info("Bot initialization complete...")

    def set_bindings(self) -> None:
        """Sets the controller bindings for each subsystem"""

        # Driver controller
        self.swerve.setDefaultCommand(self.field_relative_drive)
        self.driver_controller.leftBumper().whileTrue(self.tune_yaw)
        self.driver_controller.leftTrigger().whileTrue(
            commands2.cmd.run(self.swerve.lock, self.swerve)
        )
        self.driver_controller.x().and_(
            lambda: not self.nt_competition_mode.value
        ).whileTrue(
            commands2.cmd.runOnce(
                lambda: self.swerve.reset_pose(),
                self.swerve,
            )
        )
        self.driver_controller.povUp().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.swerve.rotate_to(0),
                self.swerve,
            )
        )
        self.driver_controller.povDown().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.swerve.rotate_to(180),
                self.swerve,
            )
        )
        self.driver_controller.povLeft().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.swerve.rotate_to(90),
                self.swerve,
            )
        )
        self.driver_controller.povRight().whileTrue(
            commands2.cmd.runOnce(
                lambda: self.swerve.rotate_to(270),
                self.swerve,
            )
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
        self.operator_controller.x().whileTrue(
            commands2.cmd.startEnd(
                self.launcher.unstick,
                self.launcher.stop,
                self.launcher,
            )
        )
        self.operator_controller.y().whileTrue(
            commands2.cmd.runOnce(self.feeder.forward, self.feeder)
        )
        self.operator_controller.rightBumper().whileTrue(
            commands2.cmd.startEnd(
                self.feeder.forward, self.feeder.stop, self.feeder
            )
        )
        self.operator_controller.rightTrigger().whileTrue(
            commands2.cmd.startEnd(
                self.feeder.reverse, self.feeder.stop, self.feeder
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

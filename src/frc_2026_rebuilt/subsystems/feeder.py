"""Feeder subsystem"""

from dataclasses import dataclass
from logging import getLogger

import rev
from commands2 import Subsystem

logger = getLogger(__name__)


@dataclass(frozen=True, slots=True)
class FeederConfig:
    """Configuration for the feeder subsystem.

    Attributes:
        motor_id: The CAN ID of the intake motor
        target_velocity_rps: Target speed at the feeder shaft, in RPS
        gear_ratio: Motor rotations per feeder-shaft rotation
    """

    motor_id: int
    target_velocity_rps: float
    gear_ratio: float

    def __post_init__(self) -> None:
        if self.target_velocity_rps <= 0:
            raise ValueError("Feeder target velocity must be positive")
        if self.gear_ratio <= 0:
            raise ValueError("Feeder gear ratio must be positive")


class FeederSubsystem(Subsystem):
    """Feeder subsystem"""

    def __init__(self, config: FeederConfig):
        super().__init__()

        self.config = config

        self.motor = rev.SparkMax(
            self.config.motor_id, rev.SparkMax.MotorType.kBrushless
        )
        self.velocity_controller = self.motor.getClosedLoopController()
        status = self.set_motor_options()
        if status != rev.REVLibError.kOk:
            logger.error(
                "Feeder motor is not responding (CAN ID %s): configuration failed with %s",
                config.motor_id,
                status,
            )

        logger.info(f"Feeder system initialized (CAN ID: {self.config.motor_id})")

    def set_motor_options(self) -> rev.REVLibError:
        """Configure motor options"""
        config = rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        config.smartCurrentLimit(11)
        config.openLoopRampRate(0.1)
        config.closedLoop.pidf(0.0001, 0, 0, 1/500)

        return self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def stop(self) -> None:
        """Attempt to stop the motor"""
        self.motor.stopMotor()

    def set(self, feeder_rps: float):
        """Run the feeder at a specific velocity in RPS"""
        self._set_velocity(feeder_rps)

    def forward(self) -> None:
        """Run the feeder toward the launcher at its configured velocity."""
        self._set_velocity(self.config.target_velocity_rps)

    def reverse(self) -> None:
        """Run the feeder away from the launcher at its configured velocity."""
        self._set_velocity(-self.config.target_velocity_rps)

    def _set_velocity(self, feeder_rps: float) -> None:
        motor_rpm = feeder_rps * self.config.gear_ratio * 60
        status = self.velocity_controller.setSetpoint(
            motor_rpm,
            rev.SparkLowLevel.ControlType.kVelocity,
        )
        if status != rev.REVLibError.kOk:
            logger.error(
                "Failed to set feeder velocity (CAN ID %s): %s",
                self.config.motor_id,
                status,
            )

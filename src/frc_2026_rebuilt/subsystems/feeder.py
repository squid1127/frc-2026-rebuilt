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
    """

    motor_id: int


class FeederSubsystem(Subsystem):
    """Feeder subsystem"""

    def __init__(self, config: FeederConfig):
        super().__init__()

        self.config = config

        self.motor = rev.SparkMax(
            self.config.motor_id, rev.SparkMax.MotorType.kBrushless
        )
        self.target = 0
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
        config.smartCurrentLimit(25)
        config.openLoopRampRate(0.1)

        return self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def stop(self) -> None:
        """Attempt to stop the motor"""
        self.motor.stopMotor()

    def set(self, value: float, deadband: float = 0):
        """Spin the motor with this strength"""
        if abs(value) > deadband:
            self.motor.set(value)
        else:
            self.motor.stopMotor()

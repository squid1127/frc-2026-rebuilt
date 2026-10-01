"""Intake subsystem"""

from logging import getLogger

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
import rev

logger = getLogger(__name__)


class IntakeSubsystem(Subsystem):
    """Intake subsystem"""

    def __init__(self, motor_id: int):
        super().__init__()

        self.motor_id = motor_id

        self.motor = rev.SparkMax(motor_id, rev.SparkMax.MotorType.kBrushless)
        self.target = 0
        self.set_motor_options()

        # if not self.motor.():
        #     raise RuntimeError("Motor is reportedly dead")

        logger.info(f"Intake system ready (IDs: {motor_id})")

    def set_motor_options(self):
        """Configure motor options"""
        config = rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        config.smartCurrentLimit(40)
        config.openLoopRampRate(0.1)

        self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def stop(self) -> None:
        """Attempt to stop the motor"""
        self.motor.stopMotor()

    def set(self, value: float):
        """Spin the motor with this strength"""
        if abs(value) > 0.01:
            self.motor.set(value)
        else:
            self.motor.stopMotor()

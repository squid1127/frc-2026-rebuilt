"""Launcher subsystem"""

from dataclasses import dataclass

import wpilib
from commands2 import Subsystem
from phoenix6.canbus import CANBus
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ..logger import get_logger

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class LauncherConfig:
    """Configuration for the launcher subsystem.

    Attributes:
        motor_id: The CAN ID of the launcher motor
        target_rev: The target velocity (in RPM) for the launcher motor
    """

    motor_id: int
    target_rev: float


class LauncherSubsystem(Subsystem):
    """Launcher subsystem"""

    def __init__(self, config: LauncherConfig, bus: CANBus):
        super().__init__()

        self.bus = bus
        self.config = config

        self.motor = TalonFX(config.motor_id, bus)
        self.target = config.target_rev
        self.set_motor_options()

        if not wpilib.RobotBase.isSimulation() and self.motor.is_connected and not self.motor.isAlive():
            logger.error(
                "Launcher motor is not responding (CAN ID %s)",
                config.motor_id,
            )

        logger.info(f"Launcher system initialized (CAN ID: {config.motor_id})")

    def set_motor_options(self):
        """Configure motor options"""
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output.peak_forward_duty_cycle = 1
        config.motor_output.peak_reverse_duty_cycle = 0
        config.feedback.sensor_to_mechanism_ratio = 1.0

        config.current_limits.stator_current_limit_enable = True
        config.current_limits.stator_current_limit = 20
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.supply_current_limit = 20

        config.slot0.k_s = 0.02
        config.slot0.k_v = 0.125
        config.slot0.k_p = 0.05
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0

        self.motor.configurator.apply(config)

    def stop(self) -> None:
        """Attempt to stop the motor"""
        self.motor.stopMotor()

    def set(self, velocity: float):
        """Spin the motor at the given velocity"""
        self.motor.set_control(VelocityVoltage(velocity))

    def launch(self):
        """Launch the motor at the given velocity"""
        self.set(self.target)

"""Intake subsystem"""

from logging import getLogger

from phoenix6.canbus import CANBus
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

logger = getLogger(__name__)


class IntakeSubsystem:
    """Intake subsystem"""

    def __init__(self, motor_id: int, bus: CANBus):
        self.bus = bus
        self.motor_id = motor_id

        self.motor = TalonFX(motor_id, bus)
        self.set_motor_options()
        logger.info(f"Intake system ready (IDs: {motor_id})")

    def set_motor_options(self):
        """Configure motor options"""
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        config.current_limits.stator_current_limit_enable = True
        config.current_limits.stator_current_limit = 60
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.supply_current_limit = 35

        config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 1

        self.motor.configurator.apply(config)

    def stop(self) -> None:
        """Attempt to stop the motor"""
        self.motor.stopMotor()

    def run(self, value: float):
        """Spin the motor with this strength"""
        self.motor.set(value)

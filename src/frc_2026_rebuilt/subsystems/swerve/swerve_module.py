"""Represents an individual swerve module with a drive motor, steer motor, and encoder"""

from .config import SwerveModuleConfig
from phoenix6.canbus import CANBus
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.signals import InvertedValue, NeutralModeValue
from ...logger import get_logger

logger = get_logger(__name__)

class SwerveModule:
    """An individual swerve module"""

    def __init__(self, config: SwerveModuleConfig, bus: CANBus, name: str = "Unknown"):

        self.drive_motor = TalonFX(config.drive_motor_id, bus)
        self.steer_motor = TalonFX(config.steer_motor_id, bus)
        self.encoder = CANcoder(config.encoder_id, bus)

        if not self.drive_motor.isAlive():
            raise RuntimeError(f"{name}: Drive Motor is reportedly dead")
        
        if not self.steer_motor.isAlive():
            raise RuntimeError(f"{name}: Steering Motor is reportedly dead")

        logger.info(f"{name}: Ready with config {config}")

    def set_motor_options(self):
        """Configure motor options"""
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        config.current_limits.stator_current_limit_enable = True
        config.current_limits.stator_current_limit = 60
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.supply_current_limit = 35

        config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 1

        self.drive_motor.configurator.apply(config)
        self.steer_motor.configurator.apply(config)
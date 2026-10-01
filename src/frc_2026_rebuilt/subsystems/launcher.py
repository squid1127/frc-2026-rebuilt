"""Launcher subsystem"""


from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from phoenix6.canbus import CANBus
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ..logger import get_logger

logger = get_logger(__name__)


class LauncherSubsystem(Subsystem):
    """Launcher subsystem"""

    def __init__(self, motor_id: int, bus: CANBus):
        super().__init__()

        self.bus = bus
        self.motor_id = motor_id

        self.motor = TalonFX(motor_id, bus)
        self.target = 0
        self.set_motor_options()

        if not self.motor.isAlive():
            raise RuntimeError("Motor is reportedly dead")

        self.limiter = SlewRateLimiter(0.05)
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
        self.target = 0
        self.limiter.reset(0)

    def set(self, value: float):
        """Spin the motor with this strength"""
        self.target = value

    def periodic(self) -> None:
        value = self.limiter.calculate(self.target)
        if abs(value) > 0.01:
            self.motor.set(value)
            # logger.info(f"Target: {value} ({self.target}), motor at {self.motor.get_motor_voltage()}" )
        else:
            self.motor.stopMotor()
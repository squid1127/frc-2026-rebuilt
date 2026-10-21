"""Launcher subsystem"""

import time
from dataclasses import dataclass

import wpilib
from commands2 import Subsystem
from phoenix6.canbus import CANBus
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ..logger import get_logger
from ..network import NTBool, NTFloat

logger = get_logger(__name__)


@dataclass(frozen=True, slots=True)
class LauncherConfig:
    """Configuration for the launcher subsystem.

    Attributes:
        motor_id: The CAN ID of the launcher motor
        target_rev: The target velocity (in RPS) for the launcher motor
        target_threshold: The allowed deviation (in RPS) in velocity for the launcher to be considered ready
        stuck_timeout_s: Time without acceleration toward target before declaring stuck
        stuck_acceleration_deadband_rps2: Minimum acceleration toward target to avoid stuck detection
        unstick_rev_rps: Reverse velocity target while the unstick control is held
    """

    motor_id: int
    target_rev: float
    target_threshold: float
    stuck_timeout_s: float
    stuck_acceleration_deadband_rps2: float
    unstick_rev_rps: float

    def __post_init__(self) -> None:
        if self.stuck_timeout_s <= 0:
            raise ValueError("Launcher stuck timeout must be positive")
        if self.stuck_acceleration_deadband_rps2 < 0:
            raise ValueError("Launcher stuck acceleration deadband cannot be negative")
        if self.unstick_rev_rps <= 0:
            raise ValueError("Launcher unstick reverse velocity must be positive")


class LauncherSubsystem(Subsystem):
    """Launcher subsystem"""

    def __init__(self, config: LauncherConfig, bus: CANBus):
        super().__init__()

        self.bus = bus
        self.config = config

        self.motor = TalonFX(config.motor_id, bus)

        self._launch_started = False
        self._stuck_since: float | None = None

        self.nt_target_rev = NTFloat("Launcher/Target", config.target_rev)
        self.nt_ready = NTBool("Launcher/Ready", False)
        self.nt_stuck = NTBool("Launcher/Stuck", False)

        self.set_motor_options()

        if (not wpilib.RobotBase.isSimulation()) and not (
            self.motor.is_connected and self.motor.isAlive()
        ):
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
        config.motor_output.peak_reverse_duty_cycle = -0.2
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
        self._launch_started = False
        self._stuck_since = None
        self.nt_stuck.value = False
        self.motor.stopMotor()

    def set(self, velocity: float):
        """Spin the motor at the given velocity"""
        self.motor.set_control(VelocityVoltage(velocity))

    def launch(self):
        """Spin the motor at the velocity defined in config"""
        self._launch_started = True
        self._stuck_since = None
        self.set(self.nt_target_rev.value)

    def unstick(self) -> None:
        """Reverse the launcher slowly while the operator holds the unstick control."""
        self._launch_started = False
        self._stuck_since = None
        self.nt_stuck.value = False
        self.set(-self.config.unstick_rev_rps)

    def periodic(self) -> None:
        super().periodic()

        velocity = self.motor.get_velocity(refresh=True).value
        rev = abs(velocity)
        sample_time = time.monotonic()
        target_rev = self.nt_target_rev.value
        self.nt_ready.value = (
            abs(rev - target_rev) <= self.config.target_threshold
        )

        if not self._launch_started or self.nt_ready.value:
            self._stuck_since = None
            self.nt_stuck.value = False
        else:
            acceleration = self.motor.get_acceleration(refresh=True).value
            velocity_direction = (
                (1 if velocity >= 0 else -1)
                if velocity != 0
                else (1 if target_rev >= 0 else -1)
            )
            speed_acceleration = acceleration * velocity_direction
            speed_error = target_rev - rev
            is_correcting = (
                speed_error > 0
                and speed_acceleration
                > self.config.stuck_acceleration_deadband_rps2
            ) or (
                speed_error < 0
                and speed_acceleration
                < -self.config.stuck_acceleration_deadband_rps2
            )
            if is_correcting:
                self._stuck_since = None
                self.nt_stuck.value = False
            else:
                if self._stuck_since is None:
                    self._stuck_since = sample_time
                self.nt_stuck.value = (
                    sample_time - self._stuck_since >= self.config.stuck_timeout_s
                )

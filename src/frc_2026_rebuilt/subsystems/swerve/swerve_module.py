"""Represents an individual swerve module with a drive motor, steer motor, and encoder"""

import math

import wpilib
from phoenix6.canbus import CANBus
from phoenix6.configs import CANcoderConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue, SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from ...logger import get_logger
from .config import SwerveModuleConfig

logger = get_logger(__name__)


class SwerveModule:
    """An individual swerve module"""

    def __init__(self, config: SwerveModuleConfig, bus: CANBus, name: str = "Unknown"):
        self.config = config
        self.name = name
        self.drive_motor = TalonFX(config.drive_motor_id, bus)
        self.steer_motor = TalonFX(config.steer_motor_id, bus)
        self.encoder = CANcoder(config.encoder_id, bus)

        self.set_motor_options()
        self.set_encoder_options()

        if not self.drive_motor.isAlive():
            raise RuntimeError(f"{name}: Drive Motor is reportedly dead")

        if not self.steer_motor.isAlive():
            raise RuntimeError(f"{name}: Steering Motor is reportedly dead")

        if not wpilib.RobotBase.isSimulation() and not self.encoder.is_connected:
            raise RuntimeError(f"{name}: Steering Encoder is reportedly dead")

        logger.info(f"{name}: Ready with config {config}")

    def set_motor_options(self):
        """Configure motor options"""

        drive_config = TalonFXConfiguration()
        drive_config.motor_output.neutral_mode = NeutralModeValue.COAST
        drive_config.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if self.config.drive_motor_inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        drive_config.current_limits.stator_current_limit_enable = True
        drive_config.current_limits.stator_current_limit = 60
        drive_config.current_limits.supply_current_limit_enable = True
        drive_config.current_limits.supply_current_limit = 35
        drive_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 1

        steer_config = TalonFXConfiguration()
        steer_config.motor_output.neutral_mode = NeutralModeValue.COAST
        steer_config.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if self.config.steer_motor_inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        steer_config.current_limits.stator_current_limit_enable = True
        steer_config.current_limits.stator_current_limit = 60
        steer_config.current_limits.supply_current_limit_enable = True
        steer_config.current_limits.supply_current_limit = 35

        self.drive_motor.configurator.apply(drive_config)
        self.steer_motor.configurator.apply(steer_config)

    def set_encoder_options(self) -> None:
        """Apply the configured absolute steering offset."""
        config = CANcoderConfiguration()
        config.magnet_sensor.magnet_offset = self.config.encoder_offset
        config.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if self.config.encoder_inverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.encoder.configurator.apply(config)

    def get_angle(self) -> Rotation2d:
        """Read the calibrated module angle."""
        position = self.encoder.get_absolute_position()
        position.refresh()
        return Rotation2d.fromRotations(position.value)

    def set_desired_state(
        self,
        state: SwerveModuleState,
        max_speed_mps: float,
        steering_kp: float = 0.5,
        max_steering_output: float = 0.4,
    ) -> None:
        """Apply a wheel velocity and angle using bounded open-loop outputs."""
        if max_speed_mps <= 0:
            raise ValueError("max_speed_mps must be greater than zero")

        current_angle = self.get_angle()
        desired_state = SwerveModuleState(state.speed, state.angle)
        desired_state.optimize(current_angle)

        angle_error = (desired_state.angle - current_angle).radians()
        steer_output = max(
            -max_steering_output,
            min(max_steering_output, steering_kp * angle_error),
        )
        drive_output = (
            desired_state.speed / max_speed_mps * math.cos(angle_error)
        )
        drive_output = max(-1.0, min(1.0, drive_output))

        self.steer_motor.set_control(DutyCycleOut(steer_output))
        self.drive_motor.set_control(DutyCycleOut(drive_output))

    def stop(self) -> None:
        """Stop both module motors."""
        self.drive_motor.stopMotor()
        self.steer_motor.stopMotor()
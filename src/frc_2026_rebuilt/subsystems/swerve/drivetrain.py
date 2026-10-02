"""Drivetrain subsystem"""

import math

from commands2 import Subsystem
from phoenix6.canbus import CANBus
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)

from ...logger import get_logger
from .config import SwerveConfig
from .imu import IMU
from .swerve_module import SwerveModule

logger = get_logger(__name__)


class DrivetrainSubsystem(Subsystem):
    """drivetrain subsystem"""

    def __init__(self, config: SwerveConfig, bus: CANBus):
        super().__init__()

        self.bus = bus
        self.config = config

        self.imu = IMU(config.imu_usb)
        self.front_left = SwerveModule(config.fl, bus, "Front Left")
        self.front_right = SwerveModule(config.fr, bus, "Front Right")
        self.back_left = SwerveModule(config.bl, bus, "Back Left")
        self.back_right = SwerveModule(config.br, bus, "Back Right")
        self.modules = (
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        )
        self.locked = False
        self.lock_states = (
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
        )

        half_wheelbase = config.wheelbase_m / 2
        half_trackwidth = config.trackwidth_m / 2
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(half_wheelbase, half_trackwidth),
            Translation2d(half_wheelbase, -half_trackwidth),
            Translation2d(-half_wheelbase, half_trackwidth),
            Translation2d(-half_wheelbase, -half_trackwidth),
        )

        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.field_relative = True
        self.forward_limiter = SlewRateLimiter(3.0)
        self.strafe_limiter = SlewRateLimiter(3.0)
        self.rotation_limiter = SlewRateLimiter(3.0)
        logger.info("Swerve system ready")

    def stop(self) -> None:
        """Stop all output"""
        self.locked = False
        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.forward_limiter.reset(0.0)
        self.strafe_limiter.reset(0.0)
        self.rotation_limiter.reset(0.0)
        for module in self.modules:
            module.stop()

    def reset_yaw(self) -> None:
        """Reset the yaw measurement of the bot"""
        self.imu.reset_yaw()

    def lock(self) -> None:
        """Hold the wheels in an X pattern to resist being pushed."""
        if self.locked:
            return

        self.locked = True
        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.forward_limiter.reset(0.0)
        self.strafe_limiter.reset(0.0)
        self.rotation_limiter.reset(0.0)

    def set(
        self,
        forward: float,
        strafe: float = 0.0,
        rotation: float = 0.0,
        *,
        field_relative: bool = True,
    ) -> None:
        """Set normalized forward, strafe, and rotation demands."""
        self.locked = False
        self.field_relative = field_relative
        self.forward_target = self._apply_deadband(forward)
        self.strafe_target = self._apply_deadband(strafe)
        self.rotation_target = self._apply_deadband(rotation)

    def periodic(self) -> None:
        """Tick system"""
        if self.locked:
            for module, state in zip(self.modules, self.lock_states, strict=True):
                module.set_desired_state(
                    state,
                    self.config.max_speed_mps,
                    self.config.steering_kp,
                    self.config.max_steering_output,
                )
            return

        forward = self.forward_limiter.calculate(self.forward_target)
        strafe = self.strafe_limiter.calculate(self.strafe_target)
        rotation = self.rotation_limiter.calculate(self.rotation_target)
        forward *= self.config.speed_scale
        strafe *= self.config.speed_scale
        rotation *= self.config.speed_scale

        if max(abs(forward), abs(strafe), abs(rotation)) < 0.01:
            for module in self.modules:
                module.stop()
            return

        max_angular_speed = self.config.max_speed_mps / math.hypot(
            self.config.wheelbase_m / 2,
            self.config.trackwidth_m / 2,
        )
        if self.field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward * self.config.max_speed_mps,
                strafe * self.config.max_speed_mps,
                rotation * max_angular_speed,
                self.imu.get_yaw(),
            )
        else:
            chassis_speeds = ChassisSpeeds(
                forward * self.config.max_speed_mps,
                strafe * self.config.max_speed_mps,
                rotation * max_angular_speed,
            )
        states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        states = self.kinematics.desaturateWheelSpeeds(
            states, self.config.max_speed_mps
        )

        for module, state in zip(self.modules, states, strict=True):
            module.set_desired_state(
                state,
                self.config.max_speed_mps,
                self.config.steering_kp,
                self.config.max_steering_output,
            )

    @staticmethod
    def _apply_deadband(value: float, deadband: float = 0.05) -> float:
        value = max(-1.0, min(1.0, value))
        if abs(value) <= deadband:
            return 0.0
        return math.copysign((abs(value) - deadband) / (1 - deadband), value)

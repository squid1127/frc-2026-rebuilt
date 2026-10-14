"""Drivetrain subsystem"""

import math

from commands2 import Subsystem
from phoenix6.canbus import CANBus
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.controller import PIDController

from ...logger import get_logger
from ...network import NTFloat, NTPose2d, NTSwerveModuleStateArray
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
        self.distance_per_motor_rotation_m = (
            math.pi * config.wheel_diameter_in * 0.0254 / config.drive_gear_ratio
        )
        self.yaw_offset = Rotation2d()
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.imu.get_yaw() + self.yaw_offset,
            self._get_module_positions(),
        )

        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.field_relative = True
        self.align_only = False
        self.forward_limiter = SlewRateLimiter(config.slew_rate)
        self.strafe_limiter = SlewRateLimiter(config.slew_rate)
        self.rotation_limiter = SlewRateLimiter(config.steering_slew_rate)
        
        self.yaw_pid = PIDController

        self.nt_speed_scale = NTFloat("Drivetrain/Speed Multiplier", config.speed_scale)
        self.nt_max_speed_mps = NTFloat("Drivetrain/Speed Cap", config.max_speed_mps)
        self.nt_yaw = NTFloat("Drivetrain/Yaw", 0)
        self.nt_offset = NTFloat("Drivetrain/Yaw Offset", 0)
        self.nt_f_out = NTFloat("Drivetrain/Output", 0)
        self.nt_s_out = NTFloat("Drivetrain/Strafe Output", 0)
        self.nt_y_out = NTFloat("Drivetrain/Yaw Output", 0)
        self.nt_pose = NTPose2d("Drivetrain/Pose", Pose2d())
        self.nt_module_state = NTSwerveModuleStateArray("Drivetrain/Module State", [])

        logger.info("Swerve system initialized")

    def stop(self) -> None:
        """Stop all output"""
        self.locked = False
        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.align_only = False
        self.forward_limiter.reset(0.0)
        self.strafe_limiter.reset(0.0)
        self.rotation_limiter.reset(0.0)
        for module in self.modules:
            module.stop()

    def reset_yaw(self) -> None:
        """Reset the yaw measurement of the bot (including offset)"""
        self.imu.reset_yaw()
        self.yaw_offset = Rotation2d()
        yaw = self.imu.get_yaw()
        current_pose = self.odometry.getPose()
        self.odometry.resetPosition(
            yaw,
            self._get_module_positions(),
            Pose2d(current_pose.translation(), yaw),
        )
        logger.info("Reset yaw")

    def set_speed_scale(self, speed: float):
        """Change the speed multiplier of the drivetrain"""
        if not 0 <= speed <= 1:
            raise ValueError("Speed must be between 0 and 1 inclusive")
        self.nt_speed_scale.set(speed)

    def offset_yaw(self, degrees: float) -> None:
        """Adjust the yaw used for field-relative driving."""
        self.yaw_offset += Rotation2d.fromDegrees(degrees)
        self.nt_offset.set(self.yaw_offset.radians())

    def lock(self) -> None:
        """Hold the wheels in an X pattern to resist being pushed."""
        if self.locked:
            return

        self.locked = True
        self.forward_target = 0.0
        self.strafe_target = 0.0
        self.rotation_target = 0.0
        self.align_only = False
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
        align_only: bool = False,
    ) -> None:
        """Set normalized drive demands, optionally steering without moving."""
        self.locked = False
        self.field_relative = field_relative
        self.align_only = align_only
        self.forward_target = self._apply_deadband(forward)
        self.strafe_target = self._apply_deadband(strafe)
        self.rotation_target = self._apply_deadband(rotation)

    def reset_pose(self):
        """Reset the robot's pose odometry"""
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.imu.get_yaw() + self.yaw_offset,
            self._get_module_positions(),
            initialPose=Pose2d(Translation2d(0, 3), Rotation2d(0)),
        )
        logger.info("Reset pose")

    def periodic(self) -> None:
        """Tick system"""
        yaw = self.imu.get_yaw() + self.yaw_offset
        pose = self.odometry.update(yaw, self._get_module_positions())
        self.nt_pose.set(pose)
        self.nt_yaw.set(yaw.radians())

        if self.locked:
            self._publish_module_states(self.lock_states)
            for module, state in zip(self.modules, self.lock_states, strict=True):
                module.set_desired_state(
                    state,
                    self.max_speed_mps,
                    self.config.steering_scale,
                    self.config.max_steering_output,
                )
            return

        forward = self.forward_limiter.calculate(self.forward_target)
        strafe = self.strafe_limiter.calculate(self.strafe_target)
        rotation = self.rotation_limiter.calculate(self.rotation_target)
        forward *= self.nt_speed_scale.value
        strafe *= self.nt_speed_scale.value
        rotation *= self.nt_speed_scale.value
        self.nt_f_out.value = forward
        self.nt_s_out.value = strafe
        self.nt_y_out.value = rotation

        if max(abs(forward), abs(strafe), abs(rotation)) < 0.01:
            self._publish_module_states(
                tuple(
                    SwerveModuleState(0.0, module.get_angle())
                    for module in self.modules
                )
            )
            for module in self.modules:
                module.stop()
            return

        max_angular_speed = self.max_speed_mps / math.hypot(
            self.config.wheelbase_m / 2,
            self.config.trackwidth_m / 2,
        )
        if self.field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward * self.max_speed_mps,
                strafe * self.max_speed_mps,
                rotation * max_angular_speed,
                self.imu.get_yaw() + self.yaw_offset,
            )
        else:
            chassis_speeds = ChassisSpeeds(
                forward * self.max_speed_mps,
                strafe * self.max_speed_mps,
                rotation * max_angular_speed,
            )
        states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        states = self.kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)
        if self.align_only:
            states = [SwerveModuleState(0.0, state.angle) for state in states]

        self._publish_module_states(states)
        for module, state in zip(self.modules, states, strict=True):
            module.set_desired_state(
                state,
                self.max_speed_mps,
                self.config.steering_scale,
                self.config.max_steering_output,
            )

    def _publish_module_states(
        self, states: tuple[SwerveModuleState, ...] | list[SwerveModuleState]
    ) -> None:
        self.nt_module_state.set(list(states))

    def _get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        distance_per_motor_rotation_m = self.distance_per_motor_rotation_m
        return (
            self.front_left.get_position(distance_per_motor_rotation_m),
            self.front_right.get_position(distance_per_motor_rotation_m),
            self.back_left.get_position(distance_per_motor_rotation_m),
            self.back_right.get_position(distance_per_motor_rotation_m),
        )

    @property
    def max_speed_mps(self) -> float:
        return self.nt_max_speed_mps.value

    @staticmethod
    def _apply_deadband(value: float, deadband: float = 0.05) -> float:
        value = max(-1.0, min(1.0, value))
        if abs(value) <= deadband:
            return 0.0
        return math.copysign((abs(value) - deadband) / (1 - deadband), value)

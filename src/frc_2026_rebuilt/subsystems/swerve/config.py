"""Configuration for the swerve subsystem"""

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
    """Configuration for an individual module"""
    drive_motor_id: int
    steer_motor_id: int
    encoder_id: int

    encoder_offset: float
    drive_motor_inverted: bool = True
    steer_motor_inverted: bool = True
    encoder_inverted: bool = False

@dataclass(frozen=True, slots=True)
class SwerveConfig:
    """Configuration for the swerve subsystem"""

    imu_usb: Literal[1, 2]
    wheelbase_m: float
    trackwidth_m: float
    max_speed_mps: float
    speed_scale: float
    steering_kp: float
    max_steering_output: float
    fr: SwerveModuleConfig
    fl: SwerveModuleConfig
    br: SwerveModuleConfig
    bl: SwerveModuleConfig

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0 or self.trackwidth_m <= 0:
            raise ValueError("Swerve wheelbase and trackwidth must be positive")
        if self.max_speed_mps <= 0:
            raise ValueError("Swerve maximum speed must be positive")
        if not 0 <= self.speed_scale <= 1:
            raise ValueError("Swerve speed scale must be between zero and one")
        if self.steering_kp < 0:
            raise ValueError("Swerve steering gain cannot be negative")
        if not 0 <= self.max_steering_output <= 1:
            raise ValueError("Maximum steering output must be between zero and one")
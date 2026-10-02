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

@dataclass(frozen=True, slots=True)
class SwerveConfig:
    """Configuration for the swerve subsystem"""

    imu_usb: Literal[1, 2]
    fr: SwerveModuleConfig
    fl: SwerveModuleConfig
    br: SwerveModuleConfig
    bl: SwerveModuleConfig
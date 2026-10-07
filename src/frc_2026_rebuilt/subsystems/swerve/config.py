"""Configuration for the swerve subsystem"""

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
    """Configuration for an individual module
    
    Attributes:
        drive_motor_id: The CAN ID of the drive motor
        steer_motor_id: The CAN ID of the steering motor
        encoder_id: The CAN ID of the encoder
        encoder_offset: The offset of the encoder
        drive_motor_inverted: Whether the drive motor is inverted
        steer_motor_inverted: Whether the steering motor is inverted
        encoder_inverted: Whether the encoder is inverted
    """
    drive_motor_id: int
    steer_motor_id: int
    encoder_id: int

    encoder_offset: float
    drive_motor_inverted: bool = True
    steer_motor_inverted: bool = True
    encoder_inverted: bool = False

@dataclass(frozen=True, slots=True)
class SwerveConfig:
    """Configuration for the swerve subsystem
    
    Attributes:
        imu_usb: The USB port the IMU is connected to
        wheelbase_m: The distance between the front and back wheels in meters (from the center of each wheel)
        trackwidth_m: The distance between the left and right wheels in meters (from the center of each wheel)
        max_speed_mps: The maximum speed of the robot in meters per second
        speed_scale: The scale factor for the robot's speed
        slew_rate: The maximum rate of change for the robot's speed
        steering_scale: The scale factor for the steering modules
        steering_slew_rate: The maximum rate of change for the steering modules
        max_steering_output: The maximum output for the steering control
        fr: Configuration for the front-right swerve module
        fl: Configuration for the front-left swerve module
        br: Configuration for the back-right swerve module
        bl: Configuration for the back-left swerve module
    """

    imu_usb: Literal[1, 2]
    wheelbase_m: float
    trackwidth_m: float
    max_speed_mps: float
    speed_scale: float
    slew_rate: float
    steering_scale: float
    steering_slew_rate: float
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
        if self.steering_scale < 0:
            raise ValueError("Swerve steering gain cannot be negative")
        if not 0 <= self.max_steering_output <= 1:
            raise ValueError("Maximum steering output must be between zero and one")
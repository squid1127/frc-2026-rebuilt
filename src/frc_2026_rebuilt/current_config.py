"""Hardcoded configuration for the current robot."""

from .config import IntakeConfig, LauncherConfig, RobotConfig
from .subsystems.swerve.config import SwerveConfig, SwerveModuleConfig


class CurrentBotConfig:
    """The current robot's hardware IDs and tuning values."""

    CONFIG = RobotConfig(
        intake=IntakeConfig(motor_id=14),
        launcher=LauncherConfig(motor_id=17),
        swerve=SwerveConfig(
            imu_usb=1,
            wheelbase_m=0.5715,
            trackwidth_m=0.5715,
            max_speed_mps=0.1,
            speed_scale=0.2,
            steering_kp=0.15,
            max_steering_output=0.4,
            fr=SwerveModuleConfig(
                drive_motor_id=9,
                steer_motor_id=8,
                encoder_id=10,
                encoder_offset=0.28,
                drive_motor_inverted=True,
                steer_motor_inverted=True,
                encoder_inverted=False,
            ),
            fl=SwerveModuleConfig(
                drive_motor_id=12,
                steer_motor_id=11,
                encoder_id=13,
                encoder_offset=0.747,
                drive_motor_inverted=True,
                steer_motor_inverted=True,
                encoder_inverted=False,
            ),
            br=SwerveModuleConfig(
                drive_motor_id=3,
                steer_motor_id=2,
                encoder_id=4,
                encoder_offset=0.987,
                drive_motor_inverted=True,
                steer_motor_inverted=True,
                encoder_inverted=False,
            ),
            bl=SwerveModuleConfig(
                drive_motor_id=6,
                steer_motor_id=5,
                encoder_id=7,
                encoder_offset=0.026806640625,
                drive_motor_inverted=True,
                steer_motor_inverted=True,
                encoder_inverted=False,
            ),
        ),
    )
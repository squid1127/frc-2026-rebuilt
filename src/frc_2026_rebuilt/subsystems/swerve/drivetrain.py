"""Drivetrain subsystem"""

from commands2 import Subsystem
from phoenix6.canbus import CANBus
from wpimath.filter import SlewRateLimiter

from ...logger import get_logger
from .config import SwerveConfig
from .imu import IMU

logger = get_logger(__name__)


class DrivetrainSubsystem(Subsystem):
    """drivetrain subsystem"""

    def __init__(self, config: SwerveConfig, bus: CANBus):
        super().__init__()

        self.bus = bus
        self.config = config

        self.imu = IMU(config.imu_usb)

        self.limiter = SlewRateLimiter(0.1)
        logger.info("Swerve system ready")

    def stop(self) -> None:
        """Stop all output"""

    def reset_yaw(self) -> None:
        """Reset the yaw measurement of the bot"""
        self.imu.reset_yaw()

    def set(self, value: float):
        """Spin the motor with this strength"""

    def periodic(self) -> None:
        """Tick system"""
        
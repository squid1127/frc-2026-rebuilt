"""IMU subsystem"""

from dataclasses import dataclass
from typing import Literal

from navx import AHRS
from wpimath.geometry import Rotation2d

from ...logger import get_logger

logger = get_logger(__name__)

@dataclass(slots=True, frozen=True)
class IMUOutput:
    """Represents an output from a IMU"""

class IMU:
    """Launcher subsystem"""

    def __init__(self, index: Literal[1, 2]):
        super().__init__()

        self.index = index
        self.imu = AHRS(AHRS.NavXComType.kUSB1 if index == 1 else AHRS.NavXComType.kUSB2)

        logger.info("Gyro ready")

    def get_yaw(self) -> Rotation2d:
        return self.imu.getRotation2d()
    def get_yaw_rate_dps(self) -> float:
        return -self.imu.getRate()
    def is_connected(self) -> bool:
        return self.imu.isConnected()
    def get_pitch(self) -> float:
        return self.imu.getPitch()
    def get_roll(self) -> float:
        return self.imu.getRoll()

    def reset_yaw(self):
        """Reset the IUM's yaw reading"""
        self.imu.reset()
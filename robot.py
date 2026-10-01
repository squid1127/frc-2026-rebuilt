"""Container for the project"""

import logging

import wpilib
from commands2 import TimedCommandRobot


from src.frc_2026_rebuilt import RobotContainer

class RobotEntrypoint(RobotContainer):
    pass


if __name__ == "__main__":
    wpilib.run(RobotEntrypoint())

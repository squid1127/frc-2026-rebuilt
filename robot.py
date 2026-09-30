"""Main robot class"""

import wpilib
import logging
import coloredlogs
from phoenix6.canbus import CANBus
import math

from models.state import RobotState
from subsystems.intake import IntakeSubsystem

coloredlogs.install(
    level=logging.DEBUG,
    fmt="%(asctime)s %(name)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        logger.info("Initializing...")

        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()

        self.bus = CANBus()
        self.intake = IntakeSubsystem(17, self.bus)

    def tick(self, state: RobotState):
        if state == RobotState.TELEOP:
            if abs(self.controller.getRightY()) > 0.03:
                self.intake.run(self.controller.getRightY() * 0.25)
            else:
                self.intake.stop()
        else:
            self.intake.stop()

    def autonomousInit(self):
        """Init on auto."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """Tick on auto."""
        self.tick(RobotState.AUTO)

    def teleopInit(self):
        """Init on teleop"""
        if not self.controller.isConnected():
            logger.error("Missing controller")

    def teleopPeriodic(self):
        """Tick on teleop"""
        self.tick(RobotState.TELEOP)

    def disabledPeriodic(self):
        """Tick on disabled"""
        self.tick(RobotState.DISABLED)

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)

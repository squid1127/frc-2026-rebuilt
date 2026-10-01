"""Container for the project"""

import logging

import wpilib
from coloredlogs import install as cl_install

cl_install(
    level=logging.DEBUG,
    fmt="%(asctime)s %(name)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

from src.frc_2026_rebuilt import Robot, RobotState


class RobotContainer(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        logger.info("Robot starting up...")
        self.robot = Robot()

    def autonomousInit(self):
        """Init on auto."""
        self.robot.state_transition(RobotState.AUTO)

    def teleopInit(self):
        """Init on teleop"""
        self.robot.state_transition(RobotState.TELEOP)

    def testInit(self):
        """Init on test."""
        self.robot.state_transition(RobotState.TEST)

    def disabledInit(self):
        """Init on disabled"""
        self.robot.state_transition(RobotState.DISABLED)

    # * Tick Methods

    def autonomousPeriodic(self):
        """Tick on auto."""
        self.robot.tick(RobotState.AUTO)

    def teleopPeriodic(self):
        """Tick on teleop"""
        self.robot.tick(RobotState.TELEOP)

    def disabledPeriodic(self):
        """Tick on disabled"""
        self.robot.tick(RobotState.DISABLED)

    def testPeriodic(self):
        """Tick on test."""
        self.robot.tick(RobotState.TEST)


if __name__ == "__main__":
    wpilib.run(RobotContainer)

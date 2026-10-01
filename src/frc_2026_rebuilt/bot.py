"""Main robot class"""

from logging import getLogger

import wpilib
from phoenix6.canbus import CANBus

from .models.state import RobotState
from .subsystems.intake import IntakeSubsystem

logger = getLogger(__name__)


class Robot:
    """Main robot class"""

    def __init__(self):
        logger.info("Initializing...")

        self.tick_index = 0

        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()

        self.bus = CANBus()
        self.intake = IntakeSubsystem(17, self.bus)

    def tick(self, state: RobotState):
        """Executed every robot tick"""
        self.tick_index += 1

        if state == RobotState.TELEOP:
            if abs(self.controller.getRightY()) > 0.03:
                self.intake.run(self.controller.getRightY() * 0.25)
            else:
                self.intake.stop()
        else:
            self.intake.stop()

    def state_transition(self, new_state: RobotState):
        logger.info(f"Transitioning to {new_state.name}")
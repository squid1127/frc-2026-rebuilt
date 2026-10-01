"""Handle logging for the project"""

from coloredlogs import install as cl_install
import logging

cl_install(
    level=logging.DEBUG,
    fmt="%(asctime)s %(name)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)


def get_logger(module: str) -> logging.Logger:
    """Get a logger for the given module"""
    return logging.getLogger(module)

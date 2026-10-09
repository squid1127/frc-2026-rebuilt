"""Syncs an attribute with SmartDashboard with the given name and kind"""

from typing import Generic, TypeVar, overload

from wpilib import SmartDashboard

from .logger import get_logger

T = TypeVar("T")

logger = get_logger(__name__)


class NTField(Generic[T]):
    """Syncs an attribute with SmartDashboard with the given name and kind"""

    def __init__(self, name: str, default: T, kind: str) -> None:
        self.name = name
        self.default = default
        self.kind = kind
    

    @overload
    def __get__(self, instance: None, owner) -> "NTField[T]": ...
    @overload
    def __get__(self, instance: object, owner) -> T: ...
    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return getattr(SmartDashboard, f"get{self.kind_method}")(
            self.name, self.default
        )

    def __set__(self, instance, value: T):
        getattr(SmartDashboard, f"put{self.kind_method}")(self.name, value)
        

    @property
    def kind_method(self) -> str:
        return self.kind.lower().title()

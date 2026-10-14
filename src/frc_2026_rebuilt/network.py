"""Syncs an attribute with SmartDashboard with the given name and kind"""

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from typing import (
    Any,
    Generic,
    Protocol,
    SupportsFloat,
    SupportsIndex,
    SupportsInt,
    TypeVar,
)

import ntcore
from wpimath.geometry import Pose2d
from wpimath.kinematics import SwerveModuleState

from .logger import get_logger

T = TypeVar("T")

logger = get_logger(__name__)


class Gettable[T](Protocol):
    """An object with a getter"""

    def get(self) -> T: ...


class Settable[T](Protocol):
    """An object with a setter"""

    def set(
        self,
        value: T,
    ) -> Any: ...


class _GetterAdapter[T]:
    def __init__(self, getter: Callable[[], T]) -> None:
        self._getter = getter

    def get(self) -> T:
        return self._getter()


class _SetterAdapter[T]:
    def __init__(self, setter: Callable[[T], None]) -> None:
        self._setter = setter

    def set(self, value: T) -> None:
        self._setter(value)


class NTVariable(ABC, Generic[T]):
    """A variable stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: T,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        self._name = name
        self._default = default
        self._table = table or ntcore.NetworkTableInstance.getDefault()
        self._get = self.gettable(self._table)
        self._set = self.settable(self._table)
        self.set_default()

    @abstractmethod
    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[T]:
        """Return an object with a get method."""
        raise NotImplementedError()

    @abstractmethod
    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[T]:
        """Return an object with a set method."""
        raise NotImplementedError()

    def __get__(self, instance: object | None, owner=None) -> Any:
        if instance is None:
            return self
        return self.get()

    def __set__(self, instance: object | None, value: T) -> None:
        if instance is not None:
            self.set(value)

    def get(self) -> T:
        """Get the value"""
        return self._get.get()

    def set(self, value: T) -> None:
        """Set the value"""
        self._set.set(value)

    def set_default(self):
        """Set to the default value"""
        self.set(self.default)

    @property
    def name(self) -> str:
        return self._name

    @property
    def value(self) -> T:
        return self.get()

    @value.setter
    def value(self, v: T) -> None:
        return self.set(v)

    @property
    def default(self) -> T:
        return self._default


class NTFloat(NTVariable[float]):
    """A float variable stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: float,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[float]:
        return table.getFloatTopic(self.name).subscribe(defaultValue=self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[float]:
        return table.getFloatTopic(self.name).publish()


class NTDouble(NTFloat):
    """A double-precision number stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: float,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)


class NTBool(NTVariable[bool]):
    """A boolean variable stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: bool,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[bool]:
        return table.getBooleanTopic(self.name).subscribe(defaultValue=self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[bool]:
        return table.getBooleanTopic(self.name).publish()


class NTBoolean(NTBool):
    """Compatibility alias for boolean NetworkTables values."""


class NTInt(NTVariable[int]):
    """An integer variable stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: int,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[int]:
        return table.getIntegerTopic(self.name).subscribe(defaultValue=self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[int]:
        return table.getIntegerTopic(self.name).publish()


class NTString(NTVariable[str]):
    """A string variable stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: str,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[str]:
        return table.getStringTopic(self.name).subscribe(defaultValue=self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[str]:
        return table.getStringTopic(self.name).publish()


class NTStructArray[NestedT](NTVariable[list[NestedT]]):
    """A structured array topic stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: list[NestedT],
        kind: type[NestedT],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        self._kind = kind
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[list[NestedT]]:
        return table.getStructArrayTopic(self.name, self._kind).subscribe(self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[list[NestedT]]:
        return table.getStructArrayTopic(self.name, self._kind).publish()


class NTPose2d(NTVariable[Pose2d]):
    """A Pose2d value stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: Pose2d,
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[Pose2d]:
        return table.getStructTopic(self.name, Pose2d).subscribe(self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[Pose2d]:
        return table.getStructTopic(self.name, Pose2d).publish()


class NTSwerveModuleStateArray(NTStructArray[SwerveModuleState]):
    """An array of swerve module states stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: list[SwerveModuleState],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, SwerveModuleState, table)


class NTSwerveModuleState(NTSwerveModuleStateArray):
    """Compatibility name for an array of swerve module states."""


class NTFloatArray(NTVariable[list[float]]):
    """A float array stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: Sequence[float],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, list(default), table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[list[float]]:
        topic_default: list[SupportsFloat | SupportsIndex] = [
            float(value) for value in self.default
        ]
        subscriber = table.getFloatArrayTopic(self.name).subscribe(topic_default)
        return _GetterAdapter(lambda: [float(value) for value in subscriber.get()])

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[list[float]]:
        publisher = table.getFloatArrayTopic(self.name).publish()

        def publish(values: list[float]) -> None:
            topic_values: list[SupportsFloat | SupportsIndex] = [
                float(value) for value in values
            ]
            publisher.set(topic_values)

        return _SetterAdapter(publish)


class NTBoolArray(NTVariable[list[bool]]):
    """A boolean array stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: Sequence[bool],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, list(default), table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[list[bool]]:
        topic_default: list[SupportsInt | SupportsIndex] = [
            int(value) for value in self.default
        ]
        subscriber = table.getBooleanArrayTopic(self.name).subscribe(topic_default)
        return _GetterAdapter(lambda: [bool(value) for value in subscriber.get()])

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[list[bool]]:
        publisher = table.getBooleanArrayTopic(self.name).publish()

        def publish(values: list[bool]) -> None:
            topic_values: list[SupportsInt | SupportsIndex] = [
                int(value) for value in values
            ]
            publisher.set(topic_values)

        return _SetterAdapter(publish)


class NTIntArray(NTVariable[list[int]]):
    """An integer array stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: Sequence[int],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, list(default), table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[list[int]]:
        topic_default: list[SupportsInt | SupportsIndex] = [
            int(value) for value in self.default
        ]
        subscriber = table.getIntegerArrayTopic(self.name).subscribe(topic_default)
        return _GetterAdapter(lambda: [int(value) for value in subscriber.get()])

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[list[int]]:
        publisher = table.getIntegerArrayTopic(self.name).publish()

        def publish(values: list[int]) -> None:
            topic_values: list[SupportsInt | SupportsIndex] = [
                int(value) for value in values
            ]
            publisher.set(topic_values)

        return _SetterAdapter(publish)


class NTStringArray(NTVariable[list[str]]):
    """A string array stored in NetworkTables."""

    def __init__(
        self,
        name: str,
        default: list[str],
        table: ntcore.NetworkTableInstance | None = None,
    ) -> None:
        super().__init__(name, default, table)

    def gettable(self, table: ntcore.NetworkTableInstance) -> Gettable[list[str]]:
        return table.getStringArrayTopic(self.name).subscribe(self.default)

    def settable(self, table: ntcore.NetworkTableInstance) -> Settable[list[str]]:
        return table.getStringArrayTopic(self.name).publish()

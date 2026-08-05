"""Hardware-independent types for unified point-to-point motion control."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class AxisName(str, Enum):
    SLIDE = "slide"
    Z = "z"
    SHOULDER = "shoulder"
    ELBOW = "elbow"
    ROTATION = "rotation"


class AxisKind(str, Enum):
    LINEAR = "linear"
    ROTARY = "rotary"


@dataclass(frozen=True)
class AxisCapabilities:
    query_state: bool
    move_absolute: bool
    stop: bool
    reference_home: bool
    configurable_velocity: bool
    configurable_acceleration: bool
    arrival_confirmation: bool


@dataclass(frozen=True)
class AxisDescriptor:
    name: AxisName
    display_name: str
    kind: AxisKind
    position_unit: str
    velocity_unit: str
    acceleration_unit: str
    minimum_position: float
    maximum_position: float
    capabilities: AxisCapabilities

    def __post_init__(self) -> None:
        if not isinstance(self.name, AxisName):
            raise ValueError("name must be an AxisName")
        if not isinstance(self.kind, AxisKind):
            raise ValueError("kind must be an AxisKind")
        if not self.display_name.strip():
            raise ValueError("display_name must not be empty")
        for field_name in (
            "position_unit",
            "velocity_unit",
            "acceleration_unit",
        ):
            if not getattr(self, field_name).strip():
                raise ValueError(f"{field_name} must not be empty")
        _require_finite("minimum_position", self.minimum_position)
        _require_finite("maximum_position", self.maximum_position)
        if self.minimum_position >= self.maximum_position:
            raise ValueError("minimum_position must be below maximum_position")


@dataclass(frozen=True)
class AxisTarget:
    axis: AxisName
    position: float
    velocity: float | None = None
    acceleration: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.axis, AxisName):
            raise ValueError("axis must be an AxisName")
        _require_finite("position", self.position)
        _require_optional_positive("velocity", self.velocity)
        _require_optional_positive("acceleration", self.acceleration)


@dataclass(frozen=True)
class MultiAxisTarget:
    targets: tuple[AxisTarget, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.targets, tuple):
            raise ValueError("targets must be a tuple")
        if not self.targets:
            raise ValueError("at least one axis target is required")
        axes: set[AxisName] = set()
        for target in self.targets:
            if not isinstance(target, AxisTarget):
                raise ValueError("targets must contain only AxisTarget values")
            if target.axis in axes:
                raise ValueError(f"duplicate target for axis {target.axis.value}")
            axes.add(target.axis)


class MotionCommandStatus(str, Enum):
    ACCEPTED = "accepted"
    MOVING = "moving"
    ARRIVED = "arrived"
    REJECTED = "rejected"
    ABORTED = "aborted"
    TIMEOUT = "timeout"
    FAULT = "fault"
    COMMUNICATION_ERROR = "communication_error"


class MotionErrorCode(str, Enum):
    INVALID_REQUEST = "invalid_request"
    UNKNOWN_AXIS = "unknown_axis"
    BACKEND_UNAVAILABLE = "backend_unavailable"
    UNSUPPORTED_PARAMETER = "unsupported_parameter"
    UNSUPPORTED_COMMAND = "unsupported_command"
    POSITION_INVALID = "position_invalid"
    NOT_HOMED = "not_homed"
    SOFT_LIMIT = "soft_limit"
    BUSY = "busy"
    TIMEOUT = "timeout"
    DEVICE_FAULT = "device_fault"
    COMMUNICATION_ERROR = "communication_error"
    BACKEND_ERROR = "backend_error"


@dataclass(frozen=True)
class MotionCommandHandle:
    command_id: str
    axis: AxisName
    target_position: float


@dataclass(frozen=True)
class MultiAxisCommandHandle:
    group_id: str
    commands: tuple[MotionCommandHandle, ...]


@dataclass(frozen=True)
class AxisState:
    axis: AxisName
    connected: bool
    enabled: bool | None
    busy: bool | None
    homed: bool | None
    position_valid: bool
    current_position: float | None
    position_unit: str
    faulted: bool
    fault_code: str | int | None
    fault_message: str | None


@dataclass(frozen=True)
class RotaryJointEnableStatus:
    """三个保持姿态旋转关节的真实使能状态快照。"""

    shoulder: bool | None
    elbow: bool | None
    rotation: bool | None

    @property
    def all_enabled(self) -> bool:
        return self.shoulder is True and self.elbow is True and self.rotation is True


@dataclass(frozen=True)
class MotionCommandResult:
    command_id: str
    axis: AxisName
    status: MotionCommandStatus
    accepted: bool
    completed: bool | None
    target_position: float
    final_position: float | None
    position_error: float | None
    error_code: MotionErrorCode | None
    message: str

    def __post_init__(self) -> None:
        expected = _RESULT_SEMANTICS[self.status]
        if (self.accepted, self.completed) != expected:
            raise ValueError(
                f"{self.status.value} requires accepted/completed={expected}, got "
                f"{(self.accepted, self.completed)}"
            )
        if self.status in _FAILURE_STATUSES and self.error_code is None:
            raise ValueError(f"{self.status.value} requires an error_code")
        if self.status not in _FAILURE_STATUSES and self.error_code is not None:
            raise ValueError(f"{self.status.value} must not have an error_code")


@dataclass(frozen=True)
class MultiAxisCommandResult:
    group_id: str
    status: MotionCommandStatus
    results: tuple[MotionCommandResult, ...]
    accepted: bool
    completed: bool | None
    message: str

    def __post_init__(self) -> None:
        if not self.results:
            raise ValueError("group result must contain at least one axis result")
        expected = _RESULT_SEMANTICS[self.status]
        if (self.accepted, self.completed) != expected:
            raise ValueError(
                f"{self.status.value} group requires accepted/completed={expected}"
            )


@dataclass(frozen=True)
class ArrivalConfig:
    position_tolerance: float
    stable_time_s: float
    poll_interval_s: float
    default_timeout_s: float

    def __post_init__(self) -> None:
        _require_positive("position_tolerance", self.position_tolerance)
        _require_finite("stable_time_s", self.stable_time_s)
        if self.stable_time_s < 0:
            raise ValueError("stable_time_s must be non-negative")
        _require_positive("poll_interval_s", self.poll_interval_s)
        _require_positive("default_timeout_s", self.default_timeout_s)


_RESULT_SEMANTICS = {
    MotionCommandStatus.ACCEPTED: (True, None),
    MotionCommandStatus.MOVING: (True, None),
    MotionCommandStatus.ARRIVED: (True, True),
    MotionCommandStatus.REJECTED: (False, False),
    MotionCommandStatus.ABORTED: (True, False),
    MotionCommandStatus.TIMEOUT: (True, False),
    MotionCommandStatus.FAULT: (True, False),
    MotionCommandStatus.COMMUNICATION_ERROR: (True, False),
}

_FAILURE_STATUSES = {
    MotionCommandStatus.REJECTED,
    MotionCommandStatus.ABORTED,
    MotionCommandStatus.TIMEOUT,
    MotionCommandStatus.FAULT,
    MotionCommandStatus.COMMUNICATION_ERROR,
}


def _require_finite(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _require_positive(name: str, value: object) -> None:
    _require_finite(name, value)
    if value <= 0:  # type: ignore[operator]
        raise ValueError(f"{name} must be positive")


def _require_optional_positive(name: str, value: object | None) -> None:
    if value is not None:
        _require_positive(name, value)


__all__ = [
    "ArrivalConfig",
    "AxisCapabilities",
    "AxisDescriptor",
    "AxisKind",
    "AxisName",
    "AxisState",
    "AxisTarget",
    "MotionCommandHandle",
    "MotionCommandResult",
    "MotionCommandStatus",
    "MotionErrorCode",
    "MultiAxisCommandHandle",
    "MultiAxisCommandResult",
    "MultiAxisTarget",
    "RotaryJointEnableStatus",
]

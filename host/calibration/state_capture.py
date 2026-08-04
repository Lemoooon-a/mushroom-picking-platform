"""从统一只读状态接口稳定采集五轴逻辑位置。"""

from __future__ import annotations

from collections.abc import Callable
import math
import time

from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import AxisName, AxisState


ALL_AXIS_NAMES = (
    AxisName.SLIDE,
    AxisName.Z,
    AxisName.SHOULDER,
    AxisName.ELBOW,
    AxisName.ROTATION,
)
_ROTARY_AXES = (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION)
_LINEAR_AXES = (AxisName.SLIDE, AxisName.Z)


class AxisCaptureError(RuntimeError):
    """当前轴状态不安全、不完整或在采样窗口内不稳定。"""


StateReader = Callable[[tuple[AxisName, ...]], tuple[AxisState, ...]]


def capture_stable_axis_state(
    state_reader: StateReader,
    *,
    samples: int = 20,
    sample_interval_s: float = 0.05,
    max_linear_drift_mm: float = 0.1,
    max_rotary_drift_deg: float = 0.1,
    require_slide_z_zero: bool = False,
    slide_zero_tolerance_mm: float = 0.5,
    z_zero_tolerance_mm: float = 0.5,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> RobotAxisState:
    """连续读取五轴；不执行 home、move、stop、enable 或 torque enable。"""

    if not callable(state_reader):
        raise TypeError("state_reader must be callable")
    if not isinstance(samples, int) or isinstance(samples, bool) or samples < 2:
        raise ValueError("samples must be an integer of at least 2")
    _require_nonnegative("sample_interval_s", sample_interval_s)
    _require_positive("max_linear_drift_mm", max_linear_drift_mm)
    _require_positive("max_rotary_drift_deg", max_rotary_drift_deg)
    _require_nonnegative("slide_zero_tolerance_mm", slide_zero_tolerance_mm)
    _require_nonnegative("z_zero_tolerance_mm", z_zero_tolerance_mm)
    if not isinstance(require_slide_z_zero, bool):
        raise TypeError("require_slide_z_zero must be a bool")

    collected: dict[AxisName, list[float]] = {
        axis: [] for axis in ALL_AXIS_NAMES
    }
    unknown_busy_axes: set[AxisName] = set()
    next_sample_time = monotonic()
    for sample_index in range(samples):
        if sample_index:
            next_sample_time += sample_interval_s
            delay = next_sample_time - monotonic()
            if delay > 0:
                sleep(delay)
        states = state_reader(ALL_AXIS_NAMES)
        state_by_axis = _validate_snapshot(states)
        for axis, state in state_by_axis.items():
            if state.busy is None:
                unknown_busy_axes.add(axis)
            assert state.current_position is not None
            collected[axis].append(float(state.current_position))

        if require_slide_z_zero:
            _require_near_zero(
                state_by_axis[AxisName.SLIDE],
                slide_zero_tolerance_mm,
            )
            _require_near_zero(
                state_by_axis[AxisName.Z],
                z_zero_tolerance_mm,
            )

    if unknown_busy_axes and samples < 3:
        rendered = ", ".join(axis.value for axis in sorted(unknown_busy_axes, key=str))
        raise AxisCaptureError(
            f"busy is unknown for {rendered}; at least 3 stable samples are required"
        )

    for axis in _LINEAR_AXES:
        drift = max(collected[axis]) - min(collected[axis])
        if drift > max_linear_drift_mm:
            raise AxisCaptureError(
                f"axis {axis.value} is unstable: drift {drift:.6f} mm exceeds "
                f"{max_linear_drift_mm:.6f} mm"
            )
    for axis in _ROTARY_AXES:
        drift = _maximum_circular_deviation_deg(collected[axis])
        if drift > max_rotary_drift_deg:
            raise AxisCaptureError(
                f"axis {axis.value} is unstable: circular drift {drift:.6f} deg "
                f"exceeds {max_rotary_drift_deg:.6f} deg"
            )

    return RobotAxisState(
        slide_mm=sum(collected[AxisName.SLIDE]) / samples,
        z_mm=sum(collected[AxisName.Z]) / samples,
        shoulder_deg=_circular_mean_deg(collected[AxisName.SHOULDER]),
        elbow_deg=_circular_mean_deg(collected[AxisName.ELBOW]),
        rotation_deg=_circular_mean_deg(collected[AxisName.ROTATION]),
    )


def initialize_read_only_rotary_positions(runtime: object) -> None:
    """用现有只读 absolute-position 初始化肩肘；不发送控制写入。"""

    for attribute in ("shoulder_joint", "elbow_joint"):
        joint = getattr(runtime, attribute, None)
        if joint is None:
            continue
        initializer = getattr(joint, "initialize", None)
        if not callable(initializer):
            raise AxisCaptureError(f"runtime.{attribute} has no initialize()")
        initializer()


def _validate_snapshot(
    states: tuple[AxisState, ...],
) -> dict[AxisName, AxisState]:
    if not isinstance(states, tuple):
        raise AxisCaptureError("state reader must return a tuple")
    state_by_axis: dict[AxisName, AxisState] = {}
    for state in states:
        if not isinstance(state, AxisState):
            raise AxisCaptureError("state reader returned a non-AxisState value")
        if state.axis in state_by_axis:
            raise AxisCaptureError(f"duplicate state for axis {state.axis.value}")
        state_by_axis[state.axis] = state
    missing = tuple(axis for axis in ALL_AXIS_NAMES if axis not in state_by_axis)
    if missing:
        raise AxisCaptureError(
            "missing axis states: " + ", ".join(axis.value for axis in missing)
        )

    for axis in ALL_AXIS_NAMES:
        state = state_by_axis[axis]
        if not isinstance(state.connected, bool):
            raise AxisCaptureError(f"axis {axis.value} connected flag is invalid")
        if state.busy not in (True, False, None) or (
            state.busy is not None and not isinstance(state.busy, bool)
        ):
            raise AxisCaptureError(f"axis {axis.value} busy flag is invalid")
        if not state.connected:
            raise AxisCaptureError(f"axis {axis.value} is not connected")
        if state.faulted:
            raise AxisCaptureError(
                f"axis {axis.value} is faulted: "
                f"{state.fault_code!r} {state.fault_message or ''}".rstrip()
            )
        if state.busy is True:
            raise AxisCaptureError(f"axis {axis.value} is busy")
        if not state.position_valid or state.current_position is None:
            raise AxisCaptureError(f"axis {axis.value} position is not valid")
        if isinstance(state.current_position, bool) or not isinstance(
            state.current_position,
            (int, float),
        ):
            raise AxisCaptureError(f"axis {axis.value} position is not numeric")
        if not math.isfinite(state.current_position):
            raise AxisCaptureError(f"axis {axis.value} position is not finite")
        expected_unit = "mm" if axis in _LINEAR_AXES else "deg"
        if state.position_unit != expected_unit:
            raise AxisCaptureError(
                f"axis {axis.value} unit must be {expected_unit}, "
                f"got {state.position_unit!r}"
            )
        if axis in _LINEAR_AXES and state.homed is not True:
            raise AxisCaptureError(f"axis {axis.value} is not homed")
    return state_by_axis


def _require_near_zero(state: AxisState, tolerance_mm: float) -> None:
    assert state.current_position is not None
    if abs(state.current_position) > tolerance_mm:
        raise AxisCaptureError(
            f"axis {state.axis.value} position {state.current_position:.6f} mm "
            f"is outside zero tolerance ±{tolerance_mm:.6f} mm"
        )


def _circular_mean_deg(values: list[float]) -> float:
    sine = sum(math.sin(math.radians(value)) for value in values)
    cosine = sum(math.cos(math.radians(value)) for value in values)
    magnitude = math.hypot(sine, cosine)
    if magnitude <= 1e-12:
        raise AxisCaptureError("circular angle samples have no unique mean")
    result = math.degrees(math.atan2(sine, cosine))
    return 0.0 if math.isclose(result, 0.0, abs_tol=1e-12) else result


def _maximum_circular_deviation_deg(values: list[float]) -> float:
    reference = _circular_mean_deg(values)
    return max(abs(_angle_difference_deg(value, reference)) for value in values)


def _angle_difference_deg(value: float, reference: float) -> float:
    return (value - reference + 180.0) % 360.0 - 180.0


def _require_nonnegative(name: str, value: object) -> float:
    converted = _finite_real(name, value)
    if converted < 0:
        raise ValueError(f"{name} must be non-negative")
    return converted


def _require_positive(name: str, value: object) -> float:
    converted = _finite_real(name, value)
    if converted <= 0:
        raise ValueError(f"{name} must be positive")
    return converted


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = [
    "ALL_AXIS_NAMES",
    "AxisCaptureError",
    "capture_stable_axis_state",
    "initialize_read_only_rotary_positions",
]

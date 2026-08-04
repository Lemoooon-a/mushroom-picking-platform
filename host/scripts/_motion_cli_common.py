"""上层运动人工 CLI 共享的小型安全辅助函数。

本模块导入时只定义函数；不会加载本机配置、枚举设备、创建 runtime 或发送命令。
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
import math

from bootstrap import UpperMotionRuntime, create_upper_motion_runtime
from config.hardware import load_local_hardware_config
from config.motion_runtime import load_local_motion_config
from motion.authorization import RuntimeMode
from motion.unified_protocol import (
    AxisDescriptor,
    AxisName,
    AxisState,
    MotionCommandResult,
    MultiAxisCommandResult,
)


_CAN_AXES = frozenset((AxisName.SHOULDER, AxisName.ELBOW))


def create_configured_runtime(
    mode: RuntimeMode,
    *,
    allow_unverified_rotation_motion: bool = False,
) -> UpperMotionRuntime:
    """显式调用时才加载本机配置、枚举设备并组装唯一 runtime。"""

    return create_upper_motion_runtime(
        load_local_hardware_config(),
        load_local_motion_config(),
        mode=mode,
        allow_unverified_rotation_motion=allow_unverified_rotation_motion,
    )


def initialize_read_only_rotary_positions(
    runtime: UpperMotionRuntime | object,
    axes: Iterable[AxisName],
) -> None:
    """建立肩肘进程内绝对位置解释；这不是 enable、home 或 motion。"""

    selected = frozenset(axes)
    if AxisName.SHOULDER in selected:
        runtime.shoulder_joint.initialize()
    if AxisName.ELBOW in selected:
        runtime.elbow_joint.initialize()


def format_axis_descriptor(descriptor: AxisDescriptor | object) -> str:
    capabilities = descriptor.capabilities
    return (
        f"axis={descriptor.name.value} kind={descriptor.kind.value} "
        f"unit={descriptor.position_unit} "
        f"limits=[{descriptor.minimum_position:.6f}, "
        f"{descriptor.maximum_position:.6f}] "
        "capabilities=("
        f"query={capabilities.query_state}, "
        f"move_absolute={capabilities.move_absolute}, "
        f"stop={capabilities.stop}, "
        f"home={capabilities.reference_home}, "
        f"velocity={capabilities.configurable_velocity}, "
        f"acceleration={capabilities.configurable_acceleration}, "
        f"arrival={capabilities.arrival_confirmation})"
    )


def format_axis_state(state: AxisState) -> str:
    position = (
        "unknown"
        if state.current_position is None
        else f"{state.current_position:.6f} {state.position_unit}"
    )
    return (
        f"axis={state.axis.value} connected={state.connected} "
        f"enabled={state.enabled} busy={state.busy} homed={state.homed} "
        f"position_valid={state.position_valid} position={position} "
        f"faulted={state.faulted} fault_code={state.fault_code} "
        f"fault_message={state.fault_message!r}"
    )


def format_command_result(result: MotionCommandResult) -> str:
    error_code = result.error_code.value if result.error_code is not None else None
    return (
        f"axis={result.axis.value} status={result.status.value} "
        f"accepted={result.accepted} completed={result.completed} "
        f"target={result.target_position:.6f} "
        f"final={result.final_position} error={result.position_error} "
        f"error_code={error_code} message={result.message!r}"
    )


def format_group_result(result: MultiAxisCommandResult) -> tuple[str, ...]:
    lines = (
        f"group status={result.status.value} accepted={result.accepted} "
        f"completed={result.completed} message={result.message!r}",
    )
    return lines + tuple(f"  {format_command_result(item)}" for item in result.results)


def motion_state_blockers(axis: AxisName, state: AxisState) -> tuple[str, ...]:
    """返回绝对位置调试入口共用的保守状态阻断原因。"""

    reasons: list[str] = []
    if not state.connected:
        reasons.append("backend is not connected")
    if state.busy is not False:
        reasons.append("busy is not confirmed false")
    if state.faulted:
        reasons.append(f"fault_code={state.fault_code!r}: {state.fault_message or 'fault'}")
    if not state.position_valid or state.current_position is None:
        reasons.append("current position is not valid")
    if axis in (AxisName.SLIDE, AxisName.Z) and state.homed is not True:
        reasons.append("reference home is not confirmed")
    if axis in _CAN_AXES and state.enabled is not True:
        reasons.append("motor enabled state is not confirmed")
    return tuple(reasons)


def prepare_rotation_power(
    runtime: UpperMotionRuntime | object,
    state: AxisState,
    *,
    confirm_no_independent_stop: bool,
    confirm_torque_enable: bool,
    emit: Callable[[str], None] = print,
) -> None:
    """显式执行 Rotation 当前位置预装和 torque enable。

    这是 power preparation，不是普通位置提交，也不是 stop/disable。调用方必须先完成
    统一目标校验，并显式确认 Rotation 没有可靠独立 stop 以及 torque enable 风险。
    """

    if state.axis is not AxisName.ROTATION:
        raise ValueError("Rotation power preparation requires Rotation state")
    if not confirm_no_independent_stop or not confirm_torque_enable:
        raise ValueError(
            "Rotation power preparation requires explicit no-stop and torque-enable confirmation"
        )
    if not state.position_valid or state.current_position is None:
        raise ValueError("Rotation current position must be valid before torque enable")
    current_rad = math.radians(state.current_position)
    runtime.rotation_axis.command_position(
        current_rad,
        runtime.rotation_axis.config.max_speed_raw,
    )
    runtime.rotation_axis.enable_torque()
    emit(
        "Rotation power preparation completed: current-position goal was preloaded "
        "before explicit torque enable. Torque remains enabled after this command."
    )


def best_effort_stop_axes_once(
    runtime: UpperMotionRuntime | object,
    axes: Iterable[AxisName],
    *,
    emit: Callable[[str], None] = print,
) -> None:
    """对每个可 stop 轴最多调用一次统一 software stop，并吞掉 stop 自身异常。"""

    attempted: set[AxisName] = set()
    for axis in axes:
        if axis in attempted:
            continue
        attempted.add(axis)
        descriptor = runtime.controller.describe_axis(axis)
        if not descriptor.capabilities.stop:
            emit(
                f"axis={axis.value}: no verified independent software stop; "
                "no stop command was sent"
            )
            continue
        try:
            result = runtime.controller.stop(axis)
        except Exception as exc:
            emit(f"axis={axis.value}: best-effort software stop failed: {exc}")
        else:
            emit(f"best-effort software stop: {format_command_result(result)}")


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise ValueError("value must be finite and greater than zero")
    return parsed


__all__ = [
    "best_effort_stop_axes_once",
    "create_configured_runtime",
    "format_axis_descriptor",
    "format_axis_state",
    "format_command_result",
    "format_group_result",
    "initialize_read_only_rotary_positions",
    "motion_state_blockers",
    "positive_float",
    "prepare_rotation_power",
]

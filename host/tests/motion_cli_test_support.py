"""上层运动 CLI 测试共享的纯 MagicMock/DTO 夹具。"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
)


def descriptor(axis: AxisName) -> AxisDescriptor:
    linear = axis in (AxisName.SLIDE, AxisName.Z)
    return AxisDescriptor(
        name=axis,
        display_name=axis.value.title(),
        kind=AxisKind.LINEAR if linear else AxisKind.ROTARY,
        position_unit="mm" if linear else "deg",
        velocity_unit="mm/s" if linear else "deg/s",
        acceleration_unit="mm/s²" if linear else "deg/s²",
        minimum_position=0.0 if linear else -180.0,
        maximum_position=100.0 if linear else 180.0,
        capabilities=AxisCapabilities(
            query_state=True,
            move_absolute=True,
            stop=axis is not AxisName.ROTATION,
            reference_home=linear,
            configurable_velocity=axis is not AxisName.ROTATION,
            configurable_acceleration=linear,
            arrival_confirmation=True,
        ),
    )


def axis_state(
    axis: AxisName,
    *,
    position: float = 0.0,
    healthy: bool = True,
) -> AxisState:
    linear = axis in (AxisName.SLIDE, AxisName.Z)
    can_axis = axis in (AxisName.SHOULDER, AxisName.ELBOW)
    return AxisState(
        axis=axis,
        connected=True,
        enabled=True if can_axis else False if linear else None,
        busy=False,
        homed=True if linear else None,
        position_valid=healthy,
        current_position=position if healthy else None,
        position_unit="mm" if linear else "deg",
        faulted=not healthy,
        fault_code=None if healthy else 3,
        fault_message=None if healthy else "test fault",
    )


def command_result(
    axis: AxisName,
    status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
    *,
    target: float = 1.0,
) -> MotionCommandResult:
    accepted, completed, error = {
        MotionCommandStatus.ARRIVED: (True, True, None),
        MotionCommandStatus.TIMEOUT: (True, False, MotionErrorCode.TIMEOUT),
        MotionCommandStatus.FAULT: (True, False, MotionErrorCode.DEVICE_FAULT),
        MotionCommandStatus.ABORTED: (True, False, MotionErrorCode.BACKEND_ERROR),
        MotionCommandStatus.REJECTED: (False, False, MotionErrorCode.BACKEND_ERROR),
    }[status]
    return MotionCommandResult(
        command_id=f"command-{axis.value}",
        axis=axis,
        status=status,
        accepted=accepted,
        completed=completed,
        target_position=target,
        final_position=target if status is MotionCommandStatus.ARRIVED else None,
        position_error=0.0 if status is MotionCommandStatus.ARRIVED else None,
        error_code=error,
        message=status.value,
    )


def group_result(
    target: MultiAxisTarget,
    status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
) -> MultiAxisCommandResult:
    results = tuple(
        command_result(item.axis, status, target=item.position)
        for item in target.targets
    )
    accepted = status is not MotionCommandStatus.REJECTED
    completed = True if status is MotionCommandStatus.ARRIVED else False
    return MultiAxisCommandResult(
        group_id="group",
        status=status,
        results=results,
        accepted=accepted,
        completed=completed,
        message=status.value,
    )


def fake_runtime() -> MagicMock:
    runtime = MagicMock()
    runtime.__enter__.return_value = runtime
    runtime.__exit__.return_value = None
    runtime.stm32_client.version.return_value = SimpleNamespace(
        protocol_version="1",
        firmware_version="test",
    )
    runtime.rotation_axis.config.max_speed_raw = 100
    runtime.controller.list_axes.return_value = tuple(descriptor(axis) for axis in AxisName)
    runtime.controller.describe_axis.side_effect = descriptor
    runtime.controller.get_state.side_effect = axis_state
    runtime.controller.get_axis_states.side_effect = (
        lambda axes=None: tuple(axis_state(axis) for axis in (tuple(AxisName) if axes is None else axes))
    )
    runtime.controller.submit_absolute.side_effect = (
        lambda target: MotionCommandHandle("single", target.axis, target.position)
    )
    runtime.controller.wait.side_effect = (
        lambda handle, timeout_s=None: command_result(
            handle.axis,
            target=handle.target_position,
        )
    )

    def submit_group(target: MultiAxisTarget) -> MultiAxisCommandHandle:
        return MultiAxisCommandHandle(
            "group",
            tuple(
                MotionCommandHandle(f"cmd-{item.axis.value}", item.axis, item.position)
                for item in target.targets
            ),
        )

    runtime.controller.submit_positions.side_effect = submit_group

    def wait_group(handle: MultiAxisCommandHandle, timeout_s=None) -> MultiAxisCommandResult:
        target = MultiAxisTarget(
            tuple(
                AxisTarget(item.axis, item.target_position)
                for item in handle.commands
            )
        )
        return group_result(target)

    runtime.controller.wait_group.side_effect = wait_group
    runtime.controller.stop.side_effect = (
        lambda axis: command_result(axis, MotionCommandStatus.ABORTED, target=0.0)
    )
    runtime.controller.home_reference.side_effect = (
        lambda axis, timeout_s=None: command_result(
            axis,
            MotionCommandStatus.ARRIVED,
            target=0.0,
        )
    )
    return runtime


__all__ = [
    "axis_state",
    "command_result",
    "descriptor",
    "fake_runtime",
    "group_result",
]

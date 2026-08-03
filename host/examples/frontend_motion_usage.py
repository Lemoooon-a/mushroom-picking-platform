"""Runnable fake-only example for ``FrontendMotionInterface`` consumers."""

from __future__ import annotations

from typing import cast
from unittest.mock import Mock

from motion.client_interfaces import FrontendMotionInterface
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


def poll_command_for_gui(
    motion: FrontendMotionInterface,
    handle: MotionCommandHandle,
) -> MotionCommandResult:
    """One non-blocking GUI timer tick; schedule another tick if unfinished."""

    return motion.get_command_result(handle)


def demonstrate_frontend(motion: FrontendMotionInterface) -> None:
    descriptors = motion.list_axes()
    for descriptor in descriptors:
        print(
            descriptor.name.value,
            descriptor.position_unit,
            descriptor.minimum_position,
            descriptor.maximum_position,
        )

    shoulder = motion.describe_axis(AxisName.SHOULDER)
    state = motion.get_state(AxisName.SHOULDER)
    print("shoulder:", shoulder.position_unit, state.current_position)

    handle = motion.submit_absolute(
        AxisTarget(AxisName.SHOULDER, position=25.0, velocity=5.0)
    )
    result = poll_command_for_gui(motion, handle)
    print("single-axis status:", result.status.value)

    group_handle = motion.submit_positions(
        MultiAxisTarget(
            (
                AxisTarget(AxisName.SLIDE, position=300.0),
                AxisTarget(AxisName.Z, position=120.0),
            )
        )
    )
    print("group status:", motion.get_group_result(group_handle).status.value)

    for axis in (AxisName.SLIDE, AxisName.Z):
        print("home", axis.value, motion.home_reference(axis, timeout_s=30.0).status.value)

    unsupported = motion.stop(AxisName.ROTATION)
    if unsupported.status == MotionCommandStatus.REJECTED:
        print("rotation stop unavailable:", unsupported.message)


def _command_result(
    handle: MotionCommandHandle,
    status: MotionCommandStatus,
) -> MotionCommandResult:
    completed = True if status == MotionCommandStatus.ARRIVED else None
    return MotionCommandResult(
        command_id=handle.command_id,
        axis=handle.axis,
        status=status,
        accepted=True,
        completed=completed,
        target_position=handle.target_position,
        final_position=(handle.target_position if completed else None),
        position_error=(0.0 if completed else None),
        error_code=None,
        message=status.value,
    )


def _build_fake_motion() -> FrontendMotionInterface:
    """Build an in-memory mock; no controller, port, bus, or hardware is opened."""

    motion = Mock(spec=FrontendMotionInterface)
    linear_caps = AxisCapabilities(True, True, True, True, True, True, True)
    rotary_caps = AxisCapabilities(True, True, True, False, True, False, True)
    descriptors = (
        AxisDescriptor(
            AxisName.SLIDE,
            "Slide",
            AxisKind.LINEAR,
            "mm",
            "mm/s",
            "mm/s²",
            0.0,
            800.0,
            linear_caps,
        ),
        AxisDescriptor(
            AxisName.SHOULDER,
            "Shoulder",
            AxisKind.ROTARY,
            "deg",
            "deg/s",
            "deg/s²",
            -60.0,
            70.0,
            rotary_caps,
        ),
    )
    shoulder_state = AxisState(
        AxisName.SHOULDER,
        True,
        True,
        False,
        None,
        True,
        10.0,
        "deg",
        False,
        None,
        None,
    )
    command_handle = MotionCommandHandle("fake-command", AxisName.SHOULDER, 25.0)
    group_handle = MultiAxisCommandHandle(
        "fake-group",
        (
            MotionCommandHandle("fake-slide", AxisName.SLIDE, 300.0),
            MotionCommandHandle("fake-z", AxisName.Z, 120.0),
        ),
    )
    group_results = tuple(
        _command_result(item, MotionCommandStatus.ACCEPTED)
        for item in group_handle.commands
    )
    motion.list_axes.return_value = descriptors
    motion.describe_axis.return_value = descriptors[1]
    motion.get_state.return_value = shoulder_state
    motion.submit_absolute.return_value = command_handle
    motion.get_command_result.return_value = _command_result(
        command_handle,
        MotionCommandStatus.ACCEPTED,
    )
    motion.submit_positions.return_value = group_handle
    motion.get_group_result.return_value = MultiAxisCommandResult(
        "fake-group",
        MotionCommandStatus.ACCEPTED,
        group_results,
        True,
        None,
        "fake group accepted",
    )
    motion.home_reference.side_effect = lambda axis, **_kwargs: MotionCommandResult(
        f"fake-home-{axis.value}",
        axis,
        MotionCommandStatus.ARRIVED,
        True,
        True,
        0.0,
        0.0,
        0.0,
        None,
        "fake reference home arrived",
    )
    motion.stop.return_value = MotionCommandResult(
        "fake-stop",
        AxisName.ROTATION,
        MotionCommandStatus.REJECTED,
        False,
        False,
        0.0,
        None,
        None,
        MotionErrorCode.UNSUPPORTED_COMMAND,
        "axis rotation has no independent stop command",
    )
    return cast(FrontendMotionInterface, motion)


if __name__ == "__main__":
    demonstrate_frontend(_build_fake_motion())

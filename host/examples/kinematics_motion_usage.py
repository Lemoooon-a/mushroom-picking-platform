"""Runnable fake-only example for ``KinematicsMotionInterface`` consumers."""

from __future__ import annotations

from typing import cast
from unittest.mock import Mock

from motion.client_interfaces import KinematicsMotionInterface
from motion.unified_protocol import (
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
)


def execute_solution(
    motion: KinematicsMotionInterface,
    target: MultiAxisTarget,
) -> MultiAxisCommandResult:
    handle = motion.submit_positions(target)
    return motion.wait_group(handle, timeout_s=10.0)


def demonstrate_kinematics(motion: KinematicsMotionInterface) -> None:
    axes = (
        AxisName.SLIDE,
        AxisName.Z,
        AxisName.SHOULDER,
        AxisName.ELBOW,
        AxisName.ROTATION,
    )
    states = motion.get_axis_states(axes)
    print("current logical positions:", [state.current_position for state in states])
    target = MultiAxisTarget(
        (
            AxisTarget(AxisName.SLIDE, position=300.0),
            AxisTarget(AxisName.Z, position=120.0),
            AxisTarget(AxisName.SHOULDER, position=25.0),
            AxisTarget(AxisName.ELBOW, position=-60.0),
            AxisTarget(AxisName.ROTATION, position=30.0),
        )
    )
    result = execute_solution(motion, target)
    print("group status:", result.status.value)


def _build_fake_motion() -> KinematicsMotionInterface:
    """Build an in-memory mock; no configuration or hardware is accessed."""

    motion = Mock(spec=KinematicsMotionInterface)
    axes = tuple(AxisName)
    motion.get_axis_states.return_value = tuple(
        AxisState(
            axis,
            True,
            True,
            False,
            True if axis in (AxisName.SLIDE, AxisName.Z) else None,
            True,
            0.0,
            "mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
            False,
            None,
            None,
        )
        for axis in axes
    )
    commands = tuple(
        MotionCommandHandle(f"fake-{axis.value}", axis, 0.0) for axis in axes
    )
    handle = MultiAxisCommandHandle("fake-group", commands)
    results = tuple(
        MotionCommandResult(
            item.command_id,
            item.axis,
            MotionCommandStatus.ARRIVED,
            True,
            True,
            item.target_position,
            item.target_position,
            0.0,
            None,
            "fake arrival",
        )
        for item in commands
    )
    final = MultiAxisCommandResult(
        handle.group_id,
        MotionCommandStatus.ARRIVED,
        results,
        True,
        True,
        "all fake axes arrived",
    )
    motion.submit_positions.return_value = handle
    motion.get_group_result.return_value = final
    motion.wait_group.return_value = final
    return cast(KinematicsMotionInterface, motion)


if __name__ == "__main__":
    demonstrate_kinematics(_build_fake_motion())

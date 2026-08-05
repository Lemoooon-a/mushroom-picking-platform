"""Stable in-process client boundaries for unified motion control.

The protocol types intentionally reuse the public DTOs from
``motion.unified_protocol`` and do not import hardware transports or adapters.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from motion.unified_protocol import (
    AxisDescriptor,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
    RotaryJointEnableStatus,
)
from motion.suction import SuctionStatus


@runtime_checkable
class FrontendMotionInterface(Protocol):
    """Stable motion surface intended for the frontend integration."""

    def list_axes(self) -> tuple[AxisDescriptor, ...]: ...

    def describe_axis(self, axis: AxisName) -> AxisDescriptor: ...

    def get_state(self, axis: AxisName) -> AxisState: ...

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]: ...

    def submit_absolute(self, target: AxisTarget) -> MotionCommandHandle: ...

    def submit_positions(
        self,
        target: MultiAxisTarget,
    ) -> MultiAxisCommandHandle: ...

    def get_command_result(
        self,
        handle: MotionCommandHandle,
    ) -> MotionCommandResult: ...

    def get_group_result(
        self,
        handle: MultiAxisCommandHandle,
    ) -> MultiAxisCommandResult: ...

    def stop(self, axis: AxisName) -> MotionCommandResult: ...

    def home_reference(
        self,
        axis: AxisName,
        *,
        timeout_s: float | None = None,
    ) -> MotionCommandResult: ...

    def suction_grip(self) -> SuctionStatus: ...

    def suction_release(self) -> SuctionStatus: ...

    def suction_idle(self) -> SuctionStatus: ...

    def get_suction_status(self) -> SuctionStatus: ...

    def enable_rotary_joints(self) -> RotaryJointEnableStatus: ...

    def disable_rotary_joints(self) -> RotaryJointEnableStatus: ...

    def rotary_joints_enabled(self) -> bool: ...

    def get_rotary_joint_enable_status(self) -> RotaryJointEnableStatus: ...


@runtime_checkable
class KinematicsMotionInterface(Protocol):
    """Narrow motion surface intended for kinematics execution."""

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...],
    ) -> tuple[AxisState, ...]: ...

    def submit_positions(
        self,
        target: MultiAxisTarget,
    ) -> MultiAxisCommandHandle: ...

    def get_group_result(
        self,
        handle: MultiAxisCommandHandle,
    ) -> MultiAxisCommandResult: ...

    def wait_group(
        self,
        handle: MultiAxisCommandHandle,
        *,
        timeout_s: float | None = None,
    ) -> MultiAxisCommandResult: ...


__all__ = ["FrontendMotionInterface", "KinematicsMotionInterface"]

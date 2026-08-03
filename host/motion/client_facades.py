"""Thin client façades over one shared unified motion controller."""

from __future__ import annotations

from typing import TYPE_CHECKING

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
)

if TYPE_CHECKING:
    from motion.unified_controller import UnifiedMotionController


class FrontendMotionFacade:
    """Expose only the unified controller operations approved for the UI."""

    def __init__(self, controller: UnifiedMotionController) -> None:
        self._controller = controller

    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        return self._controller.list_axes()

    def describe_axis(self, axis: AxisName) -> AxisDescriptor:
        return self._controller.describe_axis(axis)

    def get_state(self, axis: AxisName) -> AxisState:
        return self._controller.get_state(axis)

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]:
        return self._controller.get_axis_states(axes)

    def submit_absolute(self, target: AxisTarget) -> MotionCommandHandle:
        return self._controller.submit_absolute(target)

    def submit_positions(
        self,
        target: MultiAxisTarget,
    ) -> MultiAxisCommandHandle:
        return self._controller.submit_positions(target)

    def get_command_result(
        self,
        handle: MotionCommandHandle,
    ) -> MotionCommandResult:
        return self._controller.get_command_result(handle)

    def get_group_result(
        self,
        handle: MultiAxisCommandHandle,
    ) -> MultiAxisCommandResult:
        return self._controller.get_group_result(handle)

    def stop(self, axis: AxisName) -> MotionCommandResult:
        return self._controller.stop(axis)

    def home_reference(
        self,
        axis: AxisName,
        *,
        timeout_s: float | None = None,
    ) -> MotionCommandResult:
        return self._controller.home_reference(axis, timeout_s=timeout_s)


class KinematicsMotionFacade:
    """Expose only multi-axis execution operations to kinematics code."""

    def __init__(self, controller: UnifiedMotionController) -> None:
        self._controller = controller

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...],
    ) -> tuple[AxisState, ...]:
        return self._controller.get_axis_states(axes)

    def submit_positions(
        self,
        target: MultiAxisTarget,
    ) -> MultiAxisCommandHandle:
        return self._controller.submit_positions(target)

    def get_group_result(
        self,
        handle: MultiAxisCommandHandle,
    ) -> MultiAxisCommandResult:
        return self._controller.get_group_result(handle)

    def wait_group(
        self,
        handle: MultiAxisCommandHandle,
        *,
        timeout_s: float | None = None,
    ) -> MultiAxisCommandResult:
        return self._controller.wait_group(handle, timeout_s=timeout_s)


__all__ = ["FrontendMotionFacade", "KinematicsMotionFacade"]

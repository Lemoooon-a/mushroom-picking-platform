"""Robot Service 的内部依赖倒置端口；不是应用层公开 façade。"""

from __future__ import annotations

from typing import Protocol

from motion.unified_protocol import (
    AxisDescriptor,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    RelativeAxisTarget,
)


class _AxisMotionPort(Protocol):
    def list_axes(self) -> tuple[AxisDescriptor, ...]: ...

    def get_state(self, axis: AxisName) -> AxisState: ...

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]: ...

    def submit_absolute(self, target: AxisTarget) -> MotionCommandHandle: ...

    def submit_relative(self, target: RelativeAxisTarget) -> MotionCommandHandle: ...

    def wait(
        self,
        handle: MotionCommandHandle,
        *,
        timeout_s: float | None = None,
    ) -> MotionCommandResult: ...

    def stop(self, axis: AxisName) -> MotionCommandResult: ...


__all__: list[str] = []

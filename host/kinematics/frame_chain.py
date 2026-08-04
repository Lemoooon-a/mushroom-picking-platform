"""以 Base 为公开根、以 Slide-zero 为内部根的坐标链。"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

import numpy as np

from geometry.rigid_transform import RigidTransform


class FrameChainError(RuntimeError):
    """Frame chain 配置或运动学提供者不满足契约。"""


class MissingToolCameraTransformError(FrameChainError):
    """调用 Camera 相关 API 时尚未录入 ``tool_T_camera``。"""


@dataclass(frozen=True)
class RobotAxisState:
    """纯运动学五轴逻辑状态；直线单位 mm，旋转单位 deg。"""

    slide_mm: float
    z_mm: float
    shoulder_deg: float
    elbow_deg: float
    rotation_deg: float

    def __post_init__(self) -> None:
        for field_name in (
            "slide_mm",
            "z_mm",
            "shoulder_deg",
            "elbow_deg",
            "rotation_deg",
        ):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{field_name} must be a finite real number")
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")


@runtime_checkable
class SlideZeroKinematics(Protocol):
    """现有或后续完整 FK 必须实现的最小无硬件接口。"""

    def forward_kinematics(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform: ...


class RobotFrameChain:
    """集中组合固定变换；构造和调用都不查询硬件。"""

    def __init__(
        self,
        *,
        base_T_slide_zero: RigidTransform,
        tool_T_camera: RigidTransform | None,
        slide_zero_kinematics: SlideZeroKinematics,
    ) -> None:
        if not isinstance(base_T_slide_zero, RigidTransform):
            raise TypeError("base_T_slide_zero must be a RigidTransform")
        if tool_T_camera is not None and not isinstance(
            tool_T_camera, RigidTransform
        ):
            raise TypeError("tool_T_camera must be a RigidTransform or None")
        if not isinstance(slide_zero_kinematics, SlideZeroKinematics):
            raise TypeError(
                "slide_zero_kinematics must implement "
                "forward_kinematics(RobotAxisState)"
            )
        self.base_T_slide_zero = base_T_slide_zero
        self.tool_T_camera = tool_T_camera
        self.slide_zero_kinematics = slide_zero_kinematics

    def slide_zero_T_tool(self, axis_state: RobotAxisState) -> RigidTransform:
        if not isinstance(axis_state, RobotAxisState):
            raise TypeError("axis_state must be RobotAxisState")
        transform = self.slide_zero_kinematics.forward_kinematics(axis_state)
        if not isinstance(transform, RigidTransform):
            raise FrameChainError(
                "slide-zero kinematics returned a non-RigidTransform value"
            )
        return transform

    def base_T_tool(self, axis_state: RobotAxisState) -> RigidTransform:
        return self.base_T_slide_zero @ self.slide_zero_T_tool(axis_state)

    def forward_kinematics_base(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform:
        """Base 根坐标下的公开 FK 薄包装。"""

        return self.base_T_tool(axis_state)

    def base_T_camera(self, axis_state: RobotAxisState) -> RigidTransform:
        if self.tool_T_camera is None:
            raise MissingToolCameraTransformError(
                "tool_T_camera is not configured; Camera transform is unavailable"
            )
        return self.base_T_tool(axis_state) @ self.tool_T_camera

    def transform_camera_point_to_base(
        self,
        point_camera_mm: Sequence[float],
        axis_state: RobotAxisState,
    ) -> np.ndarray:
        return self.base_T_camera(axis_state).transform_point(point_camera_mm)

    def transform_base_target_to_slide_zero(
        self,
        base_T_target: RigidTransform,
    ) -> RigidTransform:
        if not isinstance(base_T_target, RigidTransform):
            raise TypeError("base_T_target must be a RigidTransform")
        return self.base_T_slide_zero.inverse() @ base_T_target


__all__ = [
    "FrameChainError",
    "MissingToolCameraTransformError",
    "RobotAxisState",
    "RobotFrameChain",
    "SlideZeroKinematics",
]

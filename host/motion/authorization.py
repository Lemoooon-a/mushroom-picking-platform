"""统一运动控制的显式运行模式与安全授权。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from motion.unified_protocol import AxisName


class RuntimeMode(str, Enum):
    """Runtime 是否允许显式运动调用通过门禁。"""

    READ_ONLY = "read_only"
    MOTION = "motion"


class MotionAuthorizationError(RuntimeError):
    """当前 Runtime 授权不允许所请求的运动。"""


class RotationMotionAuthorizationError(MotionAuthorizationError):
    """末端旋转轴缺少额外的无可靠 stop 风险授权。"""


@dataclass(frozen=True)
class MotionAuthorization:
    """无副作用的运动门禁；默认拒绝全部运动。"""

    mode: RuntimeMode = RuntimeMode.READ_ONLY
    allow_unverified_rotation_motion: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.mode, RuntimeMode):
            raise TypeError("mode must be a RuntimeMode")
        if not isinstance(self.allow_unverified_rotation_motion, bool):
            raise TypeError("allow_unverified_rotation_motion must be a bool")

    def require_motion(self) -> None:
        if self.mode is not RuntimeMode.MOTION:
            raise MotionAuthorizationError(
                "motion is disabled because the runtime is in READ_ONLY mode"
            )

    def require_axis_motion(self, axis: AxisName) -> None:
        self.require_motion()
        if not isinstance(axis, AxisName):
            raise TypeError("axis must be an AxisName")
        if (
            axis is AxisName.ROTATION
            and not self.allow_unverified_rotation_motion
        ):
            raise RotationMotionAuthorizationError(
                "Rotation motion is disabled because the backend has no "
                "verified independent stop. Set "
                "allow_unverified_rotation_motion=True only after explicit "
                "risk acceptance."
            )


__all__ = [
    "MotionAuthorization",
    "MotionAuthorizationError",
    "RotationMotionAuthorizationError",
    "RuntimeMode",
]

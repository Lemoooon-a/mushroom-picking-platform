"""机器人启动与中间安全阶段的软件策略配置。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math


WORKING_HEIGHT_BASE_Z_MM = 200.0


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


@dataclass(frozen=True)
class StartupSafePoseConfig:
    """专用于 Homing 后 startup/return 的固定安全姿态策略。"""

    base_x_mm: float = 400.0
    base_y_mm: float = 150.0
    tool_yaw_deg: float = 0.0
    slide_mm: float = 0.0
    z_axis_mm: float = 0.0

    def __post_init__(self) -> None:
        for field_name in (
            "base_x_mm",
            "base_y_mm",
            "tool_yaw_deg",
            "slide_mm",
            "z_axis_mm",
        ):
            object.__setattr__(self, field_name, _finite(field_name, getattr(self, field_name)))


@dataclass(frozen=True)
class WorkspaceEntryClearanceConfig:
    """从区外进入 arm-local 工作区时采用的绝对 Base TCP Z 最低高度。"""

    clearance_base_z_mm: float = WORKING_HEIGHT_BASE_Z_MM

    def __post_init__(self) -> None:
        clearance = _finite("clearance_base_z_mm", self.clearance_base_z_mm)
        if clearance <= 0.0:
            raise ValueError("clearance_base_z_mm must be positive")
        object.__setattr__(self, "clearance_base_z_mm", clearance)


@dataclass(frozen=True)
class RobotMotionEnvelopeConfig:
    """Configuration for known startup and intermediate-stage safety policies.

    This is not a complete collision model or certified mechanical envelope.
    中间安全阶段允许高于 Tray 最终任务 Z 范围，但仍须满足轴和关节软限位。
    """

    startup_pose: StartupSafePoseConfig = field(default_factory=StartupSafePoseConfig)
    workspace_entry: WorkspaceEntryClearanceConfig = field(
        default_factory=WorkspaceEntryClearanceConfig
    )

    def __post_init__(self) -> None:
        if not isinstance(self.startup_pose, StartupSafePoseConfig):
            raise TypeError("startup_pose must be StartupSafePoseConfig")
        if not isinstance(self.workspace_entry, WorkspaceEntryClearanceConfig):
            raise TypeError(
                "workspace_entry must be WorkspaceEntryClearanceConfig"
            )


DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG = RobotMotionEnvelopeConfig()


__all__ = [
    "DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG",
    "RobotMotionEnvelopeConfig",
    "StartupSafePoseConfig",
    "WORKING_HEIGHT_BASE_Z_MM",
    "WorkspaceEntryClearanceConfig",
]

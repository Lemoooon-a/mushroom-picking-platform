"""固定区域扫描、抓取与放置流程的配置和结果值对象。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from application.motion_target import BaseToolTarget
from config.project.robot_motion_envelope import WORKING_HEIGHT_BASE_Z_MM
from vision.target_size import TargetSizeClass


SCAN_POSITION_COUNT = 4


@dataclass(frozen=True)
class ScanPickProfile:
    scan_x_positions_mm: tuple[float, float]
    scan_y_positions_mm: tuple[float, float]
    scan_yaw_deg: float
    place_pose: BaseToolTarget
    oversized_place_pose: BaseToolTarget
    max_picks_per_scan_pose: int
    scan_settle_time_s: float = 0.0

    def __post_init__(self) -> None:
        x_positions = _finite_tuple(
            "scan_x_positions_mm", self.scan_x_positions_mm, length=2
        )
        y_positions = _finite_tuple(
            "scan_y_positions_mm", self.scan_y_positions_mm, length=2
        )
        object.__setattr__(self, "scan_x_positions_mm", x_positions)
        object.__setattr__(self, "scan_y_positions_mm", y_positions)
        yaw = _finite("scan_yaw_deg", self.scan_yaw_deg)
        if yaw != 0.0:
            raise ValueError("scan_yaw_deg must be 0 for the first scan-pick version")
        object.__setattr__(self, "scan_yaw_deg", yaw)
        for name, pose in (
            ("place_pose", self.place_pose),
            ("oversized_place_pose", self.oversized_place_pose),
        ):
            if not isinstance(pose, BaseToolTarget):
                raise TypeError(f"{name} must be a BaseToolTarget")
            if pose.yaw_deg != 0.0:
                raise ValueError(
                    f"{name} yaw_deg must be 0 for the first scan-pick version"
                )
        settle_time = _finite("scan_settle_time_s", self.scan_settle_time_s)
        if settle_time < 0.0:
            raise ValueError("scan_settle_time_s must be non-negative")
        object.__setattr__(self, "scan_settle_time_s", settle_time)
        if (
            isinstance(self.max_picks_per_scan_pose, bool)
            or not isinstance(self.max_picks_per_scan_pose, int)
            or self.max_picks_per_scan_pose <= 0
        ):
            raise ValueError("max_picks_per_scan_pose must be a positive integer")

    @property
    def scan_poses(self) -> tuple[BaseToolTarget, ...]:
        return tuple(
            BaseToolTarget(x_mm, y_mm, WORKING_HEIGHT_BASE_Z_MM, self.scan_yaw_deg)
            for x_mm in self.scan_x_positions_mm
            for y_mm in self.scan_y_positions_mm
        )

    @property
    def scan_z_mm(self) -> float:
        return WORKING_HEIGHT_BASE_Z_MM

    def place_pose_for(self, size_class: TargetSizeClass) -> BaseToolTarget:
        """按视觉尺寸分类选择唯一放置点；未知分类不允许回退。"""

        if not isinstance(size_class, TargetSizeClass):
            raise TypeError("size_class must be a TargetSizeClass")
        if size_class is TargetSizeClass.NORMAL:
            return self.place_pose
        if size_class is TargetSizeClass.OVERSIZED:
            return self.oversized_place_pose
        raise ValueError(f"unsupported target size class: {size_class!r}")


@dataclass(frozen=True)
class ScanPositionResult:
    scan_index: int
    detected_count: int
    picked_count: int
    final_reason: str


@dataclass(frozen=True)
class ScanAndPickResult:
    result: str
    visited_scan_positions: tuple[ScanPositionResult, ...]
    total_picked: int


def _finite_tuple(name: str, value: object, *, length: int) -> tuple[float, ...]:
    if not isinstance(value, tuple) or len(value) != length:
        raise ValueError(f"{name} must be a tuple containing exactly {length} numbers")
    return tuple(_finite(f"{name}[{index}]", item) for index, item in enumerate(value))


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = [
    "SCAN_POSITION_COUNT",
    "ScanAndPickResult",
    "ScanPickProfile",
    "ScanPositionResult",
]

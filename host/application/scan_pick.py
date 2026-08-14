"""固定区域扫描、抓取与放置流程的配置和结果值对象。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from application.motion_target import BaseToolTarget
from config.project.robot_motion_envelope import WORKING_HEIGHT_BASE_Z_MM


@dataclass(frozen=True)
class ScanPickProfile:
    scan_x_positions_mm: tuple[float, float]
    scan_y_positions_mm: tuple[float, float, float, float]
    scan_yaw_deg: float
    place_pose: BaseToolTarget
    max_picks_per_scan_pose: int
    scan_settle_time_s: float = 0.0

    def __post_init__(self) -> None:
        x_positions = _finite_tuple(
            "scan_x_positions_mm", self.scan_x_positions_mm, length=2
        )
        y_positions = _finite_tuple(
            "scan_y_positions_mm", self.scan_y_positions_mm, length=4
        )
        object.__setattr__(self, "scan_x_positions_mm", x_positions)
        object.__setattr__(self, "scan_y_positions_mm", y_positions)
        yaw = _finite("scan_yaw_deg", self.scan_yaw_deg)
        if yaw != 0.0:
            raise ValueError("scan_yaw_deg must be 0 for the first scan-pick version")
        object.__setattr__(self, "scan_yaw_deg", yaw)
        if not isinstance(self.place_pose, BaseToolTarget):
            raise TypeError("place_pose must be a BaseToolTarget")
        if self.place_pose.yaw_deg != 0.0:
            raise ValueError("place_pose yaw_deg must be 0 for the first scan-pick version")
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


__all__ = ["ScanAndPickResult", "ScanPickProfile", "ScanPositionResult"]

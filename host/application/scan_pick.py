"""固定区域扫描、抓取与放置流程的配置和结果值对象。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from application.motion_target import BaseToolTarget


@dataclass(frozen=True)
class ScanPickProfile:
    scan_x_positions_mm: tuple[float, float]
    scan_y_positions_mm: tuple[float, float, float, float]
    scan_z_mm: float
    scan_yaw_deg: float
    place_pose: BaseToolTarget
    place_approach_height_mm: float
    max_picks_per_scan_pose: int

    def __post_init__(self) -> None:
        x_positions = _finite_tuple(
            "scan_x_positions_mm", self.scan_x_positions_mm, length=2
        )
        y_positions = _finite_tuple(
            "scan_y_positions_mm", self.scan_y_positions_mm, length=4
        )
        object.__setattr__(self, "scan_x_positions_mm", x_positions)
        object.__setattr__(self, "scan_y_positions_mm", y_positions)
        object.__setattr__(self, "scan_z_mm", _finite("scan_z_mm", self.scan_z_mm))
        yaw = _finite("scan_yaw_deg", self.scan_yaw_deg)
        if yaw != 0.0:
            raise ValueError("scan_yaw_deg must be 0 for the first scan-pick version")
        object.__setattr__(self, "scan_yaw_deg", yaw)
        if not isinstance(self.place_pose, BaseToolTarget):
            raise TypeError("place_pose must be a BaseToolTarget")
        if self.place_pose.yaw_deg != 0.0:
            raise ValueError("place_pose yaw_deg must be 0 for the first scan-pick version")
        height = _finite(
            "place_approach_height_mm", self.place_approach_height_mm
        )
        if height < 0.0:
            raise ValueError("place_approach_height_mm must be non-negative")
        object.__setattr__(self, "place_approach_height_mm", height)
        if (
            isinstance(self.max_picks_per_scan_pose, bool)
            or not isinstance(self.max_picks_per_scan_pose, int)
            or self.max_picks_per_scan_pose <= 0
        ):
            raise ValueError("max_picks_per_scan_pose must be a positive integer")

    @property
    def scan_poses(self) -> tuple[BaseToolTarget, ...]:
        return tuple(
            BaseToolTarget(x_mm, y_mm, self.scan_z_mm, self.scan_yaw_deg)
            for x_mm in self.scan_x_positions_mm
            for y_mm in self.scan_y_positions_mm
        )

    @property
    def place_pre_pose(self) -> BaseToolTarget:
        return BaseToolTarget(
            self.place_pose.x_mm,
            self.place_pose.y_mm,
            self.place_pose.z_mm + self.place_approach_height_mm,
            self.place_pose.yaw_deg,
        )


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

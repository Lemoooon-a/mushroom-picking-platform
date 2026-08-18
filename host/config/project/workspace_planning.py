"""单一 arm-local 工作区与 Slide 候选策略的集中配置。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class ArmLocalWorkspaceStatus(str, Enum):
    INSIDE = "inside"
    OUTSIDE = "outside"


class SlideSelectionReason(str, Enum):
    KEEP_CURRENT_SLIDE = "keep_current_slide"
    WORKSPACE_CENTER = "workspace_center"
    WORKSPACE_FALLBACK = "workspace_fallback"
    FIXED_SLIDE = "fixed_slide"


@dataclass(frozen=True)
class ArmLocalWorkspaceConfig:
    """机械臂平面局部坐标中的单一闭区间矩形。"""

    local_x_min_mm: float = 100.0
    local_x_max_mm: float = 600.0
    local_y_min_mm: float = 150.0
    local_y_max_mm: float = 350.0
    center_y_mm: float = 250.0
    fallback_local_y_step_mm: float = 10.0
    boundary_tolerance_mm: float = 1e-9
    max_fallback_candidates: int = 64

    def __post_init__(self) -> None:
        for field_name in (
            "local_x_min_mm",
            "local_x_max_mm",
            "local_y_min_mm",
            "local_y_max_mm",
            "center_y_mm",
            "fallback_local_y_step_mm",
            "boundary_tolerance_mm",
        ):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{field_name} must be a finite real number")
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")
        if self.local_x_min_mm >= self.local_x_max_mm:
            raise ValueError("local_x_min_mm must be below local_x_max_mm")
        if not (0.0 < self.local_y_min_mm <= self.center_y_mm <= self.local_y_max_mm):
            raise ValueError("workspace Y interval must be ordered above zero")
        if self.fallback_local_y_step_mm <= 0.0:
            raise ValueError("fallback_local_y_step_mm must be positive")
        if self.boundary_tolerance_mm < 0.0:
            raise ValueError("boundary_tolerance_mm must be non-negative")
        if (
            isinstance(self.max_fallback_candidates, bool)
            or not isinstance(self.max_fallback_candidates, int)
            or self.max_fallback_candidates < 1
        ):
            raise ValueError("max_fallback_candidates must be positive")

    def classify(
        self,
        local_x_mm: float,
        local_y_mm: float,
    ) -> ArmLocalWorkspaceStatus:
        x = _finite("local_x_mm", local_x_mm)
        y = _finite("local_y_mm", local_y_mm)
        tolerance = self.boundary_tolerance_mm
        if not (
            self.local_x_min_mm - tolerance
            <= x
            <= self.local_x_max_mm + tolerance
        ):
            return ArmLocalWorkspaceStatus.OUTSIDE
        if (
            self.local_y_min_mm - tolerance
            <= y
            <= self.local_y_max_mm + tolerance
        ):
            return ArmLocalWorkspaceStatus.INSIDE
        return ArmLocalWorkspaceStatus.OUTSIDE

    def fallback_local_y_candidates(
        self,
        current_local_y_mm: float,
    ) -> tuple[float, ...]:
        """返回有限、稳定、去重的工作区内 local-y 搜索序列。"""

        current = _finite("current_local_y_mm", current_local_y_mm)
        minimum, maximum = self.local_y_min_mm, self.local_y_max_mm
        center = self.center_y_mm
        projected = min(maximum, max(minimum, current))
        raw = [projected]
        step = self.fallback_local_y_step_mm
        maximum_offset = max(center - minimum, maximum - center)
        steps = int(math.ceil(maximum_offset / step))
        for index in range(1, steps + 1):
            offset = index * step
            raw.extend((center - offset, center + offset))
        raw.extend((minimum, maximum))
        result: list[float] = []
        tolerance = self.boundary_tolerance_mm
        for value in raw:
            if value < minimum - tolerance or value > maximum + tolerance:
                continue
            bounded = min(maximum, max(minimum, float(value)))
            if math.isclose(bounded, center, rel_tol=0.0, abs_tol=tolerance):
                continue
            if any(
                math.isclose(bounded, existing, rel_tol=0.0, abs_tol=tolerance)
                for existing in result
            ):
                continue
            result.append(bounded)
            if len(result) >= self.max_fallback_candidates:
                break
        return tuple(result)


DEFAULT_ARM_LOCAL_WORKSPACE_CONFIG = ArmLocalWorkspaceConfig()


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = [
    "ArmLocalWorkspaceConfig",
    "ArmLocalWorkspaceStatus",
    "DEFAULT_ARM_LOCAL_WORKSPACE_CONFIG",
    "SlideSelectionReason",
]

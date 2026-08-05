"""正负偏置工作区与 Slide 候选策略的集中配置。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class OffsetWorkspaceSide(str, Enum):
    POSITIVE = "positive"
    NEGATIVE = "negative"
    OUTSIDE = "outside"


class SlideSelectionReason(str, Enum):
    KEEP_CURRENT_SLIDE = "keep_current_slide"
    POSITIVE_OFFSET_CENTER = "positive_offset_center"
    NEGATIVE_OFFSET_CENTER = "negative_offset_center"
    POSITIVE_OFFSET_FALLBACK = "positive_offset_fallback"
    NEGATIVE_OFFSET_FALLBACK = "negative_offset_fallback"
    FIXED_SLIDE = "fixed_slide"


@dataclass(frozen=True)
class OffsetWorkspaceConfig:
    """机械臂平面局部坐标中的两个闭区间偏置矩形。"""

    local_x_min_mm: float = 50.0
    local_x_max_mm: float = 450.0
    positive_y_min_mm: float = 150.0
    positive_y_max_mm: float = 350.0
    positive_center_y_mm: float = 250.0
    negative_y_min_mm: float = -350.0
    negative_y_max_mm: float = -150.0
    negative_center_y_mm: float = -250.0
    fallback_local_y_step_mm: float = 10.0
    boundary_tolerance_mm: float = 1e-9
    max_fallback_candidates_per_side: int = 64

    def __post_init__(self) -> None:
        for field_name in (
            "local_x_min_mm",
            "local_x_max_mm",
            "positive_y_min_mm",
            "positive_y_max_mm",
            "positive_center_y_mm",
            "negative_y_min_mm",
            "negative_y_max_mm",
            "negative_center_y_mm",
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
        if not (
            0.0 < self.positive_y_min_mm
            <= self.positive_center_y_mm
            <= self.positive_y_max_mm
        ):
            raise ValueError("positive workspace must be ordered above zero")
        if not (
            self.negative_y_min_mm
            <= self.negative_center_y_mm
            <= self.negative_y_max_mm
            < 0.0
        ):
            raise ValueError("negative workspace must be ordered below zero")
        if self.negative_y_max_mm >= self.positive_y_min_mm:
            raise ValueError("positive and negative workspaces must not overlap")
        if self.fallback_local_y_step_mm <= 0.0:
            raise ValueError("fallback_local_y_step_mm must be positive")
        if self.boundary_tolerance_mm < 0.0:
            raise ValueError("boundary_tolerance_mm must be non-negative")
        if (
            isinstance(self.max_fallback_candidates_per_side, bool)
            or not isinstance(self.max_fallback_candidates_per_side, int)
            or self.max_fallback_candidates_per_side < 1
        ):
            raise ValueError("max_fallback_candidates_per_side must be positive")

    def classify(self, local_x_mm: float, local_y_mm: float) -> OffsetWorkspaceSide:
        x = _finite("local_x_mm", local_x_mm)
        y = _finite("local_y_mm", local_y_mm)
        tolerance = self.boundary_tolerance_mm
        if not (
            self.local_x_min_mm - tolerance
            <= x
            <= self.local_x_max_mm + tolerance
        ):
            return OffsetWorkspaceSide.OUTSIDE
        if (
            self.positive_y_min_mm - tolerance
            <= y
            <= self.positive_y_max_mm + tolerance
        ):
            return OffsetWorkspaceSide.POSITIVE
        if (
            self.negative_y_min_mm - tolerance
            <= y
            <= self.negative_y_max_mm + tolerance
        ):
            return OffsetWorkspaceSide.NEGATIVE
        return OffsetWorkspaceSide.OUTSIDE

    def y_interval(self, side: OffsetWorkspaceSide) -> tuple[float, float]:
        if side is OffsetWorkspaceSide.POSITIVE:
            return self.positive_y_min_mm, self.positive_y_max_mm
        if side is OffsetWorkspaceSide.NEGATIVE:
            return self.negative_y_min_mm, self.negative_y_max_mm
        raise ValueError("OUTSIDE has no allowed local-y interval")

    def center_y(self, side: OffsetWorkspaceSide) -> float:
        if side is OffsetWorkspaceSide.POSITIVE:
            return self.positive_center_y_mm
        if side is OffsetWorkspaceSide.NEGATIVE:
            return self.negative_center_y_mm
        raise ValueError("OUTSIDE has no workspace center")

    def center_reason(self, side: OffsetWorkspaceSide) -> SlideSelectionReason:
        if side is OffsetWorkspaceSide.POSITIVE:
            return SlideSelectionReason.POSITIVE_OFFSET_CENTER
        if side is OffsetWorkspaceSide.NEGATIVE:
            return SlideSelectionReason.NEGATIVE_OFFSET_CENTER
        raise ValueError("OUTSIDE has no center selection reason")

    def fallback_reason(self, side: OffsetWorkspaceSide) -> SlideSelectionReason:
        if side is OffsetWorkspaceSide.POSITIVE:
            return SlideSelectionReason.POSITIVE_OFFSET_FALLBACK
        if side is OffsetWorkspaceSide.NEGATIVE:
            return SlideSelectionReason.NEGATIVE_OFFSET_FALLBACK
        raise ValueError("OUTSIDE has no fallback selection reason")

    def fallback_local_y_candidates(
        self,
        side: OffsetWorkspaceSide,
        current_local_y_mm: float,
    ) -> tuple[float, ...]:
        """返回有限、稳定、去重的侧内 local-y 搜索序列。"""

        current = _finite("current_local_y_mm", current_local_y_mm)
        minimum, maximum = self.y_interval(side)
        center = self.center_y(side)
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
            if len(result) >= self.max_fallback_candidates_per_side:
                break
        return tuple(result)


DEFAULT_OFFSET_WORKSPACE_CONFIG = OffsetWorkspaceConfig()


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = [
    "DEFAULT_OFFSET_WORKSPACE_CONFIG",
    "OffsetWorkspaceConfig",
    "OffsetWorkspaceSide",
    "SlideSelectionReason",
]

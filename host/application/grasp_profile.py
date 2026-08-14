"""抓取策略值对象；tracked example 不包含真实抓取参数。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class GraspYawMode(str, Enum):
    FIXED = "fixed"
    KEEP_CURRENT = "keep_current"
    FROM_OBSERVATION = "from_observation"


@dataclass(frozen=True)
class GraspProfile:
    contact_offset_mm: float
    yaw_mode: GraspYawMode
    fixed_yaw_deg: float | None
    minimum_confidence: float
    maximum_observation_age_s: float
    suction_settle_time_s: float = 2.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "contact_offset_mm",
            _finite("contact_offset_mm", self.contact_offset_mm),
        )
        if not isinstance(self.yaw_mode, GraspYawMode):
            raise TypeError("yaw_mode must be a GraspYawMode")
        if self.yaw_mode is GraspYawMode.FIXED:
            if self.fixed_yaw_deg is None:
                raise ValueError("fixed_yaw_deg is required for FIXED yaw mode")
            _finite("fixed_yaw_deg", self.fixed_yaw_deg)
        elif self.fixed_yaw_deg is not None:
            raise ValueError("fixed_yaw_deg must be None unless yaw_mode is FIXED")
        confidence = _finite("minimum_confidence", self.minimum_confidence)
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_confidence must be between 0 and 1")
        age = _finite("maximum_observation_age_s", self.maximum_observation_age_s)
        if age <= 0.0:
            raise ValueError("maximum_observation_age_s must be positive")
        settle_time = _finite("suction_settle_time_s", self.suction_settle_time_s)
        if settle_time < 0.0:
            raise ValueError("suction_settle_time_s must be non-negative")
        object.__setattr__(self, "suction_settle_time_s", settle_time)


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = ["GraspProfile", "GraspYawMode"]

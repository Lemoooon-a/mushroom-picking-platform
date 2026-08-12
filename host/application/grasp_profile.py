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
    approach_offset_mm: float
    contact_offset_mm: float
    retreat_offset_mm: float | None
    yaw_mode: GraspYawMode
    fixed_yaw_deg: float | None
    minimum_confidence: float
    maximum_observation_age_s: float
    retreat_z_mm: float | None = None
    minimum_transit_z_mm: float | None = None

    def __post_init__(self) -> None:
        approach = _finite("approach_offset_mm", self.approach_offset_mm)
        contact = _finite("contact_offset_mm", self.contact_offset_mm)
        if approach < 0.0:
            raise ValueError("approach_offset_mm must be non-negative")
        if approach < contact:
            raise ValueError("approach_offset_mm must not be below contact_offset_mm")
        has_retreat_offset = self.retreat_offset_mm is not None
        has_retreat_z = self.retreat_z_mm is not None
        if has_retreat_offset == has_retreat_z:
            raise ValueError(
                "exactly one of retreat_offset_mm and retreat_z_mm is required"
            )
        if has_retreat_offset:
            retreat = _finite("retreat_offset_mm", self.retreat_offset_mm)
            if retreat < 0.0:
                raise ValueError("retreat_offset_mm must be non-negative")
            if retreat < contact:
                raise ValueError(
                    "retreat_offset_mm must not be below contact_offset_mm"
                )
        else:
            _finite("retreat_z_mm", self.retreat_z_mm)
        if self.minimum_transit_z_mm is not None:
            _finite("minimum_transit_z_mm", self.minimum_transit_z_mm)
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


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = ["GraspProfile", "GraspYawMode"]

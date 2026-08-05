"""应用层 Base-frame Tool 目标值对象。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from geometry.rigid_transform import RigidTransform


@dataclass(frozen=True)
class BaseToolTarget:
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float | None = None

    def __post_init__(self) -> None:
        for name in ("x_mm", "y_mm", "z_mm"):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))
        if self.yaw_deg is not None:
            object.__setattr__(self, "yaw_deg", _finite("yaw_deg", self.yaw_deg))

    def as_transform(self, *, fallback_yaw_deg: float | None = None) -> RigidTransform:
        yaw = self.yaw_deg if self.yaw_deg is not None else fallback_yaw_deg
        if yaw is None:
            raise ValueError("target yaw is unresolved")
        return RigidTransform.from_xyz_yaw_deg(
            x_mm=self.x_mm, y_mm=self.y_mm, z_mm=self.z_mm, yaw_deg=yaw
        )


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = ["BaseToolTarget"]

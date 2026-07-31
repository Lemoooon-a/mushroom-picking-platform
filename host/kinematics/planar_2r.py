"""右手坐标系中的平面二旋转关节正逆运动学。"""

from __future__ import annotations

from dataclasses import dataclass
import math


_REACHABILITY_REL_TOL = 1e-12
_SINGULARITY_TOL = 1e-12


class KinematicsError(ValueError):
    """运动学输入、参数或计算结果无效。"""


class UnreachableTargetError(KinematicsError):
    """目标点位于二连杆工作空间之外。"""


@dataclass(frozen=True)
class PlanarPoint:
    """XY 平面点；单位与运动学对象的连杆长度相同。"""

    x: float
    y: float


@dataclass(frozen=True)
class JointAngles:
    """肩、肘逻辑关节角，单位为 rad。"""

    shoulder_rad: float
    elbow_rad: float


@dataclass(frozen=True)
class Planar2RKinematics:
    """纯计算的平面 2R 正逆运动学。

    坐标系为 x 向前、y 向左、z 向上。肩角相对全局 +x，肘角相对
    link1；从 +z 方向看，正角按右手定则从 +x 转向 +y。
    """

    link1_length: float
    link2_length: float

    def __post_init__(self) -> None:
        _require_positive_finite("link1_length", self.link1_length)
        _require_positive_finite("link2_length", self.link2_length)

    def forward(self, shoulder_rad: float, elbow_rad: float) -> PlanarPoint:
        """由肩、肘逻辑角计算末端 XY 坐标。"""

        _require_finite("shoulder_rad", shoulder_rad)
        _require_finite("elbow_rad", elbow_rad)
        link2_angle = shoulder_rad + elbow_rad
        return PlanarPoint(
            x=(
                self.link1_length * math.cos(shoulder_rad)
                + self.link2_length * math.cos(link2_angle)
            ),
            y=(
                self.link1_length * math.sin(shoulder_rad)
                + self.link2_length * math.sin(link2_angle)
            ),
        )

    def inverse(self, x: float, y: float) -> tuple[JointAngles, ...]:
        """由末端 XY 坐标返回全部主值数学逆解。

        普通位置按肘角为正、肘角为负的顺序返回两支解；伸直或完全折叠
        等奇异位置只返回一个去重后的解。本方法不应用关节软件限位。
        """

        _require_finite("x", x)
        _require_finite("y", y)

        radius = math.hypot(x, y)
        inner_radius = abs(self.link1_length - self.link2_length)
        outer_radius = self.link1_length + self.link2_length
        reach_tolerance = _REACHABILITY_REL_TOL * outer_radius

        if (
            radius < inner_radius - reach_tolerance
            or radius > outer_radius + reach_tolerance
        ):
            raise UnreachableTargetError(
                f"target ({x!r}, {y!r}) has radius {radius:.12g}, outside "
                f"reachable interval [{inner_radius:.12g}, {outer_radius:.12g}]"
            )

        cosine_elbow = (
            radius * radius
            - self.link1_length * self.link1_length
            - self.link2_length * self.link2_length
        ) / (2.0 * self.link1_length * self.link2_length)
        cosine_tolerance = _REACHABILITY_REL_TOL * 4.0
        if cosine_elbow < -1.0 - cosine_tolerance or cosine_elbow > 1.0 + cosine_tolerance:
            raise UnreachableTargetError(
                f"target ({x!r}, {y!r}) produces invalid elbow cosine "
                f"{cosine_elbow:.12g}"
            )
        cosine_elbow = min(1.0, max(-1.0, cosine_elbow))
        sine_magnitude = math.sqrt(max(0.0, 1.0 - cosine_elbow * cosine_elbow))

        positive_elbow = math.atan2(sine_magnitude, cosine_elbow)
        elbow_candidates = [positive_elbow]
        if sine_magnitude > _SINGULARITY_TOL:
            elbow_candidates.append(-positive_elbow)

        solutions: list[JointAngles] = []
        for elbow_rad in elbow_candidates:
            if radius <= reach_tolerance and inner_radius <= reach_tolerance:
                # 等长连杆在原点完全折叠时肩角任意；固定为 0，保证输出确定。
                shoulder_rad = 0.0
            else:
                shoulder_rad = math.atan2(y, x) - math.atan2(
                    self.link2_length * math.sin(elbow_rad),
                    self.link1_length + self.link2_length * math.cos(elbow_rad),
                )
            solutions.append(
                JointAngles(
                    shoulder_rad=_normalize_angle(shoulder_rad),
                    elbow_rad=_normalize_angle(elbow_rad),
                )
            )
        return tuple(solutions)


def _normalize_angle(angle_rad: float) -> float:
    """归一化到 (-pi, pi]，并统一使用正向 pi 端点。"""

    wrapped = (angle_rad + math.pi) % math.tau - math.pi
    if math.isclose(wrapped, -math.pi, abs_tol=1e-15):
        return math.pi
    return 0.0 if math.isclose(wrapped, 0.0, abs_tol=1e-15) else wrapped


def _require_finite(name: str, value: float) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise KinematicsError(f"{name} must be a finite real number, got {value!r}")
    if not math.isfinite(value):
        raise KinematicsError(f"{name} must be finite, got {value!r}")


def _require_positive_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value <= 0:
        raise KinematicsError(f"{name} must be greater than zero, got {value!r}")


__all__ = [
    "JointAngles",
    "KinematicsError",
    "Planar2RKinematics",
    "PlanarPoint",
    "UnreachableTargetError",
]

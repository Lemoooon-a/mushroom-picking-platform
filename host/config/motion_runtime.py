"""五轴 Runtime 的集中纯数据运动参数配置。"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
import math
from typing import Mapping

from motion.unified_protocol import ArrivalConfig, AxisName


class MotionRuntimeConfigLoadError(RuntimeError):
    """本地运动 Runtime 配置缺失或内容无效。"""


def _validate_positive(value: object, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    if not 0 < float(value) < float("inf"):
        raise ValueError(f"{name} must be finite and greater than zero")


def _validate_optional_positive(value: float | None, name: str) -> None:
    if value is None:
        return
    _validate_positive(value, name)


@dataclass(frozen=True)
class AxisMotionProfile:
    """一个轴的默认运动参数和到位判断参数。"""

    default_velocity: float | None
    default_acceleration: float | None
    arrival: ArrivalConfig

    def __post_init__(self) -> None:
        _validate_optional_positive(self.default_velocity, "default_velocity")
        _validate_optional_positive(
            self.default_acceleration,
            "default_acceleration",
        )
        if not isinstance(self.arrival, ArrivalConfig):
            raise TypeError("arrival must be ArrivalConfig")


@dataclass(frozen=True)
class LinearAxisPositionLimits:
    """线性轴归零后的 Host 机器坐标范围，单位 mm。"""

    minimum_position_mm: float
    maximum_position_mm: float

    def __post_init__(self) -> None:
        for field_name in ("minimum_position_mm", "maximum_position_mm"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{field_name} must be a real number")
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be finite")
        if self.minimum_position_mm >= self.maximum_position_mm:
            raise ValueError(
                "minimum_position_mm must be below maximum_position_mm"
            )


@dataclass(frozen=True)
class LinearAxisMotionLimits:
    """线性轴 Host 运动参数上限，单位 mm/s 和 mm/s²。"""

    maximum_velocity_mm_s: float
    maximum_acceleration_mm_s2: float

    def __post_init__(self) -> None:
        _validate_positive(
            self.maximum_velocity_mm_s,
            "maximum_velocity_mm_s",
        )
        _validate_positive(
            self.maximum_acceleration_mm_s2,
            "maximum_acceleration_mm_s2",
        )


@dataclass(frozen=True)
class MotionRuntimeConfig:
    """Slide、Z、肩、肘和末端旋转轴的集中运行参数。"""

    slide: AxisMotionProfile
    z: AxisMotionProfile
    shoulder: AxisMotionProfile
    elbow: AxisMotionProfile
    rotation: AxisMotionProfile
    slide_position_limits: LinearAxisPositionLimits
    z_position_limits: LinearAxisPositionLimits
    slide_motion_limits: LinearAxisMotionLimits
    z_motion_limits: LinearAxisMotionLimits

    def __post_init__(self) -> None:
        for field_name in ("slide", "z", "shoulder", "elbow", "rotation"):
            if not isinstance(getattr(self, field_name), AxisMotionProfile):
                raise TypeError(f"{field_name} must be AxisMotionProfile")
        for field_name in ("slide_position_limits", "z_position_limits"):
            if not isinstance(getattr(self, field_name), LinearAxisPositionLimits):
                raise TypeError(f"{field_name} must be LinearAxisPositionLimits")
        for axis_name in ("slide", "z"):
            limits_field = f"{axis_name}_motion_limits"
            limits = getattr(self, limits_field)
            if not isinstance(limits, LinearAxisMotionLimits):
                raise TypeError(f"{limits_field} must be LinearAxisMotionLimits")
            profile = getattr(self, axis_name)
            if profile.default_velocity is None:
                raise ValueError(f"{axis_name}.default_velocity must be configured")
            if profile.default_acceleration is None:
                raise ValueError(
                    f"{axis_name}.default_acceleration must be configured"
                )
            if profile.default_velocity > limits.maximum_velocity_mm_s:
                raise ValueError(
                    f"{axis_name}.default_velocity must not exceed "
                    f"{limits_field}.maximum_velocity_mm_s"
                )
            if profile.default_acceleration > limits.maximum_acceleration_mm_s2:
                raise ValueError(
                    f"{axis_name}.default_acceleration must not exceed "
                    f"{limits_field}.maximum_acceleration_mm_s2"
                )

    def profiles(self) -> Mapping[AxisName, AxisMotionProfile]:
        return {
            AxisName.SLIDE: self.slide,
            AxisName.Z: self.z,
            AxisName.SHOULDER: self.shoulder,
            AxisName.ELBOW: self.elbow,
            AxisName.ROTATION: self.rotation,
        }

    def arrival_configs(self) -> dict[AxisName, ArrivalConfig]:
        return {
            axis: profile.arrival
            for axis, profile in self.profiles().items()
        }

    def default_motion_parameters(
        self,
    ) -> dict[AxisName, tuple[float | None, float | None]]:
        return {
            axis: (profile.default_velocity, profile.default_acceleration)
            for axis, profile in self.profiles().items()
        }

    def linear_position_limits(self) -> dict[AxisName, tuple[float, float]]:
        return {
            AxisName.SLIDE: (
                self.slide_position_limits.minimum_position_mm,
                self.slide_position_limits.maximum_position_mm,
            ),
            AxisName.Z: (
                self.z_position_limits.minimum_position_mm,
                self.z_position_limits.maximum_position_mm,
            ),
        }

    def linear_motion_limits(self) -> dict[AxisName, tuple[float, float]]:
        return {
            AxisName.SLIDE: (
                self.slide_motion_limits.maximum_velocity_mm_s,
                self.slide_motion_limits.maximum_acceleration_mm_s2,
            ),
            AxisName.Z: (
                self.z_motion_limits.maximum_velocity_mm_s,
                self.z_motion_limits.maximum_acceleration_mm_s2,
            ),
        }


def load_local_motion_config() -> MotionRuntimeConfig:
    """加载被 Git 忽略的同包 ``motion_local.py``。"""

    module_name = f"{__package__}.motion_local"
    try:
        module = import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name != module_name:
            raise MotionRuntimeConfigLoadError(
                f"导入 {module_name} 时缺少依赖模块 {exc.name!r}"
            ) from exc
        raise MotionRuntimeConfigLoadError(
            "未找到本地运动配置。请复制 "
            "host/config/motion_local.example.py 为 "
            "host/config/motion_local.py，并仅填写经过当前台架确认的参数。"
        ) from exc
    except Exception as exc:
        raise MotionRuntimeConfigLoadError(
            f"导入本地运动配置 {module_name} 失败: {exc}"
        ) from exc

    config = getattr(module, "MOTION", None)
    if not isinstance(config, MotionRuntimeConfig):
        raise MotionRuntimeConfigLoadError(
            f"{module_name}.MOTION 必须是 MotionRuntimeConfig 实例"
        )
    return config


__all__ = [
    "ArrivalConfig",
    "AxisMotionProfile",
    "LinearAxisMotionLimits",
    "LinearAxisPositionLimits",
    "MotionRuntimeConfig",
    "MotionRuntimeConfigLoadError",
    "load_local_motion_config",
]

"""五轴 Runtime 的集中纯数据运动参数配置。"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import Mapping

from motion.unified_protocol import ArrivalConfig, AxisName


class MotionRuntimeConfigLoadError(RuntimeError):
    """本地运动 Runtime 配置缺失或内容无效。"""


def _validate_optional_positive(value: float | None, name: str) -> None:
    if value is None:
        return
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number or None")
    if not 0 < float(value) < float("inf"):
        raise ValueError(f"{name} must be finite and greater than zero")


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
class MotionRuntimeConfig:
    """Slide、Z、肩、肘和末端旋转轴的集中运行参数。"""

    slide: AxisMotionProfile
    z: AxisMotionProfile
    shoulder: AxisMotionProfile
    elbow: AxisMotionProfile
    rotation: AxisMotionProfile

    def __post_init__(self) -> None:
        for field_name in ("slide", "z", "shoulder", "elbow", "rotation"):
            if not isinstance(getattr(self, field_name), AxisMotionProfile):
                raise TypeError(f"{field_name} must be AxisMotionProfile")

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
    "MotionRuntimeConfig",
    "MotionRuntimeConfigLoadError",
    "load_local_motion_config",
]

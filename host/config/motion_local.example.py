"""
运动 Runtime 本地配置结构示例。

以下数值全部是 EXAMPLE / BENCH-TEST PLACEHOLDER，NOT PRODUCTION-CALIBRATED。
复制为 motion_local.py 后，必须用当前机构已经确认的参数替换。
"""

from .motion_runtime import (
    ArrivalConfig,
    AxisMotionProfile,
    LinearAxisMotionLimits,
    LinearAxisPositionLimits,
    MotionRuntimeConfig,
)


MOTION = MotionRuntimeConfig(
    slide=AxisMotionProfile(
        default_velocity=60.0,  # mm/s; current bench-tested operating default
        default_acceleration=180.0,  # mm/s²; current bench-tested default
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 180.0),
    ),
    z=AxisMotionProfile(
        default_velocity=8.0,  # mm/s; current bench-tested operating default
        default_acceleration=25.0,  # mm/s²; current bench-tested default
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 180.0),
    ),
    shoulder=AxisMotionProfile(
        default_velocity=2.0,  # deg/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=None,  # backend has no verified engineering mapping
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    elbow=AxisMotionProfile(
        default_velocity=2.0,  # deg/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=None,  # backend has no verified engineering mapping
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    rotation=AxisMotionProfile(
        default_velocity=None,  # physical deg/s mapping is not verified
        default_acceleration=None,  # physical deg/s² mapping is not verified
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    # 当前 STM32 firmware 软限位；Z 已按最高点归零和 190 mm 行程标定。
    slide_position_limits=LinearAxisPositionLimits(0.0, 799.988),
    z_position_limits=LinearAxisPositionLimits(-190.0, 0.0),
    # 与当前 STM32 firmware 保守硬上限同步；firmware 变化时必须一并复核。
    slide_motion_limits=LinearAxisMotionLimits(72.0, 180.0),
    z_motion_limits=LinearAxisMotionLimits(10.0, 25.0),
)

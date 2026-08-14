"""
当前机械臂实际加载的正式运动 Runtime 配置。

速度、加速度和到位参数已用于当前台架测试，但尚未完成生产工况标定。
Z 位置范围同步已完成行程验证的 STM32 firmware 软限位；Slide 仍需按最终整机验收更新。
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
        default_velocity=60.0,  # mm/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=180.0,  # mm/s²; NOT PRODUCTION-CALIBRATED
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 180.0),
    ),
    z=AxisMotionProfile(
        default_velocity=8.0,  # mm/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=25.0,  # mm/s²; NOT PRODUCTION-CALIBRATED
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 180.0),
    ),
    shoulder=AxisMotionProfile(
        default_velocity=30.0,  # deg/s; BENCH-TEST DEFAULT, NOT PRODUCTION-CALIBRATED
        default_acceleration=None,  # backend has no verified engineering mapping
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    elbow=AxisMotionProfile(
        default_velocity=30.0,  # deg/s; BENCH-TEST DEFAULT, NOT PRODUCTION-CALIBRATED
        default_acceleration=None,  # backend has no verified engineering mapping
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    rotation=AxisMotionProfile(
        default_velocity=None,  # physical deg/s mapping is not verified
        default_acceleration=None,  # physical deg/s² mapping is not verified
        arrival=ArrivalConfig(0.5, 0.2, 0.05, 10.0),
    ),
    slide_position_limits=LinearAxisPositionLimits(0.0, 799.988),
    z_position_limits=LinearAxisPositionLimits(-190.0, 0.0),
    slide_motion_limits=LinearAxisMotionLimits(72.0, 180.0),
    z_motion_limits=LinearAxisMotionLimits(10.0, 25.0),
)

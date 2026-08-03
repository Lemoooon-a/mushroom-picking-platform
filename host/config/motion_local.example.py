"""
运动 Runtime 本地配置结构示例。

以下数值全部是 EXAMPLE / BENCH-TEST PLACEHOLDER，NOT PRODUCTION-CALIBRATED。
复制为 motion_local.py 后，必须用当前机构已经确认的参数替换。
"""

from .motion_runtime import ArrivalConfig, AxisMotionProfile, MotionRuntimeConfig


MOTION = MotionRuntimeConfig(
    slide=AxisMotionProfile(
        default_velocity=2.0,  # mm/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=4.0,  # mm/s²; NOT PRODUCTION-CALIBRATED
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 10.0),
    ),
    z=AxisMotionProfile(
        default_velocity=1.0,  # mm/s; EXAMPLE / BENCH-TEST PLACEHOLDER
        default_acceleration=3.0,  # mm/s²; NOT PRODUCTION-CALIBRATED
        arrival=ArrivalConfig(0.2, 0.2, 0.05, 10.0),
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
)

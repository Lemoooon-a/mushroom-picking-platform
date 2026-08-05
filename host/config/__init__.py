"""主机侧静态配置；公开旧名称使用惰性导入，避免纯数学模块加载硬件驱动。"""

from __future__ import annotations


_FEETECH_NAMES = {
    "END_EFFECTOR_ROTATION_CONFIG",
    "END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW",
    "END_EFFECTOR_ROTATION_POSITIVE_DIRECTION",
    "FEETECH_MODEL_PROFILES",
    "SM45BL_C001_PROFILE",
}
_JOINT_NAMES = {
    "ELBOW_JOINT_CONFIG",
    "JOINT_CONFIGS",
    "SHOULDER_JOINT_CONFIG",
}


def __getattr__(name: str) -> object:
    if name in _FEETECH_NAMES:
        from . import feetech

        return getattr(feetech, name)
    if name in _JOINT_NAMES:
        from . import joints

        return getattr(joints, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = sorted(_FEETECH_NAMES | _JOINT_NAMES)

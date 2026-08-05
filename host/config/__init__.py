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
_OFFSET_WORKSPACE_NAMES = {
    "DEFAULT_OFFSET_WORKSPACE_CONFIG",
    "OffsetWorkspaceConfig",
}
_ROBOT_MOTION_ENVELOPE_NAMES = {
    "DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG",
    "RobotMotionEnvelopeConfig",
    "SideSwitchClearanceConfig",
    "StartupSafePoseConfig",
}
_TRAY_WORKSPACE_NAMES = {"TrayWorkspaceConfig"}


def __getattr__(name: str) -> object:
    if name in _FEETECH_NAMES:
        from . import feetech

        return getattr(feetech, name)
    if name in _JOINT_NAMES:
        from . import joints

        return getattr(joints, name)
    if name in _OFFSET_WORKSPACE_NAMES:
        from . import workspace_planning

        return getattr(workspace_planning, name)
    if name in _ROBOT_MOTION_ENVELOPE_NAMES:
        from . import robot_motion_envelope

        return getattr(robot_motion_envelope, name)
    if name in _TRAY_WORKSPACE_NAMES:
        from . import tray_workspace

        return getattr(tray_workspace, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = sorted(
    _FEETECH_NAMES
    | _JOINT_NAMES
    | _OFFSET_WORKSPACE_NAMES
    | _ROBOT_MOTION_ENVELOPE_NAMES
    | _TRAY_WORKSPACE_NAMES
)

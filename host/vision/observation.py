"""视觉目标观察的数据契约；本模块不采集图像。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState


class CaptureMotionState(str, Enum):
    STATIONARY = "stationary"
    MOVING = "moving"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class VisionTargetObservation:
    """Camera frame 中的完整目标位姿及同一采集时刻的轴快照。"""

    camera_T_target: RigidTransform
    capture_axis_state: RobotAxisState
    frame_id: str
    capture_motion_state: CaptureMotionState
    timestamp: float | None
    confidence: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.camera_T_target, RigidTransform):
            raise TypeError("camera_T_target must be a RigidTransform")
        if not isinstance(self.capture_axis_state, RobotAxisState):
            raise TypeError("capture_axis_state must be a RobotAxisState")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("frame_id must be a non-empty string")
        if not isinstance(self.capture_motion_state, CaptureMotionState):
            raise TypeError("capture_motion_state must be a CaptureMotionState")
        _optional_finite("timestamp", self.timestamp)
        confidence = _optional_finite("confidence", self.confidence)
        if confidence is not None and not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")


def _optional_finite(name: str, value: float | None) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number or None")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = ["CaptureMotionState", "VisionTargetObservation"]

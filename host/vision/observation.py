"""视觉观察与拍照快照的数据契约；本模块不访问相机或机器人硬件。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from collections.abc import Sequence

import numpy as np

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState


class CaptureMotionState(str, Enum):
    STATIONARY = "stationary"
    MOVING = "moving"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z


@dataclass(frozen=True)
class Quaternion:
    """单位四元数，分量顺序为 x/y/z/w。"""

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        values = tuple(_finite(name, getattr(self, name)) for name in ("x", "y", "z", "w"))
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("orientation quaternion must have unit norm")
        for name, value in zip(("x", "y", "z", "w"), values, strict=True):
            object.__setattr__(self, name, value)

    def to_rotation_matrix(self) -> np.ndarray:
        x, y, z, w = self.x, self.y, self.z, self.w
        return np.array(
            (
                (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
            ),
            dtype=float,
        )

    @classmethod
    def from_rotation_matrix(cls, matrix: np.ndarray) -> "Quaternion":
        rotation = np.asarray(matrix, dtype=float)
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            return cls(
                x=(rotation[2, 1] - rotation[1, 2]) / scale,
                y=(rotation[0, 2] - rotation[2, 0]) / scale,
                z=(rotation[1, 0] - rotation[0, 1]) / scale,
                w=0.25 * scale,
            )
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            return cls(0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale)
        if index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            return cls((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale)
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        return cls((rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale)


@dataclass(frozen=True)
class CaptureSnapshot:
    request_id: str
    axis_state: RobotAxisState
    base_T_tool: RigidTransform
    captured_at: float

    def __post_init__(self) -> None:
        _non_empty("request_id", self.request_id)
        if not isinstance(self.axis_state, RobotAxisState):
            raise TypeError("axis_state must be a RobotAxisState")
        if not isinstance(self.base_T_tool, RigidTransform):
            raise TypeError("base_T_tool must be a RigidTransform")
        object.__setattr__(self, "captured_at", _finite("captured_at", self.captured_at))


@dataclass(frozen=True, init=False)
class VisionTargetObservation:
    """Camera frame 目标及与拍摄时刻绑定的五轴快照。

    新调用方应传 ``position_mm``/``orientation``；``camera_T_target`` 仅为兼容
    已有解析器和测试保留。orientation 缺失时，内部单位旋转不得解释为观测 yaw。
    """

    request_id: str
    frame_id: str
    timestamp: float | None
    position_mm: Vector3
    orientation: Quaternion | None
    confidence: float | None
    target_id: str | None
    capture_axis_state: RobotAxisState
    capture_motion_state: CaptureMotionState
    camera_T_target: RigidTransform

    def __init__(
        self,
        *,
        frame_id: str,
        capture_axis_state: RobotAxisState,
        capture_motion_state: CaptureMotionState,
        timestamp: float | None,
        request_id: str = "legacy-observation",
        position_mm: Vector3 | Sequence[float] | None = None,
        orientation: Quaternion | None = None,
        confidence: float | None = None,
        target_id: str | None = None,
        camera_T_target: RigidTransform | None = None,
    ) -> None:
        request_id = _non_empty("request_id", request_id)
        frame_id = _non_empty("frame_id", frame_id)
        if not isinstance(capture_axis_state, RobotAxisState):
            raise TypeError("capture_axis_state must be a RobotAxisState")
        if not isinstance(capture_motion_state, CaptureMotionState):
            raise TypeError("capture_motion_state must be a CaptureMotionState")
        timestamp = _optional_finite("timestamp", timestamp)
        confidence = _optional_finite("confidence", confidence)
        if confidence is not None and not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        if target_id is not None:
            target_id = _non_empty("target_id", target_id)
        if orientation is not None and not isinstance(orientation, Quaternion):
            raise TypeError("orientation must be a Quaternion or None")

        if camera_T_target is not None:
            if not isinstance(camera_T_target, RigidTransform):
                raise TypeError("camera_T_target must be a RigidTransform")
            derived_position = Vector3(*(float(value) for value in camera_T_target.translation_mm))
            if position_mm is not None and _vector3(position_mm) != derived_position:
                raise ValueError("position_mm does not match camera_T_target translation")
            position = derived_position
            derived_orientation = Quaternion.from_rotation_matrix(camera_T_target.rotation_matrix)
            if orientation is not None and orientation != derived_orientation:
                raise ValueError("orientation does not match camera_T_target rotation")
            orientation = derived_orientation
        else:
            if position_mm is None:
                raise ValueError("position_mm is required when camera_T_target is absent")
            position = _vector3(position_mm)
            matrix = np.eye(4, dtype=float)
            matrix[:3, 3] = position.as_tuple()
            if orientation is not None:
                matrix[:3, :3] = orientation.to_rotation_matrix()
            camera_T_target = RigidTransform(matrix)
        if position.z <= 0.0:
            raise ValueError("position_mm.z depth must be positive")

        for name, value in (
            ("request_id", request_id), ("frame_id", frame_id), ("timestamp", timestamp),
            ("position_mm", position), ("orientation", orientation), ("confidence", confidence),
            ("target_id", target_id), ("capture_axis_state", capture_axis_state),
            ("capture_motion_state", capture_motion_state), ("camera_T_target", camera_T_target),
        ):
            object.__setattr__(self, name, value)


def create_capture_snapshot(
    *,
    request_id: str,
    axis_state: RobotAxisState,
    base_T_tool: RigidTransform,
    captured_at: float,
    motion_state: CaptureMotionState,
) -> CaptureSnapshot:
    if motion_state is not CaptureMotionState.STATIONARY:
        raise ValueError("capture requires a stationary, arrived five-axis state")
    return CaptureSnapshot(request_id, axis_state, base_T_tool, captured_at)


def require_snapshot_unchanged(
    snapshot: CaptureSnapshot,
    *,
    axis_state: RobotAxisState,
    motion_state: CaptureMotionState,
    tolerance: float | None = None,
) -> None:
    if not isinstance(snapshot, CaptureSnapshot):
        raise TypeError("snapshot must be a CaptureSnapshot")
    if motion_state is not CaptureMotionState.STATIONARY:
        raise ValueError("robot moved while the vision request was in flight")
    if not isinstance(axis_state, RobotAxisState):
        raise TypeError("axis_state must be a RobotAxisState")
    if tolerance is not None:
        uniform_tolerance = _finite("tolerance", tolerance)
        if uniform_tolerance < 0.0:
            raise ValueError("tolerance must be non-negative")
    else:
        uniform_tolerance = None
    for name in _AXIS_FIELDS:
        before = getattr(snapshot.axis_state, name)
        after = getattr(axis_state, name)
        allowed = (
            _CAPTURE_AXIS_TOLERANCES[name]
            if uniform_tolerance is None
            else uniform_tolerance
        )
        delta = abs(before - after)
        if delta > allowed:
            raise ValueError(
                "robot axis state changed while the vision request was in flight: "
                f"{name} delta={delta:g} exceeds tolerance={allowed:g}"
            )


_AXIS_FIELDS = ("slide_mm", "z_mm", "shoulder_deg", "elbow_deg", "rotation_deg")
_CAPTURE_AXIS_TOLERANCES = {
    # 仅吸收静止反馈量化，不替代 STATIONARY/busy 运动门禁。
    "slide_mm": 0.01,
    "z_mm": 0.01,
    "shoulder_deg": 0.01,
    "elbow_deg": 0.01,
    "rotation_deg": 0.1,
}


def _vector3(value: Vector3 | Sequence[float]) -> Vector3:
    if isinstance(value, Vector3):
        return value
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 3:
        raise TypeError("position_mm must be Vector3 or a three-number sequence")
    return Vector3(value[0], value[1], value[2])


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _optional_finite(name: str, value: float | None) -> float | None:
    return None if value is None else _finite(name, value)


def _non_empty(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


FiveAxisState = RobotAxisState

__all__ = [
    "CaptureMotionState", "CaptureSnapshot", "FiveAxisState", "Quaternion", "Vector3",
    "VisionTargetObservation", "create_capture_snapshot", "require_snapshot_unchanged",
]

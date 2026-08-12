"""手眼标定记录及其独立验证状态。"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
import math

from config.frame_transforms import FrameTransformsDocument
from geometry.rigid_transform import RigidTransform


class HandEyeCalibrationStatus(str, Enum):
    MISSING = "missing"
    PROVISIONAL = "provisional"
    VALIDATED = "validated"


@dataclass(frozen=True)
class HandEyeCalibration:
    """Camera 到 Tool 的固定外参及其证据状态。"""

    tool_T_camera: RigidTransform
    validated: bool
    source: str
    method: str
    created_at: str | None = None
    target_compensation_base_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_compensation_camera_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not isinstance(self.tool_T_camera, RigidTransform):
            raise TypeError("tool_T_camera must be a RigidTransform")
        if not isinstance(self.validated, bool):
            raise TypeError("validated must be a bool")
        for field_name in ("source", "method"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field_name} must be a non-empty string")
        if self.created_at is not None and not isinstance(self.created_at, str):
            raise TypeError("created_at must be a string or None")
        object.__setattr__(
            self,
            "target_compensation_base_mm",
            _finite_triplet(
                "target_compensation_base_mm",
                self.target_compensation_base_mm,
            ),
        )
        object.__setattr__(
            self,
            "target_compensation_camera_mm",
            _finite_triplet(
                "target_compensation_camera_mm",
                self.target_compensation_camera_mm,
            ),
        )
        if any(self.target_compensation_base_mm) and any(
            self.target_compensation_camera_mm
        ):
            raise ValueError(
                "target compensation must be configured in Camera or Base, not both"
            )

    @property
    def status(self) -> HandEyeCalibrationStatus:
        return (
            HandEyeCalibrationStatus.VALIDATED
            if self.validated
            else HandEyeCalibrationStatus.PROVISIONAL
        )

    def compensate_base_point(
        self,
        point_xyz_mm: Sequence[float],
    ) -> tuple[float, float, float]:
        """把 Base 点加上不随 Camera/Tool 姿态旋转的目标补偿。"""

        point = _finite_triplet("point_xyz_mm", point_xyz_mm)
        return tuple(
            coordinate + offset
            for coordinate, offset in zip(
                point,
                self.target_compensation_base_mm,
                strict=True,
            )
        )

    def compensate_camera_point(
        self,
        point_xyz_mm: Sequence[float],
    ) -> tuple[float, float, float]:
        """在 Camera frame 中给检测点加目标补偿。"""

        point = _finite_triplet("point_xyz_mm", point_xyz_mm)
        return tuple(
            coordinate + offset
            for coordinate, offset in zip(
                point,
                self.target_compensation_camera_mm,
                strict=True,
            )
        )

    def compensate_camera_pose(self, pose: RigidTransform) -> RigidTransform:
        """只补偿 Camera 下目标平移，保持目标旋转不变。"""

        if not isinstance(pose, RigidTransform):
            raise TypeError("pose must be a RigidTransform")
        matrix = pose.matrix.copy()
        matrix[:3, 3] = self.compensate_camera_point(
            tuple(float(value) for value in pose.translation_mm)
        )
        return RigidTransform(matrix)

    def compensate_base_pose(self, pose: RigidTransform) -> RigidTransform:
        """只补偿 Base 位姿平移，保持原始旋转不变。"""

        if not isinstance(pose, RigidTransform):
            raise TypeError("pose must be a RigidTransform")
        matrix = pose.matrix.copy()
        matrix[:3, 3] = self.compensate_base_point(
            tuple(float(value) for value in pose.translation_mm)
        )
        return RigidTransform(matrix)


def hand_eye_status(
    calibration: HandEyeCalibration | None,
) -> HandEyeCalibrationStatus:
    if calibration is None:
        return HandEyeCalibrationStatus.MISSING
    if not isinstance(calibration, HandEyeCalibration):
        raise TypeError("calibration must be HandEyeCalibration or None")
    return calibration.status


def hand_eye_from_frame_document(
    document: FrameTransformsDocument,
    *,
    source: str,
) -> HandEyeCalibration | None:
    """读取兼容配置；Base 的 ``metadata.validated`` 不代表手眼有效。"""

    if not isinstance(document, FrameTransformsDocument):
        raise TypeError("document must be a FrameTransformsDocument")
    if not isinstance(source, str) or not source.strip():
        raise ValueError("source must be a non-empty string")
    transform = document.transforms.tool_T_camera
    if transform is None:
        return None
    metadata = document.metadata
    method = metadata.get("tool_camera_method")
    created_at = metadata.get("tool_camera_set_at")
    return HandEyeCalibration(
        tool_T_camera=transform,
        validated=metadata.get("tool_camera_validated") is True,
        source=str(metadata.get("tool_camera_source") or source),
        method=str(method or "unspecified"),
        created_at=created_at if isinstance(created_at, str) else None,
        target_compensation_base_mm=metadata.get(
            "tool_camera_target_compensation_base_mm",
            (0.0, 0.0, 0.0),
        ),
        target_compensation_camera_mm=metadata.get(
            "tool_camera_target_compensation_camera_mm",
            (0.0, 0.0, 0.0),
        ),
    )


def _finite_triplet(
    name: str,
    value: object,
) -> tuple[float, float, float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TypeError(f"{name} must be a sequence of three finite numbers")
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise TypeError(f"{name}[{index}] must be a finite real number")
        converted = float(item)
        if not math.isfinite(converted):
            raise ValueError(f"{name}[{index}] must be finite")
        result.append(converted)
    return result[0], result[1], result[2]


__all__ = [
    "HandEyeCalibration",
    "HandEyeCalibrationStatus",
    "hand_eye_from_frame_document",
    "hand_eye_status",
]

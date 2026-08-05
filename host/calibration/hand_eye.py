"""手眼标定记录及其独立验证状态。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

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

    @property
    def status(self) -> HandEyeCalibrationStatus:
        return (
            HandEyeCalibrationStatus.VALIDATED
            if self.validated
            else HandEyeCalibrationStatus.PROVISIONAL
        )


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
    )


__all__ = [
    "HandEyeCalibration",
    "HandEyeCalibrationStatus",
    "hand_eye_from_frame_document",
    "hand_eye_status",
]

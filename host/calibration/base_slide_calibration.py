"""Base 与 Slide-zero 固定变换的纯数学标定和验证。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from geometry.rigid_transform import RigidTransform, angular_difference_deg


_RECONSTRUCTION_POSITION_TOLERANCE_MM = 1e-7
_RECONSTRUCTION_YAW_TOLERANCE_DEG = 1e-7


@dataclass(frozen=True)
class BaseSlideCalibrationInput:
    base_T_tool_reference: RigidTransform
    slide_zero_T_tool_at_capture: RigidTransform
    expected_slide_yaw_deg: float | None = 0.0
    max_slide_yaw_error_deg: float = 5.0
    max_roll_pitch_deg: float = 1.0

    def __post_init__(self) -> None:
        for field_name in (
            "base_T_tool_reference",
            "slide_zero_T_tool_at_capture",
        ):
            if not isinstance(getattr(self, field_name), RigidTransform):
                raise TypeError(f"{field_name} must be a RigidTransform")
        if self.expected_slide_yaw_deg is not None:
            _require_finite("expected_slide_yaw_deg", self.expected_slide_yaw_deg)
        _require_positive("max_slide_yaw_error_deg", self.max_slide_yaw_error_deg)
        _require_positive("max_roll_pitch_deg", self.max_roll_pitch_deg)


@dataclass(frozen=True)
class BaseSlideCalibrationResult:
    base_T_slide_zero: RigidTransform
    slide_zero_T_base: RigidTransform
    reconstructed_base_T_tool: RigidTransform
    position_residual_mm: float
    yaw_residual_deg: float
    estimated_base_slide_yaw_deg: float
    estimated_base_slide_roll_deg: float
    estimated_base_slide_pitch_deg: float
    expected_base_slide_yaw_deg: float | None
    slide_yaw_alignment_error_deg: float | None
    valid: bool
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class BaseSlideVerificationResult:
    predicted_base_T_tool: RigidTransform
    reference_base_T_tool: RigidTransform
    position_error_xyz_mm: tuple[float, float, float]
    position_error_mm: float
    yaw_error_deg: float
    valid: bool
    warnings: tuple[str, ...]


def calibrate_base_T_slide_zero(
    calibration_input: BaseSlideCalibrationInput,
) -> BaseSlideCalibrationResult:
    """按单一已知 TCP 参考姿态求 ``base_T_slide_zero``。

    单点代数重建接近零是必然结果，不代表机械标定已经由独立姿态验证。
    """

    if not isinstance(calibration_input, BaseSlideCalibrationInput):
        raise TypeError("calibration_input must be BaseSlideCalibrationInput")
    reference = calibration_input.base_T_tool_reference
    captured = calibration_input.slide_zero_T_tool_at_capture
    base_T_slide_zero = reference @ captured.inverse()
    slide_zero_T_base = base_T_slide_zero.inverse()
    reconstructed = base_T_slide_zero @ captured

    position_delta = (
        reconstructed.translation_mm - reference.translation_mm
    )
    position_residual = float(np.linalg.norm(position_delta))
    yaw_residual = abs(
        angular_difference_deg(reconstructed.yaw_deg, reference.yaw_deg)
    )

    roll, pitch, yaw = (float(value) for value in base_T_slide_zero.rpy_deg)
    alignment_error = (
        None
        if calibration_input.expected_slide_yaw_deg is None
        else abs(
            angular_difference_deg(
                yaw,
                calibration_input.expected_slide_yaw_deg,
            )
        )
    )
    warnings: list[str] = []
    if position_residual > _RECONSTRUCTION_POSITION_TOLERANCE_MM:
        warnings.append(
            "algebraic reconstruction position residual exceeds numeric tolerance"
        )
    if yaw_residual > _RECONSTRUCTION_YAW_TOLERANCE_DEG:
        warnings.append(
            "algebraic reconstruction yaw residual exceeds numeric tolerance"
        )
    if alignment_error is not None and (
        alignment_error > calibration_input.max_slide_yaw_error_deg
    ):
        warnings.append(
            "estimated Slide-zero yaw is outside the configured Base +y alignment tolerance"
        )
    if max(abs(roll), abs(pitch)) > calibration_input.max_roll_pitch_deg:
        warnings.append(
            "estimated Base-to-Slide-zero roll/pitch exceeds the yaw-only model tolerance"
        )

    valid = not warnings
    return BaseSlideCalibrationResult(
        base_T_slide_zero=base_T_slide_zero,
        slide_zero_T_base=slide_zero_T_base,
        reconstructed_base_T_tool=reconstructed,
        position_residual_mm=position_residual,
        yaw_residual_deg=yaw_residual,
        estimated_base_slide_yaw_deg=yaw,
        estimated_base_slide_roll_deg=roll,
        estimated_base_slide_pitch_deg=pitch,
        expected_base_slide_yaw_deg=calibration_input.expected_slide_yaw_deg,
        slide_yaw_alignment_error_deg=alignment_error,
        valid=valid,
        warnings=tuple(warnings),
    )


def verify_base_T_slide_zero(
    *,
    base_T_slide_zero: RigidTransform,
    slide_zero_T_tool_at_capture: RigidTransform,
    base_T_tool_reference: RigidTransform,
    max_position_error_mm: float,
    max_yaw_error_deg: float,
) -> BaseSlideVerificationResult:
    """用第二个独立参考姿态验证已保存的固定变换。"""

    for name, value in (
        ("base_T_slide_zero", base_T_slide_zero),
        ("slide_zero_T_tool_at_capture", slide_zero_T_tool_at_capture),
        ("base_T_tool_reference", base_T_tool_reference),
    ):
        if not isinstance(value, RigidTransform):
            raise TypeError(f"{name} must be a RigidTransform")
    _require_positive("max_position_error_mm", max_position_error_mm)
    _require_positive("max_yaw_error_deg", max_yaw_error_deg)

    predicted = base_T_slide_zero @ slide_zero_T_tool_at_capture
    delta = predicted.translation_mm - base_T_tool_reference.translation_mm
    errors = tuple(float(value) for value in delta)
    position_error = float(np.linalg.norm(delta))
    yaw_error = angular_difference_deg(
        predicted.yaw_deg,
        base_T_tool_reference.yaw_deg,
    )
    warnings: list[str] = []
    if position_error > max_position_error_mm:
        warnings.append("position error exceeds configured verification threshold")
    if abs(yaw_error) > max_yaw_error_deg:
        warnings.append("yaw error exceeds configured verification threshold")
    return BaseSlideVerificationResult(
        predicted_base_T_tool=predicted,
        reference_base_T_tool=base_T_tool_reference,
        position_error_xyz_mm=errors,
        position_error_mm=position_error,
        yaw_error_deg=yaw_error,
        valid=not warnings,
        warnings=tuple(warnings),
    )


def _require_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _require_positive(name: str, value: object) -> float:
    converted = _require_finite(name, value)
    if converted <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return converted


__all__ = [
    "BaseSlideCalibrationInput",
    "BaseSlideCalibrationResult",
    "BaseSlideVerificationResult",
    "calibrate_base_T_slide_zero",
    "verify_base_T_slide_zero",
]

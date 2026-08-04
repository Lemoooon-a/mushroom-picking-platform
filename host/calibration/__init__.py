"""纯数学标定与只读状态采集辅助。"""

from calibration.base_slide_calibration import (
    BaseSlideCalibrationInput,
    BaseSlideCalibrationResult,
    BaseSlideVerificationResult,
    calibrate_base_T_slide_zero,
    verify_base_T_slide_zero,
)

__all__ = [
    "BaseSlideCalibrationInput",
    "BaseSlideCalibrationResult",
    "BaseSlideVerificationResult",
    "calibrate_base_T_slide_zero",
    "verify_base_T_slide_zero",
]

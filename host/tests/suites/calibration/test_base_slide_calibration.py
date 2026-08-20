from __future__ import annotations

import unittest

import numpy as np

from calibration.base_slide_calibration import (
    BaseSlideCalibrationInput,
    calibrate_base_T_slide_zero,
    verify_base_T_slide_zero,
)
from geometry.rigid_transform import RigidTransform


class BaseSlideCalibrationTests(unittest.TestCase):
    def _recover(
        self,
        known_base_T_slide_zero: RigidTransform,
        slide_zero_T_tool: RigidTransform | None = None,
        **input_overrides: object,
    ):
        captured = slide_zero_T_tool or RigidTransform.from_xyz_yaw_deg(
            x_mm=20, y_mm=30, z_mm=40, yaw_deg=15
        )
        reference = known_base_T_slide_zero @ captured
        calibration_input = BaseSlideCalibrationInput(
            base_T_tool_reference=reference,
            slide_zero_T_tool_at_capture=captured,
            **input_overrides,
        )
        return calibrate_base_T_slide_zero(calibration_input)

    def test_identity(self) -> None:
        result = self._recover(RigidTransform.identity())
        np.testing.assert_allclose(
            result.base_T_slide_zero.matrix,
            np.eye(4),
            atol=1e-12,
        )
        self.assertTrue(result.valid)

    def test_known_translation(self) -> None:
        known = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=-200, z_mm=300, yaw_deg=0
        )
        result = self._recover(known)
        np.testing.assert_allclose(
            result.base_T_slide_zero.matrix,
            known.matrix,
            rtol=0.0,
            atol=1e-12,
        )

    def test_known_yaw(self) -> None:
        known = RigidTransform.from_xyz_yaw_deg(
            x_mm=0, y_mm=0, z_mm=0, yaw_deg=35
        )
        result = self._recover(known, expected_slide_yaw_deg=35)
        self.assertAlmostEqual(result.estimated_base_slide_yaw_deg, 35)
        self.assertTrue(result.valid)

    def test_known_translation_and_yaw(self) -> None:
        known = RigidTransform.from_xyz_yaw_deg(
            x_mm=12, y_mm=34, z_mm=56, yaw_deg=-78
        )
        result = self._recover(known, expected_slide_yaw_deg=-78)
        np.testing.assert_allclose(result.base_T_slide_zero.matrix, known.matrix)
        np.testing.assert_allclose(
            (result.base_T_slide_zero @ result.slide_zero_T_base).matrix,
            np.eye(4),
            atol=1e-12,
        )

    def test_reconstruction_residual_is_near_zero(self) -> None:
        result = self._recover(RigidTransform.identity())
        self.assertLess(result.position_residual_mm, 1e-10)
        self.assertLess(result.yaw_residual_deg, 1e-10)

    def test_expected_yaw_alignment_passes(self) -> None:
        result = self._recover(
            RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=2
            ),
            max_slide_yaw_error_deg=3,
        )
        self.assertTrue(result.valid)

    def test_expected_yaw_alignment_fails(self) -> None:
        result = self._recover(
            RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=10
            ),
            max_slide_yaw_error_deg=3,
        )
        self.assertFalse(result.valid)
        self.assertRegex(result.warnings[0], "alignment")

    def test_expected_180_handles_wrap(self) -> None:
        result = self._recover(
            RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=-179
            ),
            expected_slide_yaw_deg=179,
            max_slide_yaw_error_deg=3,
        )
        self.assertAlmostEqual(result.slide_yaw_alignment_error_deg, 2)
        self.assertTrue(result.valid)

    def test_roll_pitch_limit_fails_without_projection(self) -> None:
        known = RigidTransform.from_xyz_rpy_deg(
            x_mm=0,
            y_mm=0,
            z_mm=0,
            roll_deg=2,
            pitch_deg=-3,
            yaw_deg=0,
        )
        result = self._recover(known, max_roll_pitch_deg=1)
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.estimated_base_slide_roll_deg, 2)
        self.assertAlmostEqual(result.estimated_base_slide_pitch_deg, -3)

    def test_expected_yaw_check_can_be_disabled(self) -> None:
        result = self._recover(
            RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=90
            ),
            expected_slide_yaw_deg=None,
        )
        self.assertIsNone(result.slide_yaw_alignment_error_deg)
        self.assertTrue(result.valid)


class BaseSlideVerificationTests(unittest.TestCase):
    def test_exact_match(self) -> None:
        base = RigidTransform.from_xyz_yaw_deg(
            x_mm=10, y_mm=20, z_mm=30, yaw_deg=40
        )
        tool = RigidTransform.from_xyz_yaw_deg(
            x_mm=1, y_mm=2, z_mm=3, yaw_deg=4
        )
        result = verify_base_T_slide_zero(
            base_T_slide_zero=base,
            slide_zero_T_tool_at_capture=tool,
            base_T_tool_reference=base @ tool,
            max_position_error_mm=0.1,
            max_yaw_error_deg=0.1,
        )
        self.assertTrue(result.valid)

    def test_position_error_fails(self) -> None:
        result = verify_base_T_slide_zero(
            base_T_slide_zero=RigidTransform.identity(),
            slide_zero_T_tool_at_capture=RigidTransform.identity(),
            base_T_tool_reference=RigidTransform.from_xyz_yaw_deg(
                x_mm=2, y_mm=0, z_mm=0, yaw_deg=0
            ),
            max_position_error_mm=1,
            max_yaw_error_deg=1,
        )
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.position_error_xyz_mm[0], -2)

    def test_yaw_error_fails_and_wraps(self) -> None:
        result = verify_base_T_slide_zero(
            base_T_slide_zero=RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=-179
            ),
            slide_zero_T_tool_at_capture=RigidTransform.identity(),
            base_T_tool_reference=RigidTransform.from_xyz_yaw_deg(
                x_mm=0, y_mm=0, z_mm=0, yaw_deg=179
            ),
            max_position_error_mm=1,
            max_yaw_error_deg=1,
        )
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.yaw_error_deg, 2)


if __name__ == "__main__":
    unittest.main()

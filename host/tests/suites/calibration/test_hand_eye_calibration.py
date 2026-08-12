from __future__ import annotations

import math
import unittest

import numpy as np

from calibration.hand_eye import (
    HandEyeCalibration,
    HandEyeCalibrationStatus,
    hand_eye_from_frame_document,
    hand_eye_status,
)
from config.frame_transforms import FrameTransformsDocument, FixedFrameTransforms
from geometry.rigid_transform import RigidTransform


class HandEyeCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base = RigidTransform.from_xyz_yaw_deg(
            x_mm=1, y_mm=2, z_mm=3, yaw_deg=4
        )
        self.tool_camera = RigidTransform.from_xyz_yaw_deg(
            x_mm=5, y_mm=6, z_mm=7, yaw_deg=8
        )

    def document(
        self,
        *,
        tool: RigidTransform | None,
        metadata: dict[str, object],
    ) -> FrameTransformsDocument:
        return FrameTransformsDocument(
            transforms=FixedFrameTransforms(self.base, tool),
            metadata=metadata,
        )

    def test_missing_transform_is_missing(self) -> None:
        calibration = hand_eye_from_frame_document(
            self.document(tool=None, metadata={"validated": True}),
            source="fixture",
        )
        self.assertIsNone(calibration)
        self.assertIs(hand_eye_status(calibration), HandEyeCalibrationStatus.MISSING)

    def test_base_validation_never_validates_hand_eye(self) -> None:
        calibration = hand_eye_from_frame_document(
            self.document(
                tool=self.tool_camera,
                metadata={"validated": True},
            ),
            source="fixture",
        )
        assert calibration is not None
        self.assertFalse(calibration.validated)
        self.assertIs(calibration.status, HandEyeCalibrationStatus.PROVISIONAL)

    def test_only_specific_tool_camera_flag_marks_validated(self) -> None:
        calibration = hand_eye_from_frame_document(
            self.document(
                tool=self.tool_camera,
                metadata={
                    "validated": False,
                    "tool_camera_validated": True,
                    "tool_camera_source": "synthetic-test",
                    "tool_camera_method": "fixture",
                },
            ),
            source="fallback",
        )
        assert calibration is not None
        self.assertTrue(calibration.validated)
        self.assertEqual(calibration.source, "synthetic-test")
        self.assertIs(calibration.status, HandEyeCalibrationStatus.VALIDATED)
        self.assertEqual(
            calibration.target_compensation_base_mm,
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            calibration.target_compensation_camera_mm,
            (0.0, 0.0, 0.0),
        )

    def test_base_target_compensation_is_loaded_and_preserves_rotation(self) -> None:
        calibration = hand_eye_from_frame_document(
            self.document(
                tool=self.tool_camera,
                metadata={
                    "tool_camera_validated": True,
                    "tool_camera_target_compensation_base_mm": [-10, 10, -10],
                },
            ),
            source="fixture",
        )
        assert calibration is not None

        raw_pose = RigidTransform.from_xyz_rpy_deg(
            x_mm=100,
            y_mm=200,
            z_mm=300,
            roll_deg=12,
            pitch_deg=23,
            yaw_deg=34,
        )
        compensated = calibration.compensate_base_pose(raw_pose)

        self.assertEqual(
            calibration.target_compensation_base_mm,
            (-10.0, 10.0, -10.0),
        )
        np.testing.assert_allclose(
            compensated.translation_mm,
            (90.0, 210.0, 290.0),
        )
        np.testing.assert_allclose(
            compensated.rotation_matrix,
            raw_pose.rotation_matrix,
        )

    def test_camera_target_compensation_is_loaded_and_preserves_rotation(self) -> None:
        calibration = hand_eye_from_frame_document(
            self.document(
                tool=self.tool_camera,
                metadata={
                    "tool_camera_validated": True,
                    "tool_camera_target_compensation_camera_mm": [-5, -20, 10],
                },
            ),
            source="fixture",
        )
        assert calibration is not None
        raw_pose = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=200, z_mm=300, yaw_deg=34
        )
        compensated = calibration.compensate_camera_pose(raw_pose)
        np.testing.assert_allclose(
            compensated.translation_mm,
            (95.0, 180.0, 310.0),
        )
        np.testing.assert_allclose(
            compensated.rotation_matrix,
            raw_pose.rotation_matrix,
        )

    def test_camera_and_base_compensation_cannot_both_be_nonzero(self) -> None:
        with self.assertRaisesRegex(ValueError, "Camera or Base"):
            HandEyeCalibration(
                tool_T_camera=self.tool_camera,
                validated=True,
                source="fixture",
                method="fixture",
                target_compensation_base_mm=(20, 5, -10),
                target_compensation_camera_mm=(-5, -20, 10),
            )

    def test_target_compensation_rejects_invalid_values(self) -> None:
        invalid_values = (
            None,
            "1,2,3",
            (1, 2),
            (1, 2, 3, 4),
            (True, 2, 3),
            ("1", 2, 3),
            (math.inf, 2, 3),
            (math.nan, 2, 3),
        )
        for value in invalid_values:
            with self.subTest(value=value), self.assertRaises((TypeError, ValueError)):
                HandEyeCalibration(
                    tool_T_camera=self.tool_camera,
                    validated=True,
                    source="fixture",
                    method="fixture",
                    target_compensation_base_mm=value,
                )


if __name__ == "__main__":
    unittest.main()

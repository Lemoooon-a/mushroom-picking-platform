from __future__ import annotations

import unittest

from calibration.hand_eye import (
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


if __name__ == "__main__":
    unittest.main()

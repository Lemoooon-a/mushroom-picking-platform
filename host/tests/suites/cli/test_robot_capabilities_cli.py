from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from calibration.hand_eye import HandEyeCalibrationStatus
from config.frame_transforms import FixedFrameTransforms
from geometry.rigid_transform import RigidTransform
from scripts.robot_capabilities import format_capabilities, load_capabilities
from tests.helpers.robot_runtime_config import write_robot_runtime_fixture


class RobotCapabilitiesCliTests(unittest.TestCase):
    @patch("scripts.robot_capabilities.load_robot_five_axis_kinematics")
    def test_missing_hand_eye_is_reported_without_disabling_base(self, _load_geometry) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(RigidTransform.identity(), None),
                metadata={"validated": True},
            )
            capabilities = load_capabilities(path)
        self.assertTrue(capabilities.base_frame_motion)
        self.assertIs(
            capabilities.hand_eye_calibration,
            HandEyeCalibrationStatus.MISSING,
        )
        self.assertFalse(capabilities.vision_target_resolution)
        rendered = "\n".join(format_capabilities(capabilities))
        self.assertIn("Base-frame motion: available", rendered)
        self.assertIn("Hand-eye calibration: missing", rendered)
        self.assertIn("Vision target motion: unavailable", rendered)

    @patch("scripts.robot_capabilities.load_robot_five_axis_kinematics")
    def test_transform_without_validation_is_provisional(self, _load_geometry) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(
                    RigidTransform.identity(),
                    RigidTransform.from_xyz_yaw_deg(
                        x_mm=1, y_mm=2, z_mm=3, yaw_deg=4
                    ),
                ),
                metadata={"validated": True, "tool_camera_validated": False},
            )
            capabilities = load_capabilities(path)
        self.assertIs(
            capabilities.hand_eye_calibration,
            HandEyeCalibrationStatus.PROVISIONAL,
        )
        self.assertFalse(capabilities.vision_target_motion)


if __name__ == "__main__":
    unittest.main()

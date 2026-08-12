from __future__ import annotations

import unittest

import numpy as np

from calibration.hand_eye import HandEyeCalibration
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.observation import CaptureMotionState, VisionTargetObservation
from vision.target_resolver import (
    CaptureStateUnavailable,
    HandEyeCalibrationUnavailable,
    ObservationFrameMismatch,
    VisionTargetResolver,
)


class FakePoseProvider:
    def __init__(self, base_T_tool: RigidTransform) -> None:
        self.base_T_tool = base_T_tool
        self.calls: list[RobotAxisState] = []

    def forward_kinematics_base(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform:
        self.calls.append(axis_state)
        return self.base_T_tool


class VisionTargetResolverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.capture_state = RobotAxisState(1, 2, 3, 4, 5)
        self.base_T_tool = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=200, z_mm=300, yaw_deg=10
        )
        self.tool_T_camera = RigidTransform.from_xyz_rpy_deg(
            x_mm=10,
            y_mm=20,
            z_mm=30,
            roll_deg=1,
            pitch_deg=2,
            yaw_deg=3,
        )
        self.camera_T_target = RigidTransform.from_xyz_rpy_deg(
            x_mm=40,
            y_mm=50,
            z_mm=60,
            roll_deg=4,
            pitch_deg=5,
            yaw_deg=6,
        )
        self.grasp_offset = RigidTransform.from_xyz_rpy_deg(
            x_mm=7,
            y_mm=8,
            z_mm=9,
            roll_deg=10,
            pitch_deg=11,
            yaw_deg=12,
        )
        self.observation = VisionTargetObservation(
            camera_T_target=self.camera_T_target,
            capture_axis_state=self.capture_state,
            frame_id="camera",
            capture_motion_state=CaptureMotionState.STATIONARY,
            timestamp=123.0,
            confidence=0.9,
        )

    def calibration(
        self,
        *,
        validated: bool,
        compensation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> HandEyeCalibration:
        return HandEyeCalibration(
            tool_T_camera=self.tool_T_camera,
            validated=validated,
            source="synthetic-test",
            method="fixture",
            target_compensation_base_mm=compensation,
        )

    def test_validated_synthetic_chain_matches_expected_matrix(self) -> None:
        provider = FakePoseProvider(self.base_T_tool)
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=self.calibration(validated=True),
        )
        actual = resolver.resolve_tool_goal_in_base(
            self.observation,
            self.grasp_offset,
        )
        expected = (
            self.base_T_tool
            @ self.tool_T_camera
            @ self.camera_T_target
            @ self.grasp_offset
        )
        np.testing.assert_allclose(actual.matrix, expected.matrix, atol=1e-12)
        self.assertEqual(provider.calls, [self.capture_state])

    def test_base_compensation_applies_once_before_grasp_offset(self) -> None:
        provider = FakePoseProvider(self.base_T_tool)
        compensation = (-10.0, 10.0, -10.0)
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=self.calibration(
                validated=True,
                compensation=compensation,
            ),
        )

        raw_object = (
            self.base_T_tool @ self.tool_T_camera @ self.camera_T_target
        )
        expected_object_matrix = raw_object.matrix.copy()
        expected_object_matrix[:3, 3] += compensation
        expected_object = RigidTransform(expected_object_matrix)

        actual_object = resolver.resolve_object_in_base(self.observation)
        actual_goal = resolver.resolve_tool_goal_in_base(
            self.observation,
            self.grasp_offset,
        )

        np.testing.assert_allclose(actual_object.matrix, expected_object.matrix)
        np.testing.assert_allclose(
            actual_goal.matrix,
            (expected_object @ self.grasp_offset).matrix,
        )
        np.testing.assert_allclose(
            actual_object.rotation_matrix,
            raw_object.rotation_matrix,
        )

    def test_camera_compensation_applies_before_camera_to_base_transform(self) -> None:
        provider = FakePoseProvider(self.base_T_tool)
        compensation = (-5.0, -20.0, 10.0)
        calibration = HandEyeCalibration(
            tool_T_camera=self.tool_T_camera,
            validated=True,
            source="synthetic-test",
            method="fixture",
            target_compensation_camera_mm=compensation,
        )
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=calibration,
        )
        compensated_camera_target_matrix = self.camera_T_target.matrix.copy()
        compensated_camera_target_matrix[:3, 3] += compensation
        expected = (
            self.base_T_tool
            @ self.tool_T_camera
            @ RigidTransform(compensated_camera_target_matrix)
        )
        actual = resolver.resolve_object_in_base(self.observation)
        np.testing.assert_allclose(actual.matrix, expected.matrix)

    def test_missing_and_provisional_calibration_reject_before_fk(self) -> None:
        for calibration in (None, self.calibration(validated=False)):
            with self.subTest(calibration=calibration):
                provider = FakePoseProvider(self.base_T_tool)
                resolver = VisionTargetResolver(
                    pose_provider=provider,
                    hand_eye_calibration=calibration,
                )
                with self.assertRaisesRegex(
                    HandEyeCalibrationUnavailable,
                    "Base-frame manual motion remains available",
                ):
                    resolver.resolve_tool_goal_in_base(
                        self.observation,
                        self.grasp_offset,
                    )
                self.assertEqual(provider.calls, [])

    def test_moving_or_unknown_capture_rejects_before_fk(self) -> None:
        for motion_state in (CaptureMotionState.MOVING, CaptureMotionState.UNKNOWN):
            with self.subTest(motion_state=motion_state):
                provider = FakePoseProvider(self.base_T_tool)
                resolver = VisionTargetResolver(
                    pose_provider=provider,
                    hand_eye_calibration=self.calibration(validated=True),
                )
                observation = VisionTargetObservation(
                    camera_T_target=self.camera_T_target,
                    capture_axis_state=self.capture_state,
                    frame_id="camera",
                    capture_motion_state=motion_state,
                    timestamp=None,
                )
                with self.assertRaises(CaptureStateUnavailable):
                    resolver.resolve_object_in_base(observation)
                self.assertEqual(provider.calls, [])

    def test_frame_mismatch_rejects(self) -> None:
        provider = FakePoseProvider(self.base_T_tool)
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=self.calibration(validated=True),
        )
        observation = VisionTargetObservation(
            camera_T_target=self.camera_T_target,
            capture_axis_state=self.capture_state,
            frame_id="camera_optical",
            capture_motion_state=CaptureMotionState.STATIONARY,
            timestamp=None,
        )
        with self.assertRaises(ObservationFrameMismatch):
            resolver.resolve_object_in_base(observation)
        self.assertEqual(provider.calls, [])


if __name__ == "__main__":
    unittest.main()

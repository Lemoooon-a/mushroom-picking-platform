from __future__ import annotations

import unittest

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.observation import CaptureMotionState, create_capture_snapshot, require_snapshot_unchanged


class CaptureSnapshotTests(unittest.TestCase):
    def setUp(self) -> None:
        self.state = RobotAxisState(1, 2, 3, 4, 5)

    def test_stationary_snapshot_and_unchanged_check(self) -> None:
        snapshot = create_capture_snapshot(
            request_id="capture-1", axis_state=self.state,
            base_T_tool=RigidTransform.identity(), captured_at=1,
            motion_state=CaptureMotionState.STATIONARY,
        )
        require_snapshot_unchanged(snapshot, axis_state=self.state, motion_state=CaptureMotionState.STATIONARY)

    def test_moving_and_changed_state_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "stationary"):
            create_capture_snapshot(request_id="capture-1", axis_state=self.state, base_T_tool=RigidTransform.identity(), captured_at=1, motion_state=CaptureMotionState.MOVING)
        snapshot = create_capture_snapshot(request_id="capture-1", axis_state=self.state, base_T_tool=RigidTransform.identity(), captured_at=1, motion_state=CaptureMotionState.STATIONARY)
        with self.assertRaisesRegex(ValueError, "changed"):
            require_snapshot_unchanged(snapshot, axis_state=RobotAxisState(2, 2, 3, 4, 5), motion_state=CaptureMotionState.STATIONARY)
        with self.assertRaisesRegex(ValueError, "moved"):
            require_snapshot_unchanged(snapshot, axis_state=self.state, motion_state=CaptureMotionState.MOVING)

    def test_stationary_feedback_quantization_is_allowed_per_axis(self) -> None:
        snapshot = create_capture_snapshot(
            request_id="capture-1",
            axis_state=self.state,
            base_T_tool=RigidTransform.identity(),
            captured_at=1,
            motion_state=CaptureMotionState.STATIONARY,
        )

        require_snapshot_unchanged(
            snapshot,
            axis_state=RobotAxisState(1.005, 2.005, 3.005, 4.005, 5.05),
            motion_state=CaptureMotionState.STATIONARY,
        )

    def test_axis_change_above_quantization_tolerance_is_rejected(self) -> None:
        snapshot = create_capture_snapshot(
            request_id="capture-1",
            axis_state=self.state,
            base_T_tool=RigidTransform.identity(),
            captured_at=1,
            motion_state=CaptureMotionState.STATIONARY,
        )
        changed_states = (
            (RobotAxisState(1.02, 2, 3, 4, 5), "slide_mm"),
            (RobotAxisState(1, 2.02, 3, 4, 5), "z_mm"),
            (RobotAxisState(1, 2, 3.02, 4, 5), "shoulder_deg"),
            (RobotAxisState(1, 2, 3, 4.02, 5), "elbow_deg"),
            (RobotAxisState(1, 2, 3, 4, 5.2), "rotation_deg"),
        )
        for state, axis_name in changed_states:
            with self.subTest(axis=axis_name):
                with self.assertRaisesRegex(ValueError, axis_name):
                    require_snapshot_unchanged(
                        snapshot,
                        axis_state=state,
                        motion_state=CaptureMotionState.STATIONARY,
                    )


if __name__ == "__main__":
    unittest.main()

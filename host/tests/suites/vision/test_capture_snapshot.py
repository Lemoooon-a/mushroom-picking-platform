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


if __name__ == "__main__":
    unittest.main()

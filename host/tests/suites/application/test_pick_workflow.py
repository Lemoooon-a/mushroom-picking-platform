from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from application.controller import MushroomRobotController
from application.grasp_profile import GraspProfile, GraspYawMode
from application.pick_planner import (
    ObservationConfidenceError, ObservationOrientationUnavailable,
    ObservationStaleError, PickPlanner,
)
from application.pick_workflow import (
    NoVisionTarget,
    PickOutcome,
    VisionPickWorkflow,
    VisionWorkflowError,
)
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import HandEyeCalibration
from config.tray_workspace import TrayWorkspaceConfig
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.gateway import FakeVisionGateway
from vision.observation import CaptureMotionState, Quaternion, Vector3, VisionTargetObservation
from vision.protocol import NoTarget, TargetDetection, VisionError
from vision.target_resolver import HandEyeCalibrationUnavailable, VisionTargetResolver


class _Backend:
    def __init__(self) -> None:
        self.calls: list[object] = []
        self.fail_plan_at: int | None = None
        self.fail_execute_at: int | None = None
        self.false_execute_at: int | None = None
        self.fail_suction = False

    def startup(self): self.calls.append("startup")
    def require_base_motion_ready(self): self.calls.append("ready")
    def plan_to_base_pose(self, x, y, z, yaw):
        number = len([item for item in self.calls if isinstance(item, tuple) and item[0] == "plan"]) + 1
        self.calls.append(("plan", x, y, z, yaw))
        if number == self.fail_plan_at: raise ValueError("planning failed")
        return f"motion-{number}"
    def execute_base_plan(self, plan):
        number = len([item for item in self.calls if isinstance(item, tuple) and item[0] == "execute"]) + 1
        self.calls.append(("execute", plan))
        if number == self.fail_execute_at: raise RuntimeError("motion failed")
        if number == self.false_execute_at: return False
        return True
    def return_to_startup(self): self.calls.append("return")
    def stop(self): self.calls.append("stop")
    def enable_joints(self): self.calls.append("enable")
    def disable_joints(self): self.calls.append("disable")
    def suction_grip(self):
        self.calls.append("grip")
        if self.fail_suction: raise RuntimeError("suction failed")
    def suction_release(self): self.calls.append("release")
    def get_status(self): return "status"
    def shutdown(self): self.calls.append("shutdown")


class _PoseProvider:
    def forward_kinematics_base(self, _state):
        return RigidTransform.from_xyz_yaw_deg(x_mm=0, y_mm=0, z_mm=0, yaw_deg=15)


class _SequenceBackend(_Backend):
    def plan_base_sequence(self, targets):
        self.calls.append(("sequence", targets))
        return "overhead-motion", "contact-motion", "lift-motion"


class PickWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 100.0
        self.state = RobotAxisState(1, 2, 3, 4, 5)
        self.backend = _Backend()
        self.provider = _PoseProvider()
        self.workspace = TrayWorkspace(TrayWorkspaceConfig(-100, 100, -100, 100, 0, 100))
        self.calibration = HandEyeCalibration(RigidTransform.identity(), True, "test", "fixture")
        self.controller = self.make_controller(self.calibration)
        self.profile = GraspProfile(0, GraspYawMode.FIXED, 20, 0.8, 2, 2)

    def make_controller(self, calibration):
        return MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
            target_resolver=VisionTargetResolver(
                pose_provider=self.provider,
                hand_eye_calibration=calibration,
                camera_frame_id="camera_optical",
            ),
        )

    def observation(self, *, confidence=0.9, timestamp=100, orientation=None):
        return VisionTargetObservation(
            request_id="capture-1", frame_id="camera_optical", timestamp=timestamp,
            position_mm=Vector3(10, 20, 50), orientation=orientation,
            confidence=confidence, target_id="target",
            capture_axis_state=self.state, capture_motion_state=CaptureMotionState.STATIONARY,
        )

    def planner(self, controller=None):
        return PickPlanner(controller or self.controller, clock=lambda: self.now)

    def workflow(self, result, *, planner=None):
        return VisionPickWorkflow(
            controller=self.controller, gateway=FakeVisionGateway([result]),
            planner=planner or self.planner(),
            capture_state_reader=lambda: (self.state, CaptureMotionState.STATIONARY),
            joints_holding=lambda: True, camera_frame="camera_optical", clock=lambda: self.now,
        )

    def test_planner_builds_all_targets_and_only_contact_uses_tray_gate(self) -> None:
        plan = self.planner().plan(self.observation(), self.profile)
        self.assertEqual(
            (plan.overhead_target.z_mm, plan.contact_target.z_mm, plan.lift_target.z_mm),
            (150, 50, 150),
        )
        self.assertEqual(plan.contact_target.yaw_deg, 20)
        self.assertEqual(len([item for item in self.backend.calls if isinstance(item, tuple) and item[0] == "plan"]), 3)

    def test_planner_preserves_contact_offset(self) -> None:
        profile = GraspProfile(5, GraspYawMode.FIXED, 20, 0.8, 2)
        plan = self.planner().plan(self.observation(), profile)
        self.assertEqual(plan.contact_target.z_mm, 55.0)
        self.assertEqual(plan.overhead_target.z_mm, 150.0)
        self.assertEqual(plan.lift_target.z_mm, 150.0)

    def test_planner_rejects_contact_at_or_above_working_height(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be below working height"):
            self.planner().plan(
                self.observation(),
                GraspProfile(100, GraspYawMode.FIXED, 20, 0.8, 2),
            )

    def test_hand_eye_compensation_precedes_grasp_stage_offsets(self) -> None:
        calibration = HandEyeCalibration(
            RigidTransform.identity(),
            True,
            "test",
            "fixture",
            target_compensation_base_mm=(-10.0, 10.0, -10.0),
        )
        controller = self.make_controller(calibration)

        plan = self.planner(controller).plan(self.observation(), self.profile)
        raw_point = self.provider.forward_kinematics_base(
            self.state
        ).transform_point((10.0, 20.0, 50.0))
        expected_contact = (
            float(raw_point[0]) - 10.0,
            float(raw_point[1]) + 10.0,
            float(raw_point[2]) - 10.0,
        )

        for actual, expected in zip(
            (
                plan.contact_target.x_mm,
                plan.contact_target.y_mm,
                plan.contact_target.z_mm,
            ),
            expected_contact,
            strict=True,
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(
            (
                plan.overhead_target.z_mm,
                plan.contact_target.z_mm,
                plan.lift_target.z_mm,
            ),
            (150.0, 40.0, 150.0),
        )

    def test_any_planning_failure_returns_no_partial_plan(self) -> None:
        self.backend.fail_plan_at = 2
        with self.assertRaisesRegex(ValueError, "planning failed"):
            self.planner().plan(self.observation(), self.profile)
        self.assertFalse(any(isinstance(item, tuple) and item[0] == "execute" for item in self.backend.calls))

    def test_sequence_backend_receives_one_atomic_chained_request(self) -> None:
        backend = _SequenceBackend()
        controller = MushroomRobotController(
            base_backend=backend,
            tray_workspace=self.workspace,
            target_resolver=VisionTargetResolver(
                pose_provider=self.provider,
                hand_eye_calibration=self.calibration,
                camera_frame_id="camera_optical",
            ),
        )
        plan = self.planner(controller).plan(self.observation(), self.profile)
        self.assertEqual(
            (plan.overhead_motion, plan.contact_motion, plan.lift_motion),
            ("overhead-motion", "contact-motion", "lift-motion"),
        )
        self.assertEqual(backend.calls[0], "ready")
        self.assertEqual(backend.calls[1][0], "sequence")

    def test_quality_and_hand_eye_fail_closed(self) -> None:
        with self.assertRaises(ObservationConfidenceError):
            self.planner().plan(self.observation(confidence=0.7), self.profile)
        with self.assertRaises(ObservationStaleError):
            self.planner().plan(self.observation(timestamp=90), self.profile)
        for calibration in (None, HandEyeCalibration(RigidTransform.identity(), False, "test", "fixture")):
            with self.subTest(calibration=calibration), self.assertRaises(HandEyeCalibrationUnavailable):
                self.planner(self.make_controller(calibration)).plan(self.observation(), self.profile)

    def test_observation_age_allows_bounded_positive_and_negative_clock_skew(self) -> None:
        for timestamp in (98.0, 102.0):
            with self.subTest(timestamp=timestamp):
                self.planner().plan(self.observation(timestamp=timestamp), self.profile)

        for timestamp in (97.9, 102.1):
            with self.subTest(timestamp=timestamp), self.assertRaisesRegex(
                ObservationStaleError,
                r"outside \[-2, 2\]s",
            ):
                self.planner().plan(self.observation(timestamp=timestamp), self.profile)

    def test_yaw_modes_and_missing_orientation(self) -> None:
        keep = GraspProfile(0, GraspYawMode.KEEP_CURRENT, None, 0.8, 2)
        self.assertEqual(self.planner().plan(self.observation(), keep).contact_target.yaw_deg, 15)
        observed = GraspProfile(0, GraspYawMode.FROM_OBSERVATION, None, 0.8, 2)
        with self.assertRaises(ObservationOrientationUnavailable):
            self.planner().plan(self.observation(), observed)
        half = math.radians(30) / 2
        plan = self.planner().plan(self.observation(orientation=Quaternion(0, 0, math.sin(half), math.cos(half))), observed)
        self.assertAlmostEqual(plan.contact_target.yaw_deg, 45)

    def test_fake_observation_dry_run_submits_zero_motion(self) -> None:
        detection = TargetDetection("capture-000001", "camera_optical", 100, "target", 0.9, Vector3(10, 20, 50))
        workflow = self.workflow(detection)
        result = workflow.run(self.profile, execute=False)
        self.assertIs(result.outcome, PickOutcome.PLANNED)
        self.assertFalse(any(isinstance(item, tuple) and item[0] == "execute" for item in self.backend.calls))

    def test_no_target_and_state_change_do_not_plan(self) -> None:
        with self.assertRaises(NoVisionTarget):
            self.workflow(NoTarget("capture-000001", "no_detection")).request_observation()
        states = iter(((self.state, CaptureMotionState.STATIONARY), (RobotAxisState(2, 2, 3, 4, 5), CaptureMotionState.STATIONARY)))
        workflow = VisionPickWorkflow(
            controller=self.controller,
            gateway=FakeVisionGateway([TargetDetection("capture-000001", "camera_optical", 100, None, 0.9, Vector3(1, 2, 3))]),
            planner=self.planner(), capture_state_reader=lambda: next(states),
            joints_holding=lambda: True, camera_frame="camera_optical", clock=lambda: self.now,
        )
        with self.assertRaisesRegex(ValueError, "changed"):
            workflow.request_observation()

    def test_vision_error_maps_to_existing_workflow_error(self) -> None:
        with self.assertRaisesRegex(
            VisionWorkflowError,
            "vision error CAMERA_UNAVAILABLE: acquisition failed",
        ):
            self.workflow(
                VisionError(
                    "capture-000001",
                    "CAMERA_UNAVAILABLE",
                    "acquisition failed",
                )
            ).request_observation()

    @patch("application.pick_workflow.time.sleep")
    def test_execute_order_wait_and_physical_pick_remains_unverified(self, sleep) -> None:
        plan = self.planner().plan(self.observation(), self.profile)
        self.backend.calls.clear()
        result = self.workflow(NoTarget("unused", "unused")).execute_pick_plan(plan, execute=True)
        self.assertIs(result.outcome, PickOutcome.PHYSICAL_PICK_UNVERIFIED)
        self.assertEqual(self.backend.calls, [("execute", "motion-1"), ("execute", "motion-2"), "grip", ("execute", "motion-3")])
        sleep.assert_called_once_with(2.0)

    @patch("application.pick_workflow.time.sleep")
    def test_cancellation_during_suction_wait_prevents_lift(self, sleep) -> None:
        plan = self.planner().plan(self.observation(), self.profile)
        self.backend.calls.clear()
        continuing = True

        def cancel_during_wait(_seconds):
            nonlocal continuing
            continuing = False

        sleep.side_effect = cancel_during_wait
        result = self.workflow(NoTarget("unused", "unused")).execute_pick_plan(
            plan,
            execute=True,
            continue_check=lambda: continuing,
        )

        self.assertIs(result.outcome, PickOutcome.FAILED)
        self.assertNotIn(("execute", "motion-3"), self.backend.calls)
        self.assertEqual(self.backend.calls[-1], "stop")

    def test_motion_and_suction_failures_request_stop(self) -> None:
        plan = self.planner().plan(self.observation(), self.profile)
        for motion_failure, suction_failure in ((2, False), (None, True)):
            with self.subTest(motion_failure=motion_failure, suction_failure=suction_failure):
                self.backend.calls.clear()
                self.backend.fail_execute_at = motion_failure
                self.backend.fail_suction = suction_failure
                result = self.workflow(NoTarget("unused", "unused")).execute_pick_plan(plan, execute=True)
                self.assertIs(result.outcome, PickOutcome.FAILED)
                self.assertEqual(self.backend.calls[-1], "stop")

    def test_false_motion_result_stops_pick_before_suction(self) -> None:
        plan = self.planner().plan(self.observation(), self.profile)
        self.backend.calls.clear()
        self.backend.false_execute_at = 2

        result = self.workflow(NoTarget("unused", "unused")).execute_pick_plan(
            plan,
            execute=True,
        )

        self.assertIs(result.outcome, PickOutcome.FAILED)
        self.assertNotIn("grip", self.backend.calls)
        self.assertEqual(self.backend.calls[-1], "stop")


if __name__ == "__main__":
    unittest.main()

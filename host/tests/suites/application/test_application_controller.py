from __future__ import annotations

import unittest

from application.controller import (
    BaseMotionExecutionError,
    MushroomRobotController,
    UnsupportedToolGoalOrientationError,
)
from application.tray_workspace import TargetOutsideTrayWorkspace, TrayWorkspace
from calibration.hand_eye import HandEyeCalibration, HandEyeCalibrationStatus
from config.tray_workspace import TrayWorkspaceConfig
from geometry.rigid_transform import RigidTransform
from kinematics.base_frame_solver import FiveAxisNoSolutionError
from kinematics.frame_chain import RobotAxisState
from vision.observation import CaptureMotionState, VisionTargetObservation
from vision.target_resolver import HandEyeCalibrationUnavailable, VisionTargetResolver


class FakeBaseBackend:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.plan_error: Exception | None = None
        self.solver_calls = 0
        self.planner_calls = 0
        self.submit_calls = 0
        self.execute_result: object = "moved"

    def startup(self) -> str:
        self.calls.append(("startup",))
        return "started"

    def require_base_motion_ready(self) -> None:
        self.calls.append(("preflight",))

    def plan_to_base_pose(self, x: float, y: float, z: float, yaw: float) -> str:
        self.calls.append(("plan", x, y, z, yaw))
        self.solver_calls += 1
        if self.plan_error is not None:
            raise self.plan_error
        self.planner_calls += 1
        return "planned"

    def execute_base_plan(self, plan: object) -> object:
        self.calls.append(("execute", plan))
        self.submit_calls += 1
        return self.execute_result

    def return_to_startup(self) -> str:
        self.calls.append(("return",))
        return "returned"

    def stop(self) -> str:
        self.calls.append(("stop",))
        return "stopped"

    def enable_joints(self) -> str:
        self.calls.append(("enable",))
        return "enabled"

    def disable_joints(self) -> str:
        self.calls.append(("disable",))
        return "disabled"

    def suction_grip(self) -> str:
        self.calls.append(("grip",))
        return "gripped"

    def suction_release(self) -> str:
        self.calls.append(("release",))
        return "released"

    def get_status(self) -> str:
        self.calls.append(("status",))
        return "backend-status"

    def shutdown(self) -> str:
        self.calls.append(("shutdown",))
        return "shutdown"


class FakePoseProvider:
    def __init__(self, pose: RigidTransform) -> None:
        self.pose = pose
        self.calls = 0

    def forward_kinematics_base(self, axis_state: RobotAxisState) -> RigidTransform:
        self.calls += 1
        return self.pose


class ApplicationControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = FakeBaseBackend()
        self.workspace = TrayWorkspace(
            TrayWorkspaceConfig(
                x_min_mm=-1000,
                x_max_mm=1000,
                y_min_mm=-1000,
                y_max_mm=1000,
                z_min_mm=-1000,
                z_max_mm=1000,
            )
        )
        self.capture_state = RobotAxisState(1, 2, 3, 4, 5)
        self.observation = VisionTargetObservation(
            camera_T_target=RigidTransform.from_xyz_yaw_deg(
                x_mm=10, y_mm=20, z_mm=30, yaw_deg=4
            ),
            capture_axis_state=self.capture_state,
            frame_id="camera",
            capture_motion_state=CaptureMotionState.STATIONARY,
            timestamp=1.0,
        )
        self.grasp = RigidTransform.from_xyz_yaw_deg(
            x_mm=1, y_mm=2, z_mm=3, yaw_deg=5
        )

    def resolver(self, *, validated: bool) -> tuple[VisionTargetResolver, FakePoseProvider]:
        provider = FakePoseProvider(
            RigidTransform.from_xyz_yaw_deg(
                x_mm=100, y_mm=200, z_mm=300, yaw_deg=6
            )
        )
        calibration = HandEyeCalibration(
            tool_T_camera=RigidTransform.from_xyz_yaw_deg(
                x_mm=7, y_mm=8, z_mm=9, yaw_deg=10
            ),
            validated=validated,
            source="synthetic-test",
            method="fixture",
        )
        return (
            VisionTargetResolver(
                pose_provider=provider,
                hand_eye_calibration=calibration,
            ),
            provider,
        )

    def test_base_frame_methods_remain_available_without_hand_eye(self) -> None:
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
        )
        self.assertEqual(controller.plan_to_base_pose(1, 2, 3, 4), "planned")
        self.assertEqual(controller.move_to_base_pose(5, 6, 7, 8), "moved")
        self.assertEqual(
            [call[0] for call in self.backend.calls],
            ["preflight", "plan", "preflight", "plan", "execute"],
        )
        self.assertTrue(controller.capabilities.base_frame_motion)
        self.assertIs(
            controller.capabilities.hand_eye_calibration,
            HandEyeCalibrationStatus.MISSING,
        )
        self.assertFalse(controller.capabilities.vision_target_motion)

    def test_missing_or_provisional_gate_never_calls_base_backend(self) -> None:
        controllers = [
            MushroomRobotController(
                base_backend=self.backend,
                tray_workspace=self.workspace,
            )
        ]
        provisional, provider = self.resolver(validated=False)
        controllers.append(
            MushroomRobotController(
                base_backend=self.backend,
                tray_workspace=self.workspace,
                target_resolver=provisional,
            )
        )
        for controller in controllers:
            with self.subTest(status=controller.capabilities.hand_eye_calibration):
                self.backend.calls.clear()
                with self.assertRaises(HandEyeCalibrationUnavailable):
                    controller.plan_to_observation(self.observation, self.grasp)
                with self.assertRaises(HandEyeCalibrationUnavailable):
                    controller.move_to_observation(self.observation, self.grasp)
                self.assertEqual(self.backend.calls, [])
        self.assertEqual(provider.calls, 0)

    def test_validated_observation_reuses_base_plan_and_move_entrypoints(self) -> None:
        resolver, provider = self.resolver(validated=True)
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
            target_resolver=resolver,
        )
        self.assertEqual(controller.plan_to_observation(self.observation, self.grasp), "planned")
        self.assertEqual(controller.move_to_observation(self.observation, self.grasp), "moved")
        self.assertEqual(
            [call[0] for call in self.backend.calls],
            ["preflight", "plan", "preflight", "plan", "execute"],
        )
        self.assertEqual(provider.calls, 2)
        self.assertTrue(controller.capabilities.vision_target_resolution)
        self.assertTrue(controller.capabilities.vision_target_motion)

    def test_base_workspace_rejection_propagates_without_motion(self) -> None:
        resolver, _provider = self.resolver(validated=True)
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
            target_resolver=resolver,
        )
        self.backend.plan_error = ValueError("outside tray workspace")
        with self.assertRaisesRegex(ValueError, "outside tray workspace"):
            controller.plan_to_observation(self.observation, self.grasp)
        self.assertEqual(
            [call[0] for call in self.backend.calls],
            ["preflight", "plan"],
        )

    def test_non_yaw_goal_is_not_silently_flattened(self) -> None:
        provider = FakePoseProvider(RigidTransform.identity())
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=HandEyeCalibration(
                tool_T_camera=RigidTransform.identity(),
                validated=True,
                source="synthetic-test",
                method="fixture",
            ),
        )
        observation = VisionTargetObservation(
            camera_T_target=RigidTransform.from_xyz_rpy_deg(
                x_mm=1,
                y_mm=2,
                z_mm=3,
                roll_deg=5,
                pitch_deg=0,
                yaw_deg=0,
            ),
            capture_axis_state=self.capture_state,
            frame_id="camera",
            capture_motion_state=CaptureMotionState.STATIONARY,
            timestamp=None,
        )
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
            target_resolver=resolver,
        )
        with self.assertRaises(UnsupportedToolGoalOrientationError):
            controller.plan_to_observation(observation, RigidTransform.identity())
        self.assertEqual(self.backend.calls, [])

    def test_outside_base_target_is_rejected_before_backend(self) -> None:
        workspace = TrayWorkspace(
            TrayWorkspaceConfig(0, 10, 20, 30, 40, 50)
        )
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=workspace,
        )
        with self.assertRaises(TargetOutsideTrayWorkspace) as captured:
            controller.move_to_base_pose(11, 25, 45, 0)
        self.assertIn("Failed dimensions:\n  x", str(captured.exception))
        self.assertEqual(self.backend.calls, [("preflight",)])
        self.assertEqual(self.backend.solver_calls, 0)
        self.assertEqual(self.backend.planner_calls, 0)
        self.assertEqual(self.backend.submit_calls, 0)

    def test_inside_but_kinematically_unreachable_error_stays_distinct(self) -> None:
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
        )
        self.backend.plan_error = FiveAxisNoSolutionError(
            "kinematically unreachable",
            stage="planar_ik",
            stage_counts={"planar_unreachable": 1},
        )
        with self.assertRaisesRegex(
            FiveAxisNoSolutionError,
            "kinematically unreachable",
        ) as captured:
            controller.plan_to_base_pose(1, 2, 3, 4)
        self.assertNotIsInstance(captured.exception, TargetOutsideTrayWorkspace)
        self.assertEqual(
            [call[0] for call in self.backend.calls],
            ["preflight", "plan"],
        )

    def test_startup_exception_is_not_available_through_normal_move(self) -> None:
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(0, 10, 20, 30, 40, 50)
            ),
        )
        self.assertEqual(controller.startup(), "started")
        self.assertEqual(controller.return_to_startup(), "returned")
        with self.assertRaises(TargetOutsideTrayWorkspace):
            controller.move_to_base_pose(200, 0, 180, 0)
        self.assertEqual(
            [call[0] for call in self.backend.calls],
            ["startup", "return", "preflight"],
        )

    def test_literal_false_execution_result_is_an_explicit_failure(self) -> None:
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=self.workspace,
        )
        self.backend.execute_result = False

        with self.assertRaisesRegex(
            BaseMotionExecutionError,
            "plan execution failed",
        ):
            controller.execute_base_plan("plan")
        with self.assertRaises(BaseMotionExecutionError):
            controller.move_to_base_pose(1, 2, 3, 4)

        self.assertEqual(self.backend.submit_calls, 2)

    def test_validated_vision_target_outside_workspace_never_calls_backend(self) -> None:
        resolver, provider = self.resolver(validated=True)
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(-10, 10, -10, 10, -10, 10)
            ),
            target_resolver=resolver,
        )
        with self.assertRaises(TargetOutsideTrayWorkspace):
            controller.move_to_observation(self.observation, self.grasp)
        self.assertEqual(provider.calls, 1)
        self.assertEqual(self.backend.calls, [("preflight",)])
        self.assertEqual(self.backend.solver_calls, 0)
        self.assertEqual(self.backend.planner_calls, 0)
        self.assertEqual(self.backend.submit_calls, 0)

    def test_tolerance_does_not_clamp_forwarded_target(self) -> None:
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(0, 10, 20, 30, 40, 50, 1e-3)
            ),
        )
        self.assertEqual(controller.plan_to_base_pose(-0.0005, 25, 45, 0), "planned")
        self.assertEqual(
            self.backend.calls,
            [("preflight",), ("plan", -0.0005, 25, 45, 0)],
        )


if __name__ == "__main__":
    unittest.main()

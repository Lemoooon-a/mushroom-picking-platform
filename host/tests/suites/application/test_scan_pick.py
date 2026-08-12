from __future__ import annotations

import unittest
from unittest.mock import Mock

from application.controller import MushroomRobotController
from application.grasp_profile import GraspProfile, GraspYawMode
from application.motion_target import BaseToolTarget
from application.pick_planner import PickPlanner
from application.pick_workflow import VisionPickWorkflow
from application.robot_service import MushroomRobotService
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.scan_pick import ScanPickProfile
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import HandEyeCalibration
from config.tray_workspace import TrayWorkspaceConfig
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.gateway import FakeVisionGateway
from vision.observation import CaptureMotionState, Vector3
from vision.protocol import NoTarget, TargetDetection
from vision.target_resolver import VisionTargetResolver


class _ScanBackend:
    def __init__(self) -> None:
        self.pose = BaseToolTarget(0.0, 0.0, 0.0, 0.0)
        self.calls: list[object] = []
        self.grip_error: Exception | None = None

    def startup(self) -> None:
        self.calls.append("startup")

    def require_base_motion_ready(self) -> None:
        return None

    def plan_to_base_pose(self, x, y, z, yaw):
        target = BaseToolTarget(x, y, z, yaw)
        self.calls.append(("plan", target))
        return target

    def plan_base_sequence(self, targets):
        self.calls.append(("plan_sequence", targets))
        return targets

    def execute_base_plan(self, plan):
        self.calls.append(("execute", plan))
        self.pose = plan
        return True

    def suction_grip(self):
        self.calls.append("grip")
        if self.grip_error is not None:
            raise self.grip_error

    def suction_release(self):
        self.calls.append("release")

    def return_to_startup(self):
        self.calls.append("return")

    def enable_joints(self):
        self.calls.append("enable")

    def disable_joints(self):
        self.calls.append("disable")

    def stop(self):
        self.calls.append("stop")

    def shutdown(self):
        self.calls.append("shutdown")

    def get_status(self):
        return None


class _ScanPoseProvider:
    def __init__(self, backend: _ScanBackend) -> None:
        self.backend = backend

    def forward_kinematics_base(self, axis_state: RobotAxisState) -> RigidTransform:
        del axis_state
        pose = self.backend.pose
        return RigidTransform.from_xyz_yaw_deg(
            x_mm=pose.x_mm,
            y_mm=pose.y_mm,
            z_mm=pose.z_mm,
            yaw_deg=pose.yaw_deg or 0.0,
        )


class ScanAndPickTests(unittest.TestCase):
    def make_service(
        self,
        responder,
        *,
        max_picks: int = 5,
        scan_settle_time_s: float = 0.0,
    ) -> tuple[MushroomRobotService, _ScanBackend, list[BaseToolTarget]]:
        backend = _ScanBackend()
        provider = _ScanPoseProvider(backend)
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=HandEyeCalibration(
                RigidTransform.identity(),
                True,
                "test",
                "test",
                target_compensation_base_mm=(10.0, 0.0, 0.0),
            ),
            camera_frame_id="camera_color_optical_frame",
        )
        controller = MushroomRobotController(
            base_backend=backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(-1000, 1000, -1000, 1000, -1000, 1000)
            ),
            target_resolver=resolver,
        )
        observed_poses: list[BaseToolTarget] = []

        def record_pose_and_respond(request):
            observed_poses.append(backend.pose)
            return responder(request)

        workflow = VisionPickWorkflow(
            controller=controller,
            gateway=FakeVisionGateway(responder=record_pose_and_respond),
            planner=PickPlanner(controller, clock=lambda: 100.0),
            capture_state_reader=lambda: (
                RobotAxisState(0.0, 0.0, 0.0, 0.0, 0.0),
                CaptureMotionState.STATIONARY,
            ),
            joints_holding=lambda: True,
            camera_frame="camera_color_optical_frame",
            clock=lambda: 100.0,
        )
        service = MushroomRobotService(
            controller=controller,
            workflow=workflow,
            mode=RobotServiceMode.DRY_RUN,
            grasp_profile=GraspProfile(
                20.0,
                0.0,
                30.0,
                GraspYawMode.FIXED,
                0.0,
                0.8,
                2.0,
            ),
            scan_pick_profile=ScanPickProfile(
                (0.0, 100.0),
                (0.0, 10.0, 20.0, 30.0),
                50.0,
                0.0,
                BaseToolTarget(500.0, 500.0, 20.0, 0.0),
                40.0,
                max_picks,
                scan_settle_time_s,
            ),
            allow_dry_run_state_advance=True,
        )
        service.startup()
        return service, backend, observed_poses

    @staticmethod
    def target(request):
        return TargetDetection(
            request.request_id,
            request.camera_frame,
            request.timestamp,
            "target",
            0.95,
            Vector3(1.0, 2.0, 3.0),
            None,
        )

    @unittest.mock.patch("application.robot_service.time.sleep")
    def test_scans_eight_poses_and_reobserves_after_each_place(self, sleep) -> None:
        calls = 0

        def responder(request):
            nonlocal calls
            calls += 1
            if calls <= 2:
                return self.target(request)
            return NoTarget(request.request_id, "empty")

        service, backend, observed_poses = self.make_service(
            responder, scan_settle_time_s=0.5
        )
        original_begin = service._begin_write_operation
        service._begin_write_operation = Mock(wraps=original_begin)

        result = service.scan_and_pick()

        self.assertEqual(result.result, "completed")
        self.assertEqual(result.total_picked, 2)
        self.assertEqual(len(result.visited_scan_positions), 8)
        self.assertEqual(
            (result.visited_scan_positions[0].detected_count,
             result.visited_scan_positions[0].picked_count,
             result.visited_scan_positions[0].final_reason),
            (2, 2, "no_target"),
        )
        expected_scans = service.scan_pick_profile.scan_poses
        self.assertEqual(
            observed_poses,
            [expected_scans[0], expected_scans[0], expected_scans[0], *expected_scans[1:]],
        )
        self.assertEqual(service._begin_write_operation.call_count, 1)
        self.assertEqual(
            service._begin_write_operation.call_args.kwargs["kind"],
            "scan-pick",
        )
        self.assertIs(service.state, RobotServiceState.READY)

        planned_targets = [
            target
            for entry in backend.calls
            if isinstance(entry, tuple) and entry[0] == "plan_sequence"
            for target in entry[1]
        ]
        first_contact = next(target for target in planned_targets if target.z_mm == 53.0)
        self.assertEqual(first_contact.x_mm, 11.0)
        self.assertTrue(all(target.yaw_deg == 0.0 for target in planned_targets))
        self.assertEqual(backend.calls.count("grip"), 2)
        self.assertEqual(backend.calls.count("release"), 2)
        self.assertEqual(sleep.call_count, len(observed_poses))
        sleep.assert_called_with(0.5)

    def test_max_pick_guard_stops_entire_task_without_fault(self) -> None:
        service, _, observed_poses = self.make_service(self.target, max_picks=2)

        result = service.scan_and_pick()

        self.assertEqual(result.result, "stopped_max_picks_per_scan_pose")
        self.assertEqual(result.total_picked, 2)
        self.assertEqual(len(result.visited_scan_positions), 1)
        self.assertEqual(
            result.visited_scan_positions[0].final_reason,
            "max_picks_per_scan_pose_reached",
        )
        self.assertEqual(len(observed_poses), 2)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_motion_or_suction_failure_faults_and_stops(self) -> None:
        service, backend, _ = self.make_service(self.target)
        backend.grip_error = RuntimeError("suction failed")

        with self.assertRaisesRegex(Exception, "suction failed"):
            service.scan_and_pick()

        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", backend.calls)
        self.assertNotIn("release", backend.calls)


if __name__ == "__main__":
    unittest.main()

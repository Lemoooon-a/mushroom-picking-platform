from __future__ import annotations

import unittest
from unittest.mock import Mock

from application.controller import MushroomRobotController
from application.grasp_profile import GraspProfile, GraspYawMode
from application.motion_target import BaseToolTarget
from application.offline_backend import create_offline_planning_controller
from application.pick_planner import PickPlanner
from application.pick_workflow import VisionPickWorkflow
from application.robot_service import MushroomRobotService, RobotServiceCapabilityError
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.scan_pick import ScanPickProfile
from application.tray_workspace import TargetOutsideTrayWorkspace, TrayWorkspace
from calibration.hand_eye import HandEyeCalibration
from config.tray_workspace import TrayWorkspaceConfig
from config.robot_runtime import load_robot_runtime_config
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.gateway import FakeVisionGateway
from vision.observation import CaptureMotionState, Vector3
from vision.protocol import NoTarget, TargetDetection
from vision.target_size import TargetSizeClass
from vision.target_resolver import VisionTargetResolver


class _ScanBackend:
    def __init__(self) -> None:
        self.pose = BaseToolTarget(0.0, 0.0, 0.0, 0.0)
        self.calls: list[object] = []
        self.grip_error: Exception | None = None
        self.fail_sequence_target: BaseToolTarget | None = None

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
        if self.fail_sequence_target in targets:
            raise ValueError("placement planning failed")
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
                target_compensation_base_mm=(10.0, 0.0, -100.0),
            ),
            camera_frame_id="camera_color_optical_frame",
        )
        controller = MushroomRobotController(
            base_backend=backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(-1000, 1000, -1000, 780, -1000, 1000)
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
                0.0,
                GraspYawMode.FIXED,
                0.0,
                0.8,
                2.0,
                0.0,
            ),
            scan_pick_profile=ScanPickProfile(
                (0.0, 100.0),
                (0.0, 10.0),
                0.0,
                BaseToolTarget(150.0, 1000.0, 200.0, 0.0),
                BaseToolTarget(450.0, 1000.0, 200.0, 0.0),
                max_picks,
                scan_settle_time_s,
            ),
            allow_dry_run_state_advance=True,
        )
        service.startup()
        return service, backend, observed_poses

    @staticmethod
    def target(request, *, size_class=TargetSizeClass.NORMAL):
        return TargetDetection(
            request.request_id,
            request.camera_frame,
            request.timestamp,
            "target",
            0.95,
            Vector3(1.0, 2.0, 3.0),
            None,
            size_class=size_class,
        )

    @unittest.mock.patch("application.robot_service.time.sleep")
    def test_scans_four_poses_and_reobserves_after_each_place(self, sleep) -> None:
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
        self.assertEqual(len(result.visited_scan_positions), 4)
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
        first_contact = next(target for target in planned_targets if target.z_mm == 103.0)
        self.assertEqual(first_contact.x_mm, 11.0)
        self.assertTrue(all(target.yaw_deg == 0.0 for target in planned_targets))
        self.assertEqual(backend.calls.count("grip"), 2)
        self.assertEqual(backend.calls.count("release"), 2)
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list].count(0.5),
            len(observed_poses),
        )
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list].count(0.0),
            2,
        )

    def test_scan_position_indices_reuse_configured_pose_order(self) -> None:
        service, backend, _ = self.make_service(
            lambda request: NoTarget(request.request_id, "empty")
        )
        expected_scans = service.scan_pick_profile.scan_poses

        for scan_index, expected_pose in enumerate(expected_scans, start=1):
            backend.calls.clear()
            result = service.move_to_scan_position(scan_index)

            self.assertFalse(result.executed)
            self.assertEqual(result.plan, expected_pose)
            self.assertEqual(backend.calls, [("plan", expected_pose)])
            self.assertIs(service.state, RobotServiceState.READY)

        for invalid_index in (0, 5, True, 1.0):
            with self.subTest(scan_index=invalid_index):
                with self.assertRaises((TypeError, ValueError)):
                    service.move_to_scan_position(invalid_index)

    def test_scan_position_requires_validated_scan_profile(self) -> None:
        service, _, _ = self.make_service(
            lambda request: NoTarget(request.request_id, "empty")
        )
        service.scan_pick_profile = None

        with self.assertRaises(RobotServiceCapabilityError):
            service.move_to_scan_position(1)
        with self.assertRaises(RobotServiceCapabilityError):
            service.pick_one_at_scan_position(1)

    def test_pick_one_moves_picks_places_and_returns(self) -> None:
        service, backend, observed_poses = self.make_service(self.target)
        expected_scan = service.scan_pick_profile.scan_poses[2]
        original_begin = service._begin_write_operation
        service._begin_write_operation = Mock(wraps=original_begin)
        original_plan_sequence = service._controller.plan_base_target_sequence
        service._controller.plan_base_target_sequence = Mock(
            wraps=original_plan_sequence
        )

        result = service.pick_one_at_scan_position(3)

        self.assertEqual(result.result, "completed")
        self.assertEqual(result.total_picked, 1)
        self.assertEqual(len(result.visited_scan_positions), 1)
        position = result.visited_scan_positions[0]
        self.assertEqual(
            (
                position.scan_index,
                position.detected_count,
                position.picked_count,
                position.final_reason,
            ),
            (3, 1, 1, "picked_and_placed_unverified"),
        )
        self.assertEqual(observed_poses, [expected_scan])
        self.assertEqual(backend.pose, expected_scan)
        self.assertEqual(backend.calls.count("grip"), 1)
        self.assertEqual(backend.calls.count("release"), 1)
        place_pose = service.scan_pick_profile.place_pose
        service._controller.plan_base_target_sequence.assert_any_call(
            (place_pose, expected_scan),
            enforce_tray_workspace=(False, True),
        )
        place_sequence = next(
            entry
            for entry in backend.calls
            if isinstance(entry, tuple)
            and entry[0] == "plan_sequence"
            and entry[1] == (place_pose, expected_scan)
        )
        place_plan_index = backend.calls.index(place_sequence)
        place_execute_index = backend.calls.index(
            ("execute", place_pose),
            place_plan_index,
        )
        release_index = backend.calls.index("release", place_execute_index)
        return_execute_index = backend.calls.index(
            ("execute", expected_scan),
            release_index,
        )
        self.assertLess(place_plan_index, place_execute_index)
        self.assertLess(place_execute_index, release_index)
        self.assertLess(release_index, return_execute_index)
        self.assertEqual(
            [
                entry
                for entry in backend.calls[place_execute_index:return_execute_index + 1]
                if isinstance(entry, tuple) and entry[0] == "execute"
            ],
            [("execute", place_pose), ("execute", expected_scan)],
        )
        self.assertEqual(service._begin_write_operation.call_count, 1)
        self.assertEqual(
            service._begin_write_operation.call_args.kwargs["kind"],
            "scan-pick-one",
        )
        self.assertIs(service.state, RobotServiceState.READY)

    def test_pick_one_routes_oversized_target_to_oversized_place_pose(self) -> None:
        service, backend, _ = self.make_service(
            lambda request: self.target(
                request,
                size_class=TargetSizeClass.OVERSIZED,
            )
        )
        expected_scan = service.scan_pick_profile.scan_poses[0]
        oversized_place = service.scan_pick_profile.oversized_place_pose

        result = service.pick_one_at_scan_position(1)

        self.assertEqual(result.total_picked, 1)
        self.assertIn(("execute", oversized_place), backend.calls)
        self.assertNotIn(("execute", service.scan_pick_profile.place_pose), backend.calls)
        self.assertEqual(backend.pose, expected_scan)

    def test_scan_pick_routes_oversized_target_then_reobserves(self) -> None:
        calls = 0

        def responder(request):
            nonlocal calls
            calls += 1
            if calls == 1:
                return self.target(
                    request,
                    size_class=TargetSizeClass.OVERSIZED,
                )
            return NoTarget(request.request_id, "empty")

        service, backend, _ = self.make_service(responder)

        result = service.scan_and_pick()

        self.assertEqual(result.total_picked, 1)
        self.assertIn(
            ("execute", service.scan_pick_profile.oversized_place_pose),
            backend.calls,
        )
        self.assertNotIn(
            ("execute", service.scan_pick_profile.place_pose),
            backend.calls,
        )

    def test_oversized_place_planning_failure_stops_without_release(self) -> None:
        service, backend, _ = self.make_service(
            lambda request: self.target(
                request,
                size_class=TargetSizeClass.OVERSIZED,
            )
        )
        backend.fail_sequence_target = service.scan_pick_profile.oversized_place_pose

        with self.assertRaisesRegex(ValueError, "placement planning failed"):
            service.pick_one_at_scan_position(1)

        self.assertNotIn("release", backend.calls)
        self.assertIn("stop", backend.calls)
        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_fixed_place_is_the_only_tray_workspace_exception(self) -> None:
        service, backend, _ = self.make_service(self.target)
        place_pose = service.scan_pick_profile.place_pose

        with self.assertRaises(TargetOutsideTrayWorkspace):
            service._controller.plan_base_target(place_pose)

        result = service.pick_one_at_scan_position(1)

        self.assertEqual(result.total_picked, 1)
        self.assertIn(("execute", place_pose), backend.calls)

    def test_pick_one_no_target_is_normal_and_stays_at_scan_pose(self) -> None:
        service, backend, observed_poses = self.make_service(
            lambda request: NoTarget(request.request_id, "empty")
        )
        expected_scan = service.scan_pick_profile.scan_poses[3]

        result = service.pick_one_at_scan_position(4)

        self.assertEqual(result.result, "completed")
        self.assertEqual(result.total_picked, 0)
        position = result.visited_scan_positions[0]
        self.assertEqual(
            (
                position.scan_index,
                position.detected_count,
                position.picked_count,
                position.final_reason,
            ),
            (4, 0, 0, "no_target"),
        )
        self.assertEqual(observed_poses, [expected_scan])
        self.assertEqual(backend.pose, expected_scan)
        self.assertNotIn("grip", backend.calls)
        self.assertNotIn("release", backend.calls)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_pick_one_target_rejection_is_normal(self) -> None:
        service, backend, _ = self.make_service(self.target)
        service._workflow.plan_observation = Mock(
            side_effect=ValueError("target rejected")
        )

        result = service.pick_one_at_scan_position(1)

        self.assertEqual(result.total_picked, 0)
        self.assertEqual(
            result.visited_scan_positions[0].final_reason,
            "target_rejected:ValueError",
        )
        self.assertNotIn("grip", backend.calls)
        self.assertNotIn("release", backend.calls)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_pick_one_motion_or_suction_failure_faults_and_stops(self) -> None:
        service, backend, _ = self.make_service(self.target)
        backend.grip_error = RuntimeError("suction failed")

        with self.assertRaisesRegex(Exception, "suction failed"):
            service.pick_one_at_scan_position(1)

        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", backend.calls)
        self.assertNotIn("release", backend.calls)

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


class ScanPickOfflineReachabilityTests(unittest.TestCase):
    def test_both_place_poses_round_trip_from_all_scan_positions(self) -> None:
        runtime_config = load_robot_runtime_config()
        controller, backend = create_offline_planning_controller(
            runtime_config=runtime_config
        )
        backend.startup()
        assert backend.axis_state is not None
        startup_pose = backend.solver.forward_kinematics_base(backend.axis_state)
        self.assertEqual(
            tuple(round(float(value), 9) for value in startup_pose.translation_mm),
            (400.0, 150.0, 200.0),
        )
        self.assertAlmostEqual(startup_pose.yaw_deg, 0.0)
        self.assertAlmostEqual(backend.axis_state.slide_mm, 0.0)
        self.assertAlmostEqual(backend.axis_state.z_mm, 0.0)
        self.assertAlmostEqual(backend.axis_state.shoulder_deg, -37.16781965284249)
        self.assertAlmostEqual(backend.axis_state.elbow_deg, 115.44772974485191)
        self.assertAlmostEqual(backend.axis_state.rotation_deg, -78.27991009200943)
        startup_status, _, _ = backend.solver.workspace_status_for_state(
            backend.axis_state
        )
        self.assertEqual(startup_status.value, "inside")
        place_poses = (
            runtime_config.scan_pick.place_pose,
            runtime_config.scan_pick.oversized_place_pose,
        )

        for place_pose in place_poses:
            with self.assertRaises(TargetOutsideTrayWorkspace):
                controller.plan_base_target(place_pose)

            for scan_index, scan_pose in enumerate(
                runtime_config.scan_pick.scan_poses,
                start=1,
            ):
                with self.subTest(place_x=place_pose.x_mm, scan_index=scan_index):
                    controller.execute_base_plan(
                        controller.plan_base_target(scan_pose)
                    )
                    place_plan, return_plan = controller.plan_base_target_sequence(
                        (place_pose, scan_pose),
                        enforce_tray_workspace=(False, True),
                    )
                    self.assertTrue(
                        all(
                            abs(float(stage.base_T_tool_target.translation_mm[2]) - 200.0)
                            <= 1e-9
                            for plan in (place_plan, return_plan)
                            for stage in plan.stages
                        )
                    )
                    place_solution = place_plan.stages[-1].solution
                    self.assertLessEqual(place_solution.slide_mm, 799.988)
                    self.assertGreaterEqual(place_solution.local_x_mm, 100.0)
                    self.assertLessEqual(place_solution.local_x_mm, 600.0)
                    self.assertTrue(150.0 <= place_solution.local_y_mm <= 350.0)
                    controller.execute_base_plan(place_plan)
                    controller.execute_base_plan(return_plan)

                    assert backend.axis_state is not None
                    returned = backend.solver.forward_kinematics_base(
                        backend.axis_state
                    )
                    self.assertAlmostEqual(
                        float(returned.translation_mm[0]),
                        scan_pose.x_mm,
                    )
                    self.assertAlmostEqual(
                        float(returned.translation_mm[1]),
                        scan_pose.y_mm,
                    )
                    self.assertAlmostEqual(
                        float(returned.translation_mm[2]),
                        scan_pose.z_mm,
                    )


if __name__ == "__main__":
    unittest.main()

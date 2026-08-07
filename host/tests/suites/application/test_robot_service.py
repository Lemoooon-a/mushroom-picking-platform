from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from application.controller import (
    BaseMotionExecutionError,
    MushroomRobotController,
)
from application.execution_record import JsonLinesExecutionRecorder
from application.motion_target import BaseToolTarget
from application.robot_service import (
    MushroomRobotService,
    RobotServiceCapabilityError,
    RobotServiceStateError,
)
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import HandEyeCalibration, HandEyeCalibrationStatus
from config.tray_workspace import TrayWorkspaceConfig
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from motion.unified_controller import UnifiedMotionError
from motion.unified_protocol import (
    AxisName,
    AxisState,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    RelativeAxisTarget,
    StopReport,
)
from vision.target_resolver import VisionTargetResolver


class _Backend:
    def __init__(self) -> None:
        self.calls = []
        self.plan_error = None
        self.execute_error = None
        self.execute_result = True
        self.last_stop_report = None

    def startup(self): self.calls.append("startup")
    def require_base_motion_ready(self): self.calls.append("ready")
    def plan_to_base_pose(self, x, y, z, yaw):
        self.calls.append(("plan", x, y, z, yaw))
        if self.plan_error: raise self.plan_error
        return "plan"
    def execute_base_plan(self, plan):
        self.calls.append(("execute", plan))
        if self.execute_error: raise self.execute_error
        return self.execute_result
    def return_to_startup(self): self.calls.append("return")
    def stop(self): self.calls.append("stop")
    def enable_joints(self): self.calls.append("enable")
    def disable_joints(self): self.calls.append("disable")
    def suction_grip(self): self.calls.append("grip")
    def suction_release(self): self.calls.append("release")
    def suction_idle(self): self.calls.append("idle")
    def get_status(self): self.calls.append("status"); return "ok"
    def shutdown(self): self.calls.append("shutdown")


class _PoseProvider:
    def __init__(self, base_T_tool: RigidTransform) -> None:
        self.base_T_tool = base_T_tool
        self.calls: list[RobotAxisState] = []

    def forward_kinematics_base(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform:
        self.calls.append(axis_state)
        return self.base_T_tool


def _axis_states(
    *,
    busy_axis: AxisName | None = None,
    invalid_axis: AxisName | None = None,
) -> tuple[AxisState, ...]:
    positions = {
        AxisName.SLIDE: 1.0,
        AxisName.Z: -2.0,
        AxisName.SHOULDER: 3.0,
        AxisName.ELBOW: 4.0,
        AxisName.ROTATION: 5.0,
    }
    return tuple(
        AxisState(
            axis=axis,
            connected=True,
            enabled=True,
            busy=axis is busy_axis,
            homed=True if axis in (AxisName.SLIDE, AxisName.Z) else None,
            position_valid=axis is not invalid_axis,
            current_position=None if axis is invalid_axis else positions[axis],
            position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
            faulted=False,
            fault_code=None,
            fault_message=None,
        )
        for axis in AxisName
    )


class RobotServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = _Backend()
        self.controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(TrayWorkspaceConfig(-10, 10, -10, 10, -10, 10)),
        )

    def service(self, mode, **kwargs):
        return MushroomRobotService(controller=self.controller, workflow=None, mode=mode, **kwargs)

    def camera_point_service(
        self,
        *,
        validated: bool = False,
        states: tuple[AxisState, ...] | None = None,
    ) -> tuple[MushroomRobotService, _PoseProvider, Mock, HandEyeCalibration]:
        base_T_tool = RigidTransform.from_xyz_yaw_deg(
            x_mm=100.0,
            y_mm=200.0,
            z_mm=300.0,
            yaw_deg=0.0,
        )
        tool_T_camera = RigidTransform.from_xyz_rpy_deg(
            x_mm=70.0,
            y_mm=32.5,
            z_mm=194.85,
            roll_deg=180.0,
            pitch_deg=0.0,
            yaw_deg=-90.0,
        )
        calibration = HandEyeCalibration(
            tool_T_camera=tool_T_camera,
            validated=validated,
            source="manual_measurement",
            method="measured_mount_center_plus_rgb_optical_center_offset",
        )
        provider = _PoseProvider(base_T_tool)
        resolver = VisionTargetResolver(
            pose_provider=provider,
            hand_eye_calibration=calibration,
            camera_frame_id="camera_color_optical_frame",
        )
        controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(-10, 10, -10, 10, -10, 10)
            ),
            target_resolver=resolver,
        )
        runtime_controller = Mock()
        runtime_controller.get_axis_states.return_value = (
            _axis_states() if states is None else states
        )
        service = MushroomRobotService(
            controller=controller,
            workflow=None,
            mode=RobotServiceMode.EXECUTE,
            axis_motion=runtime_controller,
            activate_controller_on_startup=False,
        )
        service.startup()
        return service, provider, runtime_controller, calibration

    def test_read_only_startup_never_activates_backend(self) -> None:
        service = self.service(RobotServiceMode.READ_ONLY)
        service.startup()
        self.assertIs(service.state, RobotServiceState.READY)
        self.assertEqual(self.backend.calls, [])
        with self.assertRaisesRegex(RobotServiceCapabilityError, "read-only"):
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))

    def test_dry_run_plans_without_submit(self) -> None:
        service = self.service(RobotServiceMode.DRY_RUN)
        service.startup()
        result = service.move_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertFalse(result.executed)
        self.assertNotIn(("execute", "plan"), self.backend.calls)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_execute_motion_failure_stops_and_faults(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.execute_error = RuntimeError("failed")
        with self.assertRaisesRegex(RuntimeError, "failed"):
            service.move_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", self.backend.calls)
        service.stop()
        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_false_execution_result_never_reports_success(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.execute_result = False

        with self.assertRaises(BaseMotionExecutionError):
            service.move_base_target(BaseToolTarget(1, 2, 3, 4))

        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", self.backend.calls)

    def test_execution_failure_with_stop_report_is_not_stopped_twice(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.execute_result = False
        self.backend.last_stop_report = StopReport(
            (
                MotionCommandResult(
                    command_id="stop-shoulder",
                    axis=AxisName.SHOULDER,
                    status=MotionCommandStatus.ABORTED,
                    accepted=True,
                    completed=False,
                    target_position=0.0,
                    final_position=0.0,
                    position_error=0.0,
                    error_code=MotionErrorCode.BACKEND_ERROR,
                    message="position hold confirmed",
                ),
            )
        )

        with self.assertRaises(BaseMotionExecutionError):
            service.move_base_target(BaseToolTarget(1, 2, 3, 4))

        self.assertNotIn("stop", self.backend.calls)
        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_planning_rejection_returns_ready_and_disable_state(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.plan_error = ValueError("no plan")
        with self.assertRaises(ValueError):
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertIs(service.state, RobotServiceState.READY)
        self.backend.plan_error = None
        service.disable_joints()
        self.assertIs(service.state, RobotServiceState.DISABLED)

    def test_missing_hand_eye_message_is_explicit(self) -> None:
        service = self.service(RobotServiceMode.DRY_RUN)
        service.startup()
        with self.assertRaisesRegex(RobotServiceCapabilityError, "Hand-eye calibration is missing or not validated"):
            service.plan_observation(object())

    def test_resolve_camera_point_uses_current_pose_and_provisional_transform(self) -> None:
        service, provider, runtime_controller, calibration = (
            self.camera_point_service(validated=False)
        )

        result = service.resolve_camera_point(20.0, -15.0, 450.0)

        self.assertEqual(result.camera_point_mm, (20.0, -15.0, 450.0))
        for actual, expected in zip(
            result.base_point_mm,
            (185.0, 212.5, 44.85),
            strict=True,
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result.frame_id, "camera_color_optical_frame")
        self.assertEqual(result.tool_camera_source, "manual_measurement")
        self.assertFalse(result.tool_camera_validated)
        self.assertIs(
            result.transform_status,
            HandEyeCalibrationStatus.PROVISIONAL,
        )
        self.assertFalse(calibration.validated)
        self.assertEqual(
            provider.calls,
            [RobotAxisState(1.0, -2.0, 3.0, 4.0, 5.0)],
        )
        runtime_controller.get_axis_states.assert_called_once_with(tuple(AxisName))
        runtime_controller.submit_positions.assert_not_called()
        runtime_controller.suction_grip.assert_not_called()
        runtime_controller.suction_release.assert_not_called()
        self.assertEqual(self.backend.calls, [])
        self.assertIs(
            service.capabilities.hand_eye_calibration,
            HandEyeCalibrationStatus.PROVISIONAL,
        )
        with self.assertRaisesRegex(
            RobotServiceCapabilityError,
            "Hand-eye calibration is missing or not validated",
        ):
            service.plan_observation(object())

    def test_resolve_camera_point_rejects_moving_or_invalid_axis_state(self) -> None:
        service, provider, _, _ = self.camera_point_service(
            states=_axis_states(busy_axis=AxisName.ELBOW)
        )
        with self.assertRaisesRegex(
            RobotServiceStateError,
            "Robot must be stationary before resolving a camera point",
        ):
            service.resolve_camera_point(1.0, 2.0, 3.0)
        self.assertEqual(provider.calls, [])

        service, provider, _, _ = self.camera_point_service(
            states=_axis_states(invalid_axis=AxisName.Z)
        )
        with self.assertRaisesRegex(
            RobotServiceStateError,
            "Current five-axis positions are unavailable or invalid",
        ):
            service.resolve_camera_point(1.0, 2.0, 3.0)
        self.assertEqual(provider.calls, [])

    def test_resolve_camera_point_validates_input_before_hardware_read(self) -> None:
        service, _, runtime_controller, _ = self.camera_point_service()
        for coordinates in (
            (float("nan"), 0.0, 1.0),
            (0.0, float("inf"), 1.0),
            (0.0, 0.0, float("-inf")),
        ):
            with self.subTest(coordinates=coordinates):
                with self.assertRaisesRegex(ValueError, "must be finite"):
                    service.resolve_camera_point(*coordinates)
        for z_mm in (0.0, -1.0):
            with self.subTest(z_mm=z_mm):
                with self.assertRaisesRegex(ValueError, "greater than zero"):
                    service.resolve_camera_point(0.0, 0.0, z_mm)
        with self.assertRaisesRegex(
            ValueError,
            "frame_id must be 'camera_color_optical_frame'",
        ):
            service.resolve_camera_point(
                0.0,
                0.0,
                1.0,
                frame_id="camera_optical",
            )
        runtime_controller.get_axis_states.assert_not_called()

    def test_jsonl_record_is_only_written_to_explicit_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "records.jsonl"
            service = self.service(
                RobotServiceMode.DRY_RUN,
                recorder=JsonLinesExecutionRecorder(path),
            )
            service.startup()
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([item["operation"] for item in records], ["startup", "plan"])
        self.assertEqual(records[-1]["application_state"], "ready")


class RobotServiceAxisMotionTests(RobotServiceTests):
    def setUp(self) -> None:
        super().setUp()
        self.axis_motion = Mock()
        self.axis_motion.get_state.return_value = _axis_states()[0]
        self.axis_motion.get_axis_states.return_value = _axis_states()
        self.axis_motion.list_axes.return_value = ()
        self.handle = MotionCommandHandle("axis-command", AxisName.SLIDE, 12.0)
        self.axis_motion.submit_absolute.return_value = self.handle
        self.axis_motion.submit_relative.return_value = self.handle
        self.axis_motion.wait.return_value = MotionCommandResult(
            command_id="axis-command",
            axis=AxisName.SLIDE,
            status=MotionCommandStatus.ARRIVED,
            accepted=True,
            completed=True,
            target_position=12.0,
            final_position=12.0,
            position_error=0.0,
            error_code=None,
            message="arrived",
        )

    def axis_service(self, mode: RobotServiceMode) -> MushroomRobotService:
        service = self.service(mode, axis_motion=self.axis_motion)
        service.startup()
        return service

    def test_query_is_available_outside_ready_and_listing_in_all_modes(self) -> None:
        for mode in RobotServiceMode:
            with self.subTest(mode=mode.value):
                service = self.axis_service(mode)
                self.assertEqual(service.list_axes(), ())
        service = self.axis_service(RobotServiceMode.EXECUTE)
        for state in (
            RobotServiceState.READY,
            RobotServiceState.EXECUTING,
            RobotServiceState.DISABLED,
            RobotServiceState.FAULT,
        ):
            with self.subTest(state=state.value):
                service.state = state
                self.assertIsInstance(
                    service.get_axis_state(AxisName.SLIDE), AxisState
                )
                self.assertEqual(len(service.get_axis_states()), 5)

    def test_absolute_motion_blocks_to_terminal_and_returns_ready(self) -> None:
        service = self.axis_service(RobotServiceMode.EXECUTE)
        result = service.move_axis_absolute(
            AxisName.SLIDE,
            12.0,
            velocity=3.0,
            acceleration=4.0,
            timeout_s=5.0,
        )
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)
        self.axis_motion.submit_absolute.assert_called_once()
        target = self.axis_motion.submit_absolute.call_args.args[0]
        self.assertEqual((target.axis, target.position), (AxisName.SLIDE, 12.0))
        self.axis_motion.wait.assert_called_once_with(self.handle, timeout_s=5.0)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_relative_motion_uses_explicit_relative_target(self) -> None:
        service = self.axis_service(RobotServiceMode.EXECUTE)
        result = service.move_axis_relative(AxisName.SLIDE, 2.0, velocity=3.0)
        target = self.axis_motion.submit_relative.call_args.args[0]
        self.assertIsInstance(target, RelativeAxisTarget)
        self.assertEqual((target.axis, target.delta), (AxisName.SLIDE, 2.0))
        self.assertEqual(result.target_position, 12.0)

    def test_motion_is_rejected_in_read_only_and_dry_run(self) -> None:
        for mode in (RobotServiceMode.READ_ONLY, RobotServiceMode.DRY_RUN):
            with self.subTest(mode=mode.value):
                service = self.axis_service(mode)
                with self.assertRaises(RobotServiceCapabilityError):
                    service.move_axis_absolute(AxisName.SLIDE, 1.0)
        self.axis_motion.submit_absolute.assert_not_called()

    def test_pre_submission_rejection_stays_ready(self) -> None:
        service = self.axis_service(RobotServiceMode.EXECUTE)
        self.axis_motion.submit_relative.side_effect = UnifiedMotionError(
            MotionErrorCode.BUSY, "busy", axis=AxisName.SLIDE
        )
        with self.assertRaises(UnifiedMotionError):
            service.move_axis_relative(AxisName.SLIDE, 1.0)
        self.assertIs(service.state, RobotServiceState.READY)
        self.axis_motion.stop.assert_not_called()

    def test_terminal_timeout_attempts_stop_and_faults(self) -> None:
        service = self.axis_service(RobotServiceMode.EXECUTE)
        self.axis_motion.wait.return_value = MotionCommandResult(
            command_id="axis-command",
            axis=AxisName.SLIDE,
            status=MotionCommandStatus.TIMEOUT,
            accepted=True,
            completed=False,
            target_position=12.0,
            final_position=10.0,
            position_error=2.0,
            error_code=MotionErrorCode.TIMEOUT,
            message="timeout",
        )
        result = service.move_axis_absolute(AxisName.SLIDE, 12.0)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.axis_motion.stop.assert_called_once_with(AxisName.SLIDE)
        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_user_stop_recovers_only_with_valid_stationary_states(self) -> None:
        service = self.axis_service(RobotServiceMode.EXECUTE)
        service.state = RobotServiceState.EXECUTING
        service.stop()
        self.assertIs(service.state, RobotServiceState.READY)

        self.axis_motion.get_axis_states.return_value = _axis_states(
            invalid_axis=AxisName.Z
        )
        service.state = RobotServiceState.EXECUTING
        service.stop()
        self.assertIs(service.state, RobotServiceState.FAULT)


if __name__ == "__main__":
    unittest.main()

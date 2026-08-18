from __future__ import annotations

import io
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from application.controller import MushroomRobotController
from application.demo_backend import DemoFlowApplicationBackend
from application.tray_workspace import TrayWorkspace
from config.project.robot_motion_envelope import (
    DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG,
    RobotMotionEnvelopeConfig,
    StartupSafePoseConfig,
)
from config.robot_runtime import RobotRuntimeConfigError
from config.tray_workspace import TrayWorkspaceConfig
from config.project.workspace_planning import ArmLocalWorkspaceStatus
from drivers.stm32_motion import STM32AxisFault
from geometry.rigid_transform import RigidTransform
from kinematics.base_frame_solver import BaseFrameFiveAxisSolver, FiveAxisNoSolutionError
from kinematics.base_move_transition_planner import BaseMoveTransitionPlanner
from kinematics.five_axis import FiveAxisGeometry, FiveAxisKinematics
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
    RotaryJointEnableStatus,
)
from motion.suction import SuctionMode, SuctionStatus
from scripts.run_motion_demo import (
    DemoFlowError,
    DemoMotionFlow,
    main,
    solve_startup_safe_pose,
)


def _model() -> FiveAxisKinematics:
    return FiveAxisKinematics(
        FiveAxisGeometry(
            link1_length_mm=400.0,
            link2_length_mm=400.0,
            tcp_height_at_z_zero_mm=180.0,
        )
    )


def _descriptors() -> dict[AxisName, AxisDescriptor]:
    limits = {
        AxisName.SLIDE: (0.0, 800.0),
        AxisName.Z: (-500.0, 0.0),
        AxisName.SHOULDER: (-65.0, 65.0),
        AxisName.ELBOW: (-160.0, 160.0),
        AxisName.ROTATION: (-150.0, 150.0),
    }
    result: dict[AxisName, AxisDescriptor] = {}
    for axis in AxisName:
        linear = axis in (AxisName.SLIDE, AxisName.Z)
        result[axis] = AxisDescriptor(
            name=axis,
            display_name=axis.value,
            kind=AxisKind.LINEAR if linear else AxisKind.ROTARY,
            position_unit="mm" if linear else "deg",
            velocity_unit="mm/s" if linear else "deg/s",
            acceleration_unit="mm/s²" if linear else "deg/s²",
            minimum_position=limits[axis][0],
            maximum_position=limits[axis][1],
            capabilities=AxisCapabilities(
                query_state=True,
                move_absolute=True,
                stop=True,
                reference_home=linear,
                configurable_velocity=axis is not AxisName.ROTATION,
                configurable_acceleration=linear,
                arrival_confirmation=True,
            ),
        )
    return result


def _solver() -> BaseFrameFiveAxisSolver:
    return BaseFrameFiveAxisSolver(
        five_axis_kinematics=_model(),
        axis_descriptors=_descriptors(),
    )


def _state_for_base_target(
    subject: BaseFrameFiveAxisSolver,
    *,
    x_mm: float = 400.0,
    y_mm: float,
    z_axis_mm: float,
    yaw_deg: float = 0.0,
) -> RobotAxisState:
    base_z = subject.five_axis_kinematics.geometry.tcp_height_at_z_zero_mm + z_axis_mm
    return subject.solve_base_target(
        base_T_tool_target=RigidTransform.from_xyz_yaw_deg(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=base_z,
            yaw_deg=yaw_deg,
        ),
        current_state=RobotAxisState(0.0, z_axis_mm, 0.0, 0.0, 0.0),
    ).axis_state()


def _axis_state(
    axis: AxisName,
    position: float | None,
    *,
    homed: bool | None,
    valid: bool = True,
    enabled: bool = True,
    busy: bool | None = False,
    faulted: bool = False,
    fault_code: int | None = None,
    fault_message: str | None = None,
) -> AxisState:
    return AxisState(
        axis=axis,
        connected=True,
        enabled=enabled,
        busy=busy,
        homed=homed,
        position_valid=valid,
        current_position=position if valid else None,
        position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
        faulted=faulted,
        fault_code=fault_code,
        fault_message=fault_message,
    )


def _terminal_axis_result(
    axis: AxisName,
    status: MotionCommandStatus,
    position: float = 0.0,
) -> MotionCommandResult:
    arrived = status is MotionCommandStatus.ARRIVED
    error_code = None
    if status is MotionCommandStatus.TIMEOUT:
        error_code = MotionErrorCode.TIMEOUT
    elif status is MotionCommandStatus.FAULT:
        error_code = MotionErrorCode.DEVICE_FAULT
    return MotionCommandResult(
        command_id=f"{axis.value}-result",
        axis=axis,
        status=status,
        accepted=True,
        completed=arrived,
        target_position=position,
        final_position=position if arrived else None,
        position_error=0.0 if arrived else None,
        error_code=error_code,
        message=status.value,
    )


class _FakeJoint:
    def __init__(self, name: str, log: list[str]) -> None:
        self.name = name
        self.log = log

    def initialize(self) -> None:
        self.log.append(f"initialize:{self.name}")


class _FakeRotationAxis:
    def __init__(self, log: list[str]) -> None:
        self.log = log
        self.config = SimpleNamespace(max_speed_raw=100)

    def command_position(self, position_rad: float, speed_raw: int) -> None:
        self.log.append("rotation:preload-current")

    def enable_torque(self) -> None:
        self.log.append("rotation:enable-torque")


class _FakeController:
    def __init__(
        self,
        log: list[str],
        *,
        z_home_status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
        slide_home_status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
    ) -> None:
        self.log = log
        self.home_status = {
            AxisName.Z: z_home_status,
            AxisName.SLIDE: slide_home_status,
        }
        self.states = {
            AxisName.SLIDE: _axis_state(AxisName.SLIDE, None, homed=False, valid=False),
            AxisName.Z: _axis_state(AxisName.Z, None, homed=False, valid=False),
            AxisName.SHOULDER: _axis_state(
                AxisName.SHOULDER, 20.0, homed=None
            ),
            AxisName.ELBOW: _axis_state(AxisName.ELBOW, -80.0, homed=None),
            AxisName.ROTATION: _axis_state(
                AxisName.ROTATION, 60.0, homed=None
            ),
        }
        self.pending: MultiAxisTarget | None = None
        self.rotary_enabled = True

    @staticmethod
    def _suction_status(mode: SuctionMode) -> SuctionStatus:
        return SuctionStatus(
            mode=mode,
            command_acknowledged=True,
            physically_verified=False,
            vacuum_detected=None,
            pump_on=mode is SuctionMode.GRIP,
            release_open=mode is SuctionMode.RELEASE,
            busy=False,
            fault=0,
            raw_state=0 if mode is SuctionMode.IDLE else 1,
        )

    def suction_idle(self) -> SuctionStatus:
        self.log.append("suction:idle")
        return self._suction_status(SuctionMode.IDLE)

    def suction_grip(self) -> SuctionStatus:
        self.log.append("suction:grip")
        return self._suction_status(SuctionMode.GRIP)

    def suction_release(self) -> SuctionStatus:
        self.log.append("suction:release")
        return self._suction_status(SuctionMode.RELEASE)

    def get_suction_status(self) -> SuctionStatus:
        self.log.append("suction:status")
        return self._suction_status(SuctionMode.IDLE)

    def get_rotary_joint_enable_status(self) -> RotaryJointEnableStatus:
        self.log.append("joints:status")
        return RotaryJointEnableStatus(
            self.rotary_enabled,
            self.rotary_enabled,
            self.rotary_enabled,
        )

    def rotary_joints_enabled(self) -> bool:
        return self.rotary_enabled

    def enable_rotary_joints(self) -> RotaryJointEnableStatus:
        self.log.append("joints:enable")
        self.rotary_enabled = True
        for axis in (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION):
            state = self.states[axis]
            self.states[axis] = _axis_state(
                axis,
                state.current_position,
                homed=None,
                enabled=True,
            )
        return RotaryJointEnableStatus(True, True, True)

    def disable_rotary_joints(self) -> RotaryJointEnableStatus:
        self.log.append("joints:disable")
        self.rotary_enabled = False
        for axis in (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION):
            state = self.states[axis]
            self.states[axis] = _axis_state(
                axis,
                state.current_position,
                homed=None,
                enabled=False,
            )
        return RotaryJointEnableStatus(False, False, False)

    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        descriptors = _descriptors()
        return tuple(descriptors[axis] for axis in AxisName)

    def get_axis_states(
        self, axes: tuple[AxisName, ...] | None = None
    ) -> tuple[AxisState, ...]:
        self.log.append("states:all")
        selected = tuple(AxisName) if axes is None else axes
        return tuple(self.states[axis] for axis in selected)

    def get_state(self, axis: AxisName) -> AxisState:
        self.log.append(f"state:{axis.value}")
        return self.states[axis]

    def home_reference(
        self, axis: AxisName, *, timeout_s: float | None = None
    ) -> MotionCommandResult:
        self.log.append(f"home:{axis.value}")
        status = self.home_status[axis]
        self.log.append(f"home-wait:{axis.value}")
        if status is MotionCommandStatus.ARRIVED:
            self.states[axis] = _axis_state(axis, 0.0, homed=True)
        return _terminal_axis_result(axis, status)

    def validate_positions(self, target: MultiAxisTarget) -> None:
        self.log.append("validate:" + ",".join(item.axis.value for item in target.targets))

    def submit_positions(self, target: MultiAxisTarget) -> MultiAxisCommandHandle:
        self.pending = target
        self.log.append("submit:" + ",".join(item.axis.value for item in target.targets))
        commands = tuple(
            MotionCommandHandle(f"command-{item.axis.value}", item.axis, item.position)
            for item in target.targets
        )
        return MultiAxisCommandHandle("group", commands)

    def wait_group(
        self, handle: MultiAxisCommandHandle, *, timeout_s: float | None = None
    ) -> MultiAxisCommandResult:
        self.log.append("wait-group")
        assert self.pending is not None
        results = []
        for target in self.pending.targets:
            previous = self.states[target.axis]
            self.states[target.axis] = _axis_state(
                target.axis,
                target.position,
                homed=(
                    True
                    if target.axis in (AxisName.SLIDE, AxisName.Z)
                    else previous.homed
                ),
            )
            results.append(
                _terminal_axis_result(
                    target.axis,
                    MotionCommandStatus.ARRIVED,
                    target.position,
                )
            )
        self.pending = None
        return MultiAxisCommandResult(
            group_id=handle.group_id,
            status=MotionCommandStatus.ARRIVED,
            results=tuple(results),
            accepted=True,
            completed=True,
            message="arrived",
        )

    def describe_axis(self, axis: AxisName) -> AxisDescriptor:
        return _descriptors()[axis]

    def stop(self, axis: AxisName) -> MotionCommandResult:
        self.log.append(f"stop:{axis.value}")
        return MotionCommandResult(
            command_id=f"stop-{axis.value}",
            axis=axis,
            status=MotionCommandStatus.ABORTED,
            accepted=True,
            completed=False,
            target_position=0.0,
            final_position=None,
            position_error=None,
            error_code=MotionErrorCode.BACKEND_ERROR,
            message="stopped",
        )


class _FakeRuntime:
    def __init__(
        self,
        *,
        z_home_status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
        slide_home_status: MotionCommandStatus = MotionCommandStatus.ARRIVED,
    ) -> None:
        self.log: list[str] = []
        self.controller = _FakeController(
            self.log,
            z_home_status=z_home_status,
            slide_home_status=slide_home_status,
        )
        self.shoulder_joint = _FakeJoint("shoulder", self.log)
        self.elbow_joint = _FakeJoint("elbow", self.log)
        self.rotation_axis = _FakeRotationAxis(self.log)
        arrival = SimpleNamespace(default_timeout_s=30.0)
        profile = SimpleNamespace(arrival=arrival)
        self.motion_config = SimpleNamespace(
            slide=profile,
            z=profile,
            shoulder=profile,
            elbow=profile,
            rotation=profile,
        )


def _flow(
    runtime: _FakeRuntime,
    *,
    execute: bool,
    motion_envelope: RobotMotionEnvelopeConfig = (
        DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG
    ),
) -> tuple[DemoMotionFlow, list[str]]:
    subject = _solver()
    output: list[str] = []
    return (
        DemoMotionFlow(
            runtime=runtime,
            solver=subject,
            planner=BaseMoveTransitionPlanner(
                subject,
                motion_envelope=motion_envelope,
            ),
            execute=execute,
            motion_envelope=motion_envelope,
            emit=output.append,
        ),
        output,
    )


def _application_controller(
    runtime: _FakeRuntime,
    flow: DemoMotionFlow,
    *,
    tray_config: TrayWorkspaceConfig | None = None,
) -> MushroomRobotController:
    return MushroomRobotController(
        base_backend=DemoFlowApplicationBackend(runtime=runtime, flow=flow),
        tray_workspace=TrayWorkspace(
            tray_config
            or TrayWorkspaceConfig(
                x_min_mm=100,
                x_max_mm=300,
                y_min_mm=-300,
                y_max_mm=300,
                z_min_mm=90,
                z_max_mm=130,
            )
        ),
    )


class StartupSafePoseSolverTests(unittest.TestCase):
    def test_startup_pose_is_inside_and_available_to_normal_solver(self) -> None:
        subject = _solver()
        solved = solve_startup_safe_pose(
            subject,
            current_state=RobotAxisState(0.0, 0.0, 20.0, -80.0, 60.0),
        )
        self.assertAlmostEqual(solved.solution.slide_mm, 0.0)
        self.assertAlmostEqual(solved.solution.z_mm, 0.0)
        self.assertAlmostEqual(solved.base_T_tool_target.translation_mm[0], 400.0)
        self.assertAlmostEqual(solved.base_T_tool_target.translation_mm[1], 150.0)
        self.assertAlmostEqual(solved.base_T_tool_target.translation_mm[2], 180.0)
        self.assertIs(
            solved.solution.workspace_status,
            ArmLocalWorkspaceStatus.INSIDE,
        )
        self.assertLess(solved.solution.position_residual_mm, 1e-6)
        normal = subject.solve_base_target(
            base_T_tool_target=solved.base_T_tool_target,
            current_state=solved.solution.axis_state(),
        )
        self.assertIs(normal.workspace_status, ArmLocalWorkspaceStatus.INSIDE)

    def test_startup_selects_the_only_joint_limit_valid_branch(self) -> None:
        subject = _solver()
        near_positive = solve_startup_safe_pose(
            subject,
            current_state=RobotAxisState(0.0, 0.0, -60.0, 120.0, -60.0),
        )
        near_negative = solve_startup_safe_pose(
            subject,
            current_state=RobotAxisState(0.0, 0.0, 60.0, -120.0, 60.0),
        )
        self.assertEqual(near_positive.solution.elbow_branch, "elbow-positive")
        self.assertEqual(near_negative.solution.elbow_branch, "elbow-positive")
        self.assertTrue(
            any(
                rejection.startswith("elbow-negative: Shoulder")
                for rejection in near_negative.branch_rejections
            )
        )

    def test_custom_startup_pose_is_injected_into_demo_flow(self) -> None:
        startup = StartupSafePoseConfig(base_x_mm=420.0)
        envelope = RobotMotionEnvelopeConfig(startup_pose=startup)
        flow, _output = _flow(
            _FakeRuntime(),
            execute=False,
            motion_envelope=envelope,
        )
        flow.startup()
        self.assertIs(flow.startup_definition, startup)
        self.assertAlmostEqual(
            flow.startup_pose.base_T_tool_target.translation_mm[0],
            420.0,
        )


class StartupExecutionTests(unittest.TestCase):
    def test_z_home_wait_precedes_slide_home_wait_and_startup_move(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        flow.startup()
        log = runtime.log
        self.assertLess(log.index("suction:idle"), log.index("joints:enable"))
        self.assertLess(log.index("joints:enable"), log.index("home:z"))
        self.assertLess(log.index("home:z"), log.index("home-wait:z"))
        self.assertLess(log.index("home-wait:z"), log.index("home:slide"))
        self.assertLess(log.index("home:slide"), log.index("home-wait:slide"))
        self.assertLess(log.index("home-wait:slide"), next(
            index for index, item in enumerate(log) if item.startswith("submit:")
        ))
        self.assertLess(
            next(index for index, item in enumerate(log) if item.startswith("submit:")),
            log.index("wait-group"),
        )
        self.assertTrue(flow.startup_fk_valid)

    def test_startup_reuses_verified_stage_state_without_second_read(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        original_get_axis_states = runtime.controller.get_axis_states
        reads_after_wait = 0

        def transient_busy_on_second_read(axes=None):
            nonlocal reads_after_wait
            states = original_get_axis_states(axes)
            if "wait-group" not in runtime.log:
                return states
            reads_after_wait += 1
            if reads_after_wait != 2:
                return states
            return tuple(
                _axis_state(
                    item.axis,
                    item.current_position,
                    homed=item.homed,
                    busy=True if item.axis is AxisName.SHOULDER else item.busy,
                )
                for item in states
            )

        runtime.controller.get_axis_states = transient_busy_on_second_read  # type: ignore[method-assign]
        flow.startup()

        self.assertEqual(reads_after_wait, 1)
        self.assertTrue(flow.startup_fk_valid)
        self.assertIsNotNone(flow.virtual_state)

    def test_startup_accepts_transient_busy_after_controller_confirmed_arrival(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        original_get_axis_states = runtime.controller.get_axis_states

        def busy_on_first_post_wait_read(axes=None):
            states = original_get_axis_states(axes)
            if "wait-group" not in runtime.log:
                return states
            return tuple(
                _axis_state(
                    item.axis,
                    item.current_position,
                    homed=item.homed,
                    busy=True if item.axis is AxisName.SHOULDER else item.busy,
                )
                for item in states
            )

        runtime.controller.get_axis_states = busy_on_first_post_wait_read  # type: ignore[method-assign]
        flow.startup()

        self.assertTrue(flow.startup_fk_valid)
        self.assertFalse(flow.motion_interrupted)

    def test_failure_reports_the_active_stage_instead_of_the_last_stage(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=True)
        flow.startup()
        original_wait_group = runtime.controller.wait_group

        def fail_transit(handle, *, timeout_s=None):
            result = original_wait_group(handle, timeout_s=timeout_s)
            return MultiAxisCommandResult(
                group_id=result.group_id,
                status=MotionCommandStatus.TIMEOUT,
                results=result.results,
                accepted=True,
                completed=False,
                message="transit timeout",
            )

        runtime.controller.wait_group = fail_transit  # type: ignore[method-assign]

        self.assertFalse(flow.move(200.0, 250.0, 120.0, 0.0))
        self.assertTrue(any(line.startswith("TRANSIT failed:") for line in output))
        self.assertFalse(any(line.startswith("LOWER failed:") for line in output))
        self.assertTrue(flow.motion_interrupted)

    def test_home_fault_or_timeout_never_runs_later_steps(self) -> None:
        cases = (
            (MotionCommandStatus.FAULT, MotionCommandStatus.ARRIVED, "z"),
            (MotionCommandStatus.TIMEOUT, MotionCommandStatus.ARRIVED, "z"),
            (MotionCommandStatus.ARRIVED, MotionCommandStatus.FAULT, "slide"),
            (MotionCommandStatus.ARRIVED, MotionCommandStatus.TIMEOUT, "slide"),
        )
        for z_status, slide_status, failed_axis in cases:
            with self.subTest(
                z_status=z_status,
                slide_status=slide_status,
                failed_axis=failed_axis,
            ):
                runtime = _FakeRuntime(
                    z_home_status=z_status,
                    slide_home_status=slide_status,
                )
                flow, _output = _flow(runtime, execute=True)
                with self.assertRaises(DemoFlowError):
                    flow.startup()
                self.assertFalse(any(item.startswith("submit:") for item in runtime.log))
                if failed_axis == "z":
                    self.assertNotIn("home:slide", runtime.log)

    def test_home_preflight_allows_homing_fault_retry_only(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        runtime.controller.states[AxisName.Z] = _axis_state(
            AxisName.Z,
            None,
            homed=False,
            valid=False,
            enabled=False,
            faulted=True,
            fault_code=int(STM32AxisFault.HOMING),
            fault_message="stm32_axis.homing",
        )

        flow._home_and_verify(AxisName.Z, 1.0)

        self.assertIn("home:z", runtime.log)
        self.assertTrue(runtime.controller.states[AxisName.Z].homed)

        runtime.log.clear()
        runtime.controller.states[AxisName.Z] = _axis_state(
            AxisName.Z,
            None,
            homed=False,
            valid=False,
            enabled=False,
            faulted=True,
            fault_code=int(STM32AxisFault.HARDWARE_OR_CONFIG),
            fault_message="stm32_axis.hardware_or_config",
        )
        with self.assertRaisesRegex(DemoFlowError, "blocking fault_code=3"):
            flow._home_and_verify(AxisName.Z, 1.0)
        self.assertNotIn("home:z", runtime.log)

    def test_rotary_enable_failure_prevents_all_homing_and_motion(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)

        def fail_enable() -> RotaryJointEnableStatus:
            runtime.log.append("joints:enable-failed")
            raise RuntimeError("elbow enable failed")

        runtime.controller.enable_rotary_joints = fail_enable  # type: ignore[method-assign]
        with self.assertRaisesRegex(RuntimeError, "elbow enable failed"):
            flow.startup()
        self.assertIn("suction:idle", runtime.log)
        self.assertNotIn("home:z", runtime.log)
        self.assertNotIn("home:slide", runtime.log)
        self.assertFalse(any(item.startswith("submit:") for item in runtime.log))

    def test_read_only_startup_never_homes_submits_stops_or_enables_torque(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        self.assertTrue(flow.return_to_startup())
        flow.stop()
        forbidden_prefixes = (
            "home:", "submit:", "stop:", "rotation:", "suction:", "joints:enable",
            "joints:disable",
        )
        self.assertFalse(
            any(item.startswith(forbidden_prefixes) for item in runtime.log),
            runtime.log,
        )
        self.assertTrue(flow.startup_fk_valid)
        self.assertFalse(flow.motion_interrupted)

    def test_execute_stop_attempts_rotation_position_hold(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)

        flow.stop()

        self.assertEqual(
            [item for item in runtime.log if item.startswith("stop:")],
            [f"stop:{axis.value}" for axis in AxisName],
        )
        self.assertFalse(flow.motion_interrupted)

    def test_successful_stop_allows_next_move_from_live_stopped_state(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        flow.startup()
        flow.stop()

        stopped_state = _state_for_base_target(
            flow.solver,
            y_mm=250.0,
            z_axis_mm=-60.0,
        )
        stopped_positions = (
            stopped_state.slide_mm,
            stopped_state.z_mm,
            stopped_state.shoulder_deg,
            stopped_state.elbow_deg,
            stopped_state.rotation_deg,
        )
        for axis, position in zip(AxisName, stopped_positions, strict=True):
            previous = runtime.controller.states[axis]
            runtime.controller.states[axis] = _axis_state(
                axis,
                position,
                homed=previous.homed,
            )
        flow.virtual_state = RobotAxisState(0.0, 0.0, 20.0, -80.0, 60.0)

        self.assertEqual(flow._planning_state(), stopped_state)
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        self.assertTrue(any(item.startswith("submit:") for item in runtime.log))

    def test_late_abort_from_stopped_move_does_not_restore_interruption_lock(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=True)
        flow.startup()
        wait_entered = threading.Event()
        release_wait = threading.Event()
        original_wait_group = runtime.controller.wait_group

        def abort_after_stop(handle, *, timeout_s=None):
            wait_entered.set()
            self.assertTrue(release_wait.wait(timeout=2.0))
            arrived = original_wait_group(handle, timeout_s=timeout_s)
            return MultiAxisCommandResult(
                group_id=arrived.group_id,
                status=MotionCommandStatus.ABORTED,
                results=arrived.results,
                accepted=True,
                completed=False,
                message="aborted by user STOP",
            )

        runtime.controller.wait_group = abort_after_stop  # type: ignore[method-assign]
        move_results: list[bool] = []
        worker = threading.Thread(
            target=lambda: move_results.append(
                flow.move(200.0, 250.0, 120.0, 0.0)
            )
        )
        worker.start()
        self.assertTrue(wait_entered.wait(timeout=1.0))

        flow.stop()
        self.assertFalse(flow.motion_interrupted)
        release_wait.set()
        worker.join(timeout=2.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(move_results, [False])
        self.assertFalse(flow.motion_interrupted)
        self.assertTrue(
            any("ended after a confirmed STOP" in line for line in output)
        )

        runtime.controller.wait_group = original_wait_group  # type: ignore[method-assign]
        runtime.log.clear()
        flow.require_base_motion_ready()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        self.assertTrue(any(item.startswith("submit:") for item in runtime.log))

    def test_unconfirmed_stop_keeps_position_commands_blocked(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        flow.startup()
        original_stop = runtime.controller.stop

        def fail_elbow_stop(axis: AxisName) -> MotionCommandResult:
            if axis is AxisName.ELBOW:
                raise RuntimeError("elbow stop failed")
            return original_stop(axis)

        runtime.controller.stop = fail_elbow_stop  # type: ignore[method-assign]

        with self.assertRaisesRegex(DemoFlowError, "elbow"):
            flow.stop()
        self.assertTrue(flow.motion_interrupted)
        with self.assertRaisesRegex(DemoFlowError, 'then "return"'):
            flow.require_base_motion_ready()

    def test_cli_suction_and_joint_commands_and_quit_lifecycle(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=True)
        flow.startup()
        runtime.log.clear()
        with patch(
            "builtins.input",
            side_effect=[
                "suction grip",
                "suction release",
                "suction idle",
                "suction status",
                "joints status",
                "joints disable",
                "joints enable",
                "stop",
                "quit",
            ],
        ):
            flow.command_loop(_application_controller(runtime, flow))
        self.assertIn("suction:grip", runtime.log)
        self.assertIn("suction:release", runtime.log)
        self.assertIn("suction:idle", runtime.log)
        self.assertIn("suction:status", runtime.log)
        self.assertIn("joints:disable", runtime.log)
        self.assertIn("joints:enable", runtime.log)
        self.assertTrue(runtime.controller.rotary_enabled)
        self.assertTrue(any("Disabling joint torque" in line for line in output))
        self.assertTrue(any("remain enabled" in line for line in output))

    def test_cli_return_alias_uses_return_to_startup_flow(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=True)
        flow.startup()
        runtime.log.clear()

        with patch("builtins.input", side_effect=["return", "quit"]):
            flow.command_loop(_application_controller(runtime, flow))

        self.assertTrue(any(line == "Return-to-startup plan:" for line in output))
        self.assertFalse(any(item.startswith("submit:") for item in runtime.log))

    def test_interrupted_move_recovery_message_uses_return_command(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        flow.startup()
        flow.motion_interrupted = True

        with self.assertRaisesRegex(DemoFlowError, 'then "return"'):
            flow.require_base_motion_ready()

    def test_cli_workspace_and_outside_move_reject_without_motion_command(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        runtime.log.clear()
        controller = _application_controller(runtime, flow)
        with (
            patch(
                "builtins.input",
                side_effect=["workspace", "move 500 250 120 0", "quit"],
            ),
            patch.object(
                controller,
                "move_to_base_pose",
                wraps=controller.move_to_base_pose,
            ) as move_to_base_pose,
        ):
            flow.command_loop(controller)
        move_to_base_pose.assert_called_once_with(500.0, 250.0, 120.0, 0.0)
        self.assertTrue(
            any(
                line == "REJECTED: target outside cultivation-tray workspace."
                for line in output
            )
        )
        self.assertTrue(
            any("Cultivation-tray workspace in Base frame:" in line for line in output)
        )
        self.assertTrue(
            any("Arm-local workspace:" in line for line in output)
        )
        for message in (
            "Tray workspace is not the robot mechanical range.",
            "Arm-local workspace is not expressed in Base coordinates.",
            "Robot motion envelope is not a collision model.",
        ):
            self.assertIn(message, output)
        self.assertTrue(
            any(
                "workspace-entry clearance Base Z: 200 mm" in line
                for line in output
            )
        )
        self.assertFalse(
            any(item.startswith(("validate:", "submit:", "stop:")) for item in runtime.log),
            runtime.log,
        )
        self.assertFalse(any(line.startswith("MOVE failed:") for line in output))

    def test_invalid_runtime_config_fails_before_runtime_creation(self) -> None:
        with (
            patch(
                "scripts.run_motion_demo.load_robot_runtime_config",
                side_effect=RobotRuntimeConfigError("invalid config"),
            ),
            patch("scripts.run_motion_demo.create_demo_flow") as create_flow,
            patch("sys.stderr", new_callable=io.StringIO),
        ):
            self.assertEqual(main([]), 1)
        create_flow.assert_not_called()

    def test_read_only_cli_never_sends_suction_or_torque_writes(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=False)
        flow.startup()
        runtime.log.clear()
        flow.suction_command("grip")
        flow.joints_command("enable")
        flow.joints_command("disable")
        self.assertEqual(runtime.log, [])


class DemoPlanningTests(unittest.TestCase):
    def test_application_gate_allows_clearance_stage_above_tray_z_max(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        controller = _application_controller(runtime, flow)
        self.assertTrue(controller.move_to_base_pose(200, 250, 120, 0))
        stage_lines = [line for line in output if "Base TCP" in line]
        self.assertTrue(
            any(
                "TRANSIT" in line and "z=180.000000" in line
                for line in stage_lines
            )
        )
        self.assertTrue(
            any(
                "LOWER" in line and "z=120.000000" in line
                for line in stage_lines
            )
        )

    def test_public_plan_to_base_pose_never_submits(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=False)
        flow.startup()
        runtime.log.clear()
        stages = flow.plan_to_base_pose(200.0, 250.0, 120.0, 0.0)
        self.assertEqual(tuple(stage.name for stage in stages), ("TRANSIT", "LOWER"))
        self.assertEqual(
            tuple(item.axis for item in stages[-1].multi_axis_target.targets),
            (AxisName.Z,),
        )
        self.assertEqual(
            sum(item.startswith("validate:") for item in runtime.log),
            2,
        )
        self.assertFalse(any(item.startswith("submit:") for item in runtime.log))

    def test_first_inside_target_is_transit_then_lower(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        special = [line for line in output if line.strip().startswith(("1.", "2."))]
        self.assertTrue(any("1. TRANSIT:" in line for line in special))
        self.assertTrue(any("2. LOWER:" in line for line in special))
        self.assertFalse(any("LIFT:" in line for line in special))

    def test_first_negative_target_is_rejected(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=False)
        flow.startup()
        with self.assertRaises(FiveAxisNoSolutionError):
            flow.move(200.0, -250.0, 120.0, 0.0)

    def test_first_target_at_z_home_uses_transit_only(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 180.0, 0.0))
        special_lines = [line for line in output if "Base TCP" in line]
        self.assertTrue(any("1. TRANSIT:" in line for line in special_lines))
        self.assertFalse(any("LOWER:" in line for line in special_lines))

    def test_regular_inside_moves_are_direct(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        output.clear()
        self.assertTrue(flow.move(250.0, 250.0, 110.0, 0.0))
        self.assertTrue(any("stages=DIRECT" in line for line in output))
        output.clear()
        self.assertTrue(flow.move(200.0, 300.0, 100.0, 0.0))
        self.assertTrue(any("stages=DIRECT" in line for line in output))

    def test_return_to_startup_is_lift_to_home_then_transit(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        output.clear()
        self.assertTrue(flow.return_to_startup())
        stage_lines = [line for line in output if line.strip().startswith(("1.", "2."))]
        self.assertTrue(any("1. LIFT_TO_HOME:" in line for line in stage_lines))
        self.assertTrue(any("2. TRANSIT_TO_STARTUP:" in line for line in stage_lines))
        assert flow.virtual_state is not None
        self.assertAlmostEqual(flow.virtual_state.slide_mm, 0.0)
        self.assertAlmostEqual(flow.virtual_state.z_mm, 0.0)
        pose = flow.solver.forward_kinematics_base(flow.virtual_state)
        self.assertAlmostEqual(pose.translation_mm[0], 400.0)
        self.assertAlmostEqual(pose.translation_mm[1], 150.0)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import math
from types import SimpleNamespace
import unittest

from config.workspace_planning import OffsetWorkspaceSide
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
)
from scripts.run_motion_demo import (
    DemoFlowError,
    DemoMotionFlow,
    StartupSafePose,
    solve_startup_safe_pose,
)


def _model() -> FiveAxisKinematics:
    return FiveAxisKinematics(
        FiveAxisGeometry(
            link1_length_mm=400.0,
            link2_length_mm=400.0,
            slide_direction_xyz=(0.0, 1.0, 0.0),
            z_direction_xyz=(0.0, 0.0, 1.0),
            slide_zero_T_planar_origin_at_zero=RigidTransform.identity(),
            rotation_output_T_tool=RigidTransform.from_xyz_yaw_deg(
                x_mm=0.0,
                y_mm=0.0,
                z_mm=-240.0,
                yaw_deg=0.0,
            ),
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
                stop=axis is not AxisName.ROTATION,
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
        base_T_slide_zero=RigidTransform.from_xyz_yaw_deg(
            x_mm=-200.0,
            y_mm=0.0,
            z_mm=420.0,
            yaw_deg=0.0,
        ),
        axis_descriptors=_descriptors(),
        base_transform_validated=True,
    )


def _state_for_base_target(
    subject: BaseFrameFiveAxisSolver,
    *,
    x_mm: float = 200.0,
    y_mm: float,
    z_axis_mm: float,
    yaw_deg: float = 0.0,
) -> RobotAxisState:
    base_z = -240.0 + z_axis_mm
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
) -> AxisState:
    return AxisState(
        axis=axis,
        connected=True,
        enabled=True,
        busy=False,
        homed=homed,
        position_valid=valid,
        current_position=position if valid else None,
        position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
        faulted=False,
        fault_code=None,
        fault_message=None,
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
        if axis is AxisName.ROTATION:
            return MotionCommandResult(
                command_id="rotation-stop",
                axis=axis,
                status=MotionCommandStatus.REJECTED,
                accepted=False,
                completed=False,
                target_position=0.0,
                final_position=None,
                position_error=None,
                error_code=MotionErrorCode.UNSUPPORTED_COMMAND,
                message="unsupported",
            )
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
) -> tuple[DemoMotionFlow, list[str]]:
    subject = _solver()
    output: list[str] = []
    return (
        DemoMotionFlow(
            runtime=runtime,
            solver=subject,
            planner=BaseMoveTransitionPlanner(subject),
            execute=execute,
            startup_definition=StartupSafePose(),
            emit=output.append,
        ),
        output,
    )


class StartupSafePoseSolverTests(unittest.TestCase):
    def test_fixed_center_pose_is_outside_without_relaxing_normal_targets(self) -> None:
        subject = _solver()
        solved = solve_startup_safe_pose(
            subject,
            current_state=RobotAxisState(0.0, 0.0, 20.0, -80.0, 60.0),
        )
        self.assertAlmostEqual(solved.solution.slide_mm, 0.0)
        self.assertAlmostEqual(solved.solution.z_mm, 0.0)
        self.assertAlmostEqual(solved.base_T_tool_target.translation_mm[0], 200.0)
        self.assertAlmostEqual(solved.base_T_tool_target.translation_mm[1], 0.0)
        self.assertIs(solved.solution.workspace_side, OffsetWorkspaceSide.OUTSIDE)
        self.assertLess(solved.solution.position_residual_mm, 1e-6)

        normal = subject.solve_base_target(
            base_T_tool_target=solved.base_T_tool_target,
            current_state=solved.solution.axis_state(),
        )
        self.assertIsNot(normal.workspace_side, OffsetWorkspaceSide.OUTSIDE)
        self.assertNotAlmostEqual(normal.slide_mm, 0.0)

    def test_selected_branch_minimizes_current_shoulder_elbow_change(self) -> None:
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
        self.assertEqual(near_negative.solution.elbow_branch, "elbow-negative")


class StartupExecutionTests(unittest.TestCase):
    def test_z_home_wait_precedes_slide_home_wait_and_startup_move(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=True)
        flow.startup()
        log = runtime.log
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

    def test_read_only_startup_never_homes_submits_stops_or_enables_torque(self) -> None:
        runtime = _FakeRuntime()
        flow, _output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        self.assertTrue(flow.return_to_startup())
        flow.stop()
        forbidden_prefixes = ("home:", "submit:", "stop:", "rotation:")
        self.assertFalse(
            any(item.startswith(forbidden_prefixes) for item in runtime.log),
            runtime.log,
        )
        self.assertTrue(flow.startup_fk_valid)


class DemoPlanningTests(unittest.TestCase):
    def test_first_positive_and_negative_targets_are_transit_then_lower(self) -> None:
        for target_y in (250.0, -250.0):
            with self.subTest(target_y=target_y):
                runtime = _FakeRuntime()
                flow, output = _flow(runtime, execute=False)
                flow.startup()
                self.assertTrue(flow.move(200.0, target_y, 120.0, 0.0))
                special = [line for line in output if line.strip().startswith(("1.", "2."))]
                self.assertTrue(any("1. TRANSIT:" in line for line in special))
                self.assertTrue(any("2. LOWER:" in line for line in special))
                self.assertFalse(any("LIFT:" in line for line in special))

    def test_first_target_at_z_home_uses_transit_only(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 180.0, 0.0))
        special_lines = [line for line in output if "Base TCP" in line]
        self.assertTrue(any("1. TRANSIT:" in line for line in special_lines))
        self.assertFalse(any("LOWER:" in line for line in special_lines))

    def test_regular_same_side_and_cross_side_keep_existing_planner_semantics(self) -> None:
        runtime = _FakeRuntime()
        flow, output = _flow(runtime, execute=False)
        flow.startup()
        self.assertTrue(flow.move(200.0, 250.0, 120.0, 0.0))
        output.clear()
        self.assertTrue(flow.move(250.0, 250.0, 110.0, 0.0))
        self.assertTrue(any("stages=DIRECT" in line for line in output))
        output.clear()
        self.assertTrue(flow.move(200.0, -250.0, 100.0, 0.0))
        self.assertTrue(any("stages=LIFT,TRANSIT,LOWER" in line for line in output))
        output.clear()
        self.assertTrue(flow.move(250.0, -250.0, 110.0, 0.0))
        self.assertTrue(any("stages=DIRECT" in line for line in output))
        output.clear()
        self.assertTrue(flow.move(200.0, 250.0, 100.0, 0.0))
        self.assertTrue(any("stages=LIFT,TRANSIT,LOWER" in line for line in output))

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
        self.assertAlmostEqual(pose.translation_mm[0], 200.0)
        self.assertAlmostEqual(pose.translation_mm[1], 0.0)


if __name__ == "__main__":
    unittest.main()

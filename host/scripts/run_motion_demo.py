#!/usr/bin/env python3
"""真实五轴启动、中心安全姿态和 Base TCP 目标运动验证入口。

默认只打开硬件并执行规划预览。只有显式传入 ``--execute`` 时，才允许
Homing、扭矩准备、位置提交或软件停止命令通过现有统一运动授权门禁。
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import shlex
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from config.frame_transforms import (  # noqa: E402
    FrameTransformsDocument,
    load_frame_transforms_document,
)
from application.controller import MushroomRobotController  # noqa: E402
from application.demo_backend import DemoFlowApplicationBackend  # noqa: E402
from application.tray_workspace import (  # noqa: E402
    TargetOutsideTrayWorkspace,
    TrayWorkspace,
)
from config.tray_workspace import (  # noqa: E402
    TrayWorkspaceConfigError,
    load_tray_workspace_config,
)
from config.workspace_planning import (  # noqa: E402
    OffsetWorkspaceSide,
    SlideSelectionReason,
)
from drivers.stm32_motion import STM32AxisFault  # noqa: E402
from geometry.rigid_transform import RigidTransform, angular_difference_deg  # noqa: E402
from kinematics.base_frame_solver import (  # noqa: E402
    BaseFrameFiveAxisSolver,
    FiveAxisNoSolutionError,
    FiveAxisSolution,
    UnvalidatedBaseTransformError,
)
from kinematics.base_move_transition_planner import (  # noqa: E402
    BaseMovePlan,
    BaseMovePlanningError,
    BaseMoveTransitionPlanner,
)
from kinematics.five_axis import (  # noqa: E402
    FiveAxisKinematics,
    load_local_five_axis_kinematics,
    rotation_deg_for_output_yaw,
)
from kinematics.frame_chain import RobotAxisState  # noqa: E402
from kinematics.planar_2r import UnreachableTargetError  # noqa: E402
from motion.authorization import RuntimeMode  # noqa: E402
from motion.unified_controller import MultiAxisSubmissionError, UnifiedMotionError  # noqa: E402
from motion.unified_protocol import (  # noqa: E402
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandStatus,
    MultiAxisTarget,
)
from scripts._motion_cli_common import (  # noqa: E402
    best_effort_stop_axes_once,
    create_configured_runtime,
    format_axis_state,
    format_command_result,
    format_group_result,
    initialize_read_only_rotary_positions,
)


INITIAL_TCP_X_MM = 200.0
INITIAL_TCP_Y_MM = 0.0
INITIAL_Z_AXIS_MM = 0.0
INITIAL_SLIDE_AXIS_MM = 0.0
INITIAL_TOOL_YAW_DEG = 0.0

HOME_POSITION_TOLERANCE_MM = 0.5
STARTUP_MATCH_LINEAR_TOLERANCE_MM = 0.5
STARTUP_MATCH_ROTARY_TOLERANCE_DEG = 1.0
STARTUP_MATCH_TCP_TOLERANCE_MM = 10.0
STARTUP_MATCH_YAW_TOLERANCE_DEG = 2.0
STARTUP_FK_POSITION_TOLERANCE_MM = 10.0
STARTUP_FK_YAW_TOLERANCE_DEG = 2.0

_AXIS_ORDER = tuple(AxisName)
_ROTARY_AXES = (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION)
_STOPPABLE_OR_REPORTED_AXES = _AXIS_ORDER
_DEFAULT_FRAME_CONFIG = HOST_ROOT / "config" / "frame_transforms.local.json"
_DEFAULT_TRAY_WORKSPACE_CONFIG = (
    HOST_ROOT / "config" / "tray_workspace.local.json"
)


class DemoFlowError(RuntimeError):
    """启动或阶段执行未通过必要的安全检查。"""


class StartupPoseNoSolutionError(DemoFlowError):
    """固定中心安全姿态的两支平面逆解都被拒绝。"""

    def __init__(self, message: str, rejections: Sequence[str]) -> None:
        super().__init__(message)
        self.rejections = tuple(rejections)


@dataclass(frozen=True)
class StartupSafePose:
    """仅允许启动、显式 ``init`` 和可选退出收回使用的中心姿态。"""

    tcp_base_x_mm: float = INITIAL_TCP_X_MM
    tcp_base_y_mm: float = INITIAL_TCP_Y_MM
    z_axis_mm: float = INITIAL_Z_AXIS_MM
    slide_axis_mm: float = INITIAL_SLIDE_AXIS_MM
    tool_yaw_deg: float = INITIAL_TOOL_YAW_DEG


@dataclass(frozen=True)
class SolvedStartupPose:
    definition: StartupSafePose
    base_T_tool_target: RigidTransform
    solution: FiveAxisSolution
    branch_rejections: tuple[str, ...]


@dataclass(frozen=True)
class DemoStage:
    name: str
    base_T_tool_target: RigidTransform
    multi_axis_target: MultiAxisTarget
    solution: FiveAxisSolution | None = None


def solve_startup_safe_pose(
    solver: BaseFrameFiveAxisSolver,
    *,
    current_state: RobotAxisState,
    definition: StartupSafePose = StartupSafePose(),
) -> SolvedStartupPose:
    """固定 Slide/Z 求解中心安全姿态，不调用普通偏置工作区搜索。"""

    if not isinstance(solver, BaseFrameFiveAxisSolver):
        raise TypeError("solver must be BaseFrameFiveAxisSolver")
    if not isinstance(current_state, RobotAxisState):
        raise TypeError("current_state must be RobotAxisState")
    if not isinstance(definition, StartupSafePose):
        raise TypeError("definition must be StartupSafePose")

    target = _startup_target_with_fk_derived_base_z(solver, definition)
    slide_zero_target = solver.transform_base_target_to_slide_zero(target)
    local = solver.five_axis_kinematics.compute_arm_local_target(
        slide_zero_target,
        definition.slide_axis_mm,
    )
    if not math.isclose(
        local.z_axis_mm,
        definition.z_axis_mm,
        rel_tol=0.0,
        abs_tol=solver.config.position_equality_tolerance_mm,
    ):
        raise StartupPoseNoSolutionError(
            "startup target could not be constructed at the configured Z logical position",
            (
                f"computed Z={local.z_axis_mm:.9f} mm, "
                f"required Z={definition.z_axis_mm:.9f} mm",
            ),
        )

    try:
        planar_solutions = solver.five_axis_kinematics.planar_2r.inverse(
            local.local_x_mm,
            local.local_y_mm,
        )
    except UnreachableTargetError as exc:
        raise StartupPoseNoSolutionError(
            "startup planar target is unreachable",
            (f"both planar branches unavailable: {exc}",),
        ) from exc

    output_yaw_deg = _rotation_output_yaw_deg(solver, slide_zero_target)
    candidates: list[FiveAxisSolution] = []
    rejections: list[str] = []
    for planar in planar_solutions:
        shoulder_deg = math.degrees(planar.shoulder_rad)
        elbow_deg = math.degrees(planar.elbow_rad)
        branch = _elbow_branch(elbow_deg)
        shoulder_limits = _axis_limits(solver, AxisName.SHOULDER)
        elbow_limits = _axis_limits(solver, AxisName.ELBOW)
        if not _within(shoulder_deg, shoulder_limits):
            rejections.append(
                f"{branch}: Shoulder {shoulder_deg:.6f} deg outside {shoulder_limits}"
            )
            continue
        if not _within(elbow_deg, elbow_limits):
            rejections.append(
                f"{branch}: Elbow {elbow_deg:.6f} deg outside {elbow_limits}"
            )
            continue
        rotation_candidates = _periodic_rotation_candidates(
            rotation_deg_for_output_yaw(
                output_yaw_deg,
                shoulder_deg,
                elbow_deg,
            ),
            _axis_limits(solver, AxisName.ROTATION),
            current_state.rotation_deg,
        )
        if not rotation_candidates:
            rejections.append(
                f"{branch}: Rotation has no equivalent value inside "
                f"{_axis_limits(solver, AxisName.ROTATION)}"
            )
            continue
        branch_accepted = False
        for rotation_deg in rotation_candidates:
            axis_state = RobotAxisState(
                definition.slide_axis_mm,
                definition.z_axis_mm,
                shoulder_deg,
                elbow_deg,
                rotation_deg,
            )
            try:
                candidate = solver.constrained_solution(
                    base_T_tool_target=target,
                    axis_state=axis_state,
                    slide_selection_reason=SlideSelectionReason.FIXED_SLIDE,
                    elbow_branch=branch,
                    allow_outside_workspace=True,
                )
            except FiveAxisNoSolutionError as exc:
                rejections.append(f"{branch}: {exc.stage}: {exc}")
                continue
            candidates.append(candidate)
            branch_accepted = True
        if not branch_accepted and not any(item.startswith(branch) for item in rejections):
            rejections.append(f"{branch}: no candidate passed full FK validation")

    if not candidates:
        raise StartupPoseNoSolutionError(
            "no STARTUP_SAFE_POSE inverse-kinematics branch passed limits and FK",
            rejections,
        )
    candidates.sort(
        key=lambda item: (
            abs(item.shoulder_deg - current_state.shoulder_deg)
            + abs(item.elbow_deg - current_state.elbow_deg),
            abs(item.rotation_deg - current_state.rotation_deg),
            item.elbow_branch,
        )
    )
    return SolvedStartupPose(
        definition=definition,
        base_T_tool_target=target,
        solution=candidates[0],
        branch_rejections=tuple(rejections),
    )


class DemoMotionFlow:
    """单 Runtime、顺序阻塞执行的最小五轴验证流程。"""

    def __init__(
        self,
        *,
        runtime: object,
        solver: BaseFrameFiveAxisSolver,
        planner: BaseMoveTransitionPlanner,
        execute: bool,
        startup_definition: StartupSafePose = StartupSafePose(),
        emit: Callable[[str], None] = print,
    ) -> None:
        self.runtime = runtime
        self.solver = solver
        self.planner = planner
        self.execute = bool(execute)
        self.startup_definition = startup_definition
        self.emit = emit
        self.startup_pose: SolvedStartupPose | None = None
        self.virtual_state: RobotAxisState | None = None
        self.motion_interrupted = False
        self.startup_fk_valid = False

    def startup(self) -> None:
        """读取状态；执行模式下严格 Z→Slide→中心姿态。"""

        initialize_read_only_rotary_positions(self.runtime, _ROTARY_AXES)
        initial_states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
        self.emit("Initial five-axis state:")
        self._emit_states(initial_states)

        if self.execute:
            suction = self.runtime.controller.suction_idle()
            self.emit("Startup suction safe state:")
            self._emit_suction_status(suction)
            enabled = self.runtime.controller.enable_rotary_joints()
            self.emit(
                "Rotary joints enabled: "
                f"shoulder={enabled.shoulder} elbow={enabled.elbow} "
                f"rotation={enabled.rotation}"
            )
            enabled_states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
            self.emit("Five-axis state after rotary-joint enable:")
            self._emit_states(enabled_states)
            rotary_seed = _rotary_seed_state(enabled_states)
            self._home_and_verify(AxisName.Z, self._home_timeout(AxisName.Z))
            self._home_and_verify(AxisName.SLIDE, self._home_timeout(AxisName.SLIDE))
            homed_states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
            current_state = _robot_axis_state(homed_states)
        else:
            rotary_seed = _rotary_seed_state(initial_states)
            current_state = RobotAxisState(
                self.startup_definition.slide_axis_mm,
                self.startup_definition.z_axis_mm,
                rotary_seed.shoulder_deg,
                rotary_seed.elbow_deg,
                rotary_seed.rotation_deg,
            )

        self.startup_pose = solve_startup_safe_pose(
            self.solver,
            current_state=current_state,
            definition=self.startup_definition,
        )
        self._emit_startup_solution(self.startup_pose)
        startup_target = self.solver.solution_to_multi_axis_target(
            self.startup_pose.solution
        )
        self.runtime.controller.validate_positions(startup_target)

        if not self.execute:
            self.virtual_state = self.startup_pose.solution.axis_state()
            self.startup_fk_valid = True
            self.emit(
                "READ_ONLY startup preview complete; no home, torque, submit, "
                "stop, or movement command was sent."
            )
            return

        self._execute_stage(
            DemoStage(
                "STARTUP_SAFE_POSE",
                self.startup_pose.base_T_tool_target,
                startup_target,
                self.startup_pose.solution,
            ),
            timeout_s=self._stage_timeout(),
        )
        actual_state = _robot_axis_state(
            self.runtime.controller.get_axis_states(_AXIS_ORDER)
        )
        self.virtual_state = actual_state
        self.startup_fk_valid = self._verify_startup_fk(actual_state)
        if not self.startup_fk_valid:
            self.motion_interrupted = True
            self.emit(
                "Startup FK residual is outside tolerance; only status/stop/quit "
                "are allowed."
            )

    def status(self) -> None:
        states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
        self.emit("Five-axis state:")
        self._emit_states(states)
        try:
            state = _robot_axis_state(states)
        except DemoFlowError as exc:
            self.emit(f"Current FK unavailable: {exc}")
        else:
            self._emit_fk_status(state, label="Current FK Base TCP pose")
        try:
            enabled = self.runtime.controller.get_rotary_joint_enable_status()
            self.emit(
                "Rotary joint enable state: "
                f"shoulder={enabled.shoulder} elbow={enabled.elbow} "
                f"rotation={enabled.rotation}"
            )
        except Exception as exc:
            self.emit(f"Rotary joint enable state unavailable: {exc}")
        try:
            self._emit_suction_status(self.runtime.controller.get_suction_status())
        except Exception as exc:
            self.emit(f"Suction state unavailable: {exc}")
        if not self.execute and self.virtual_state is not None:
            self._emit_fk_status(
                self.virtual_state,
                label="READ_ONLY virtual planned FK Base TCP pose",
            )

    def move(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float | None) -> bool:
        self.require_base_motion_ready()
        stages = self.plan_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)
        return self.execute_plan(stages)

    def require_base_motion_ready(self) -> None:
        if not self.startup_fk_valid:
            raise DemoFlowError("startup FK validation is not valid")
        if self.execute and not self.runtime.controller.rotary_joints_enabled():
            raise DemoFlowError(
                'Rotary joints are disabled. Run "joints enable" before motion.'
            )
        if self.motion_interrupted:
            raise DemoFlowError(
                'MOVE REJECTED: run "status" and then "init" or restart first'
            )

    def execute_plan(self, plan: object) -> bool:
        """执行已由规划入口完整验证的阶段，不重新解算目标。"""

        if not isinstance(plan, tuple) or not plan or not all(
            isinstance(stage, DemoStage) for stage in plan
        ):
            raise TypeError("plan must be a non-empty tuple of DemoStage objects")
        return self._run_stages(plan)

    def plan_to_base_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float | None,
    ) -> tuple[DemoStage, ...]:
        """只生成现有 Base-frame 阶段，不提交任何轴命令。"""

        current_state = self._planning_state()
        current_pose = self.solver.forward_kinematics_base(current_state)
        target_yaw = current_pose.yaw_deg if yaw_deg is None else _finite("yaw_deg", yaw_deg)
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=_finite("x_mm", x_mm),
            y_mm=_finite("y_mm", y_mm),
            z_mm=_finite("z_mm", z_mm),
            yaw_deg=target_yaw,
        )

        if self._strictly_matches_startup(current_state):
            stages = self._plan_first_target_from_startup(current_state, target)
            self.emit("Special first-target plan from verified STARTUP_SAFE_POSE:")
            self._emit_demo_stages(stages)
        else:
            plan = self.planner.plan(
                current_state=current_state,
                base_T_tool_target=target,
            )
            self._emit_regular_plan(plan)
            stages = tuple(
                DemoStage(
                    stage.kind.name,
                    stage.base_T_tool_target,
                    stage.multi_axis_target,
                    stage.solution,
                )
                for stage in plan.stages
            )
        for stage in stages:
            self.runtime.controller.validate_positions(stage.multi_axis_target)
        return stages

    def return_to_startup(self) -> bool:
        if self.execute and not self.runtime.controller.rotary_joints_enabled():
            self.emit('Rotary joints are disabled. Run "joints enable" before motion.')
            return False
        if self.startup_pose is None:
            raise DemoFlowError("startup pose has not been solved")
        current_state = self._planning_state()
        lift_state = RobotAxisState(
            current_state.slide_mm,
            self.startup_definition.z_axis_mm,
            current_state.shoulder_deg,
            current_state.elbow_deg,
            current_state.rotation_deg,
        )
        lift_pose = self.solver.forward_kinematics_base(lift_state)
        lift_target = MultiAxisTarget(
            (AxisTarget(AxisName.Z, self.startup_definition.z_axis_mm),)
        )
        startup_target = self.solver.solution_to_multi_axis_target(
            self.startup_pose.solution
        )
        stages = (
            DemoStage("LIFT_TO_HOME", lift_pose, lift_target),
            DemoStage(
                "TRANSIT_TO_STARTUP",
                self.startup_pose.base_T_tool_target,
                startup_target,
                self.startup_pose.solution,
            ),
        )
        self.emit("Return-to-startup plan:")
        self._emit_demo_stages(stages)
        succeeded = self._run_stages(stages)
        if not succeeded:
            return False
        if self.execute:
            actual = _robot_axis_state(
                self.runtime.controller.get_axis_states(_AXIS_ORDER)
            )
            self.virtual_state = actual
            self.startup_fk_valid = self._verify_startup_fk(actual)
            succeeded = self.startup_fk_valid
        else:
            self.startup_fk_valid = True
        self.motion_interrupted = not succeeded
        return succeeded

    def stop(self) -> None:
        self.motion_interrupted = True
        if not self.execute:
            self.emit("READ_ONLY stop preview; no stop command was sent")
            return
        best_effort_stop_axes_once(
            self.runtime,
            _STOPPABLE_OR_REPORTED_AXES,
            emit=self.emit,
        )
        self.emit(
            "Stop requested. No plan will resume automatically; Rotation has no "
            "verified independent software stop and is reported as such. "
            "Rotary joint holding torque remains enabled."
        )

    def suction_command(self, action: str) -> None:
        if not self.execute and action != "status":
            self.emit(f"READ_ONLY suction {action} preview; no suction command was sent")
            return
        self.emit(f"Suction command requested: {action.upper()}")
        methods = {
            "grip": self.runtime.controller.suction_grip,
            "release": self.runtime.controller.suction_release,
            "idle": self.runtime.controller.suction_idle,
            "status": self.runtime.controller.get_suction_status,
        }
        self._emit_suction_status(methods[action]())

    def joints_command(self, action: str) -> None:
        if action == "status":
            status = self.runtime.controller.get_rotary_joint_enable_status()
            self.emit(
                "Rotary joints: "
                f"shoulder={status.shoulder} elbow={status.elbow} "
                f"rotation={status.rotation} all_enabled={status.all_enabled}"
            )
            return
        if not self.execute:
            self.emit(f"READ_ONLY joints {action} preview; no torque command was sent")
            return
        if action == "disable":
            self.emit("WARNING:")
            self.emit("Disabling joint torque removes position holding.")
            self.emit("Make sure the mechanism is supported and clear of obstacles.")
            status = self.runtime.controller.disable_rotary_joints()
            self.emit(
                "Rotary joints disabled: "
                f"shoulder={status.shoulder} elbow={status.elbow} "
                f"rotation={status.rotation}"
            )
            return
        status = self.runtime.controller.enable_rotary_joints()
        self.emit(
            "Rotary joints enabled: "
            f"shoulder={status.shoulder} elbow={status.elbow} "
            f"rotation={status.rotation}"
        )
        states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
        self._emit_states(states)
        state = _robot_axis_state(states)
        self.virtual_state = state
        self._emit_fk_status(state, label="Current FK Base TCP pose after enable")

    def command_loop(self, application_controller: MushroomRobotController) -> None:
        if not isinstance(application_controller, MushroomRobotController):
            raise TypeError(
                "application_controller must be MushroomRobotController"
            )
        self.emit('Type "help" for commands.')
        while True:
            try:
                line = input("motion-demo> ").strip()
            except EOFError:
                line = "quit"
            except KeyboardInterrupt:
                self.emit("\nCtrl+C received; stopping the current plan")
                self.stop()
                continue
            if not line:
                continue
            try:
                parts = shlex.split(line)
            except ValueError as exc:
                self.emit(f"Command parse error: {exc}")
                continue
            command = parts[0].lower()
            if command in ("quit", "exit"):
                if len(parts) != 1:
                    self.emit("usage: quit")
                    continue
                self.emit('Use "init" before "quit" if you want to return to the startup pose.')
                self.stop()
                self.emit(
                    'Rotary joints remain enabled unless "joints disable" was '
                    "explicitly executed. Support the mechanism before disabling "
                    "joint torque."
                )
                return
            if command == "help":
                self._emit_help()
                continue
            if command == "status":
                self.status()
                continue
            if command == "workspace":
                self._emit_workspace(application_controller.tray_workspace)
                continue
            if command == "stop":
                self.stop()
                continue
            if command == "suction":
                if len(parts) != 2 or parts[1].lower() not in (
                    "grip", "release", "idle", "status"
                ):
                    self.emit("usage: suction <grip|release|idle|status>")
                    continue
                try:
                    self.suction_command(parts[1].lower())
                except Exception as exc:
                    self.emit(f"SUCTION failed: {exc}")
                continue
            if command == "joints":
                if len(parts) != 2 or parts[1].lower() not in (
                    "enable", "disable", "status"
                ):
                    self.emit("usage: joints <enable|disable|status>")
                    continue
                try:
                    self.joints_command(parts[1].lower())
                except Exception as exc:
                    self.emit(f"JOINTS failed: {exc}")
                continue
            if not self.startup_fk_valid:
                self.emit("Command rejected: startup FK validation is not valid")
                continue
            if command in ("init", "return-init"):
                try:
                    self.return_to_startup()
                except Exception as exc:
                    self._handle_plan_failure("RETURN_TO_STARTUP", exc)
                continue
            if command == "move":
                if len(parts) not in (4, 5):
                    self.emit("usage: move <x_mm> <y_mm> <z_mm> [yaw_deg]")
                    continue
                try:
                    values = tuple(float(value) for value in parts[1:])
                    application_controller.move_to_base_pose(
                        values[0],
                        values[1],
                        values[2],
                        values[3] if len(values) == 4 else None,
                    )
                except TargetOutsideTrayWorkspace as exc:
                    self.emit(
                        "REJECTED: target outside cultivation-tray workspace."
                    )
                    self.emit(str(exc))
                except Exception as exc:
                    self._handle_plan_failure("MOVE", exc)
                continue
            self.emit(f"Unknown command: {command!r}; type help")

    def _home_and_verify(self, axis: AxisName, timeout_s: float) -> None:
        before = self.runtime.controller.get_state(axis)
        self.emit(f"HOME preflight state: {format_axis_state(before)}")
        blockers: list[str] = []
        if not before.connected:
            blockers.append("axis is not connected")
        if before.busy is not False:
            blockers.append("moving/busy is not confirmed false")
        if before.faulted and before.fault_code != int(
            STM32AxisFault.POSITION_INVALID
        ):
            blockers.append(f"blocking fault_code={before.fault_code!r}")
        if blockers:
            raise DemoFlowError(
                f"{axis.value} Home preflight rejected: " + "; ".join(blockers)
            )
        self.emit(f"HOME stage start: {axis.value}")
        result = self.runtime.controller.home_reference(axis, timeout_s=timeout_s)
        self.emit(f"HOME result: {format_command_result(result)}")
        state = self.runtime.controller.get_state(axis)
        self.emit(f"HOME verified state: {format_axis_state(state)}")
        if result.status is not MotionCommandStatus.ARRIVED:
            raise DemoFlowError(
                f"{axis.value} Home failed with terminal status {result.status.value}"
            )
        if state.homed is not True:
            raise DemoFlowError(f"{axis.value} Home completed but homed is not true")
        if not state.position_valid or state.current_position is None:
            raise DemoFlowError(f"{axis.value} Home completed with invalid position")
        if state.faulted:
            raise DemoFlowError(f"{axis.value} Home completed with fault {state.fault_code!r}")
        if state.busy is not False:
            raise DemoFlowError(f"{axis.value} Home completed but moving/busy is not false")
        if not math.isclose(
            state.current_position,
            0.0,
            rel_tol=0.0,
            abs_tol=HOME_POSITION_TOLERANCE_MM,
        ):
            raise DemoFlowError(
                f"{axis.value} Home position {state.current_position:.6f} mm is not near 0"
            )

    def _execute_stage(self, stage: DemoStage, *, timeout_s: float) -> RobotAxisState:
        self.runtime.controller.validate_positions(stage.multi_axis_target)
        self.emit(f"Executing stage: {stage.name}")
        handle = self.runtime.controller.submit_positions(stage.multi_axis_target)
        result = self.runtime.controller.wait_group(handle, timeout_s=timeout_s)
        for line in format_group_result(result):
            self.emit(line)
        if result.status is not MotionCommandStatus.ARRIVED:
            raise DemoFlowError(
                f"stage {stage.name} failed with terminal status {result.status.value}"
            )
        states = self.runtime.controller.get_axis_states(_AXIS_ORDER)
        state = _robot_axis_state(states)
        self.emit(f"Stage {stage.name} verified after arrival:")
        self._emit_states(states)
        return state

    def _run_stages(self, stages: Sequence[DemoStage]) -> bool:
        try:
            for stage in stages:
                self.runtime.controller.validate_positions(stage.multi_axis_target)
                if self.execute:
                    self.virtual_state = self._execute_stage(
                        stage,
                        timeout_s=self._stage_timeout(),
                    )
                elif stage.solution is not None:
                    self.virtual_state = stage.solution.axis_state()
                else:
                    assert self.virtual_state is not None
                    positions = _target_positions(stage.multi_axis_target)
                    self.virtual_state = _state_with_overrides(self.virtual_state, positions)
            if not self.execute:
                self.emit("READ_ONLY stage preview complete; no submit command was sent")
            self.motion_interrupted = False
            return True
        except Exception as exc:
            self._handle_plan_failure(stages[-1].name if stages else "EMPTY_PLAN", exc)
            return False

    def _handle_plan_failure(self, stage: str, exc: BaseException) -> None:
        self.emit(f"{stage} failed: {exc}")
        self.motion_interrupted = True
        if self.execute:
            best_effort_stop_axes_once(
                self.runtime,
                _STOPPABLE_OR_REPORTED_AXES,
                emit=self.emit,
            )

    def _planning_state(self) -> RobotAxisState:
        if not self.execute:
            if self.virtual_state is None:
                raise DemoFlowError("READ_ONLY virtual startup state is unavailable")
            return self.virtual_state
        return _robot_axis_state(
            self.runtime.controller.get_axis_states(_AXIS_ORDER)
        )

    def _plan_first_target_from_startup(
        self,
        current_state: RobotAxisState,
        target: RigidTransform,
    ) -> tuple[DemoStage, ...]:
        if not self._strictly_matches_startup(current_state):
            raise DemoFlowError("first-target exception requires exact STARTUP_SAFE_POSE")
        final = self.solver.solve_base_target(
            base_T_tool_target=target,
            current_state=current_state,
        )
        if final.workspace_side is OffsetWorkspaceSide.OUTSIDE:
            raise DemoFlowError("normal target resolved outside both offset workspaces")
        transit_state = RobotAxisState(
            final.slide_mm,
            self.startup_definition.z_axis_mm,
            final.shoulder_deg,
            final.elbow_deg,
            final.rotation_deg,
        )
        transit_pose = self.solver.forward_kinematics_base(transit_state)
        transit = self.solver.constrained_solution(
            base_T_tool_target=transit_pose,
            axis_state=transit_state,
            slide_selection_reason=final.slide_selection_reason,
            elbow_branch=final.elbow_branch,
        )
        stages = [
            DemoStage(
                "TRANSIT",
                transit_pose,
                self.solver.solution_to_multi_axis_target(transit),
                transit,
            )
        ]
        if not math.isclose(
            final.z_mm,
            self.startup_definition.z_axis_mm,
            rel_tol=0.0,
            abs_tol=self.solver.config.position_equality_tolerance_mm,
        ):
            stages.append(
                DemoStage(
                    "LOWER",
                    target,
                    self.solver.solution_to_multi_axis_target(final),
                    final,
                )
            )
        return tuple(stages)

    def _strictly_matches_startup(self, state: RobotAxisState) -> bool:
        if self.startup_pose is None:
            return False
        expected = self.startup_pose.solution.axis_state()
        if not all(
            math.isclose(actual, wanted, rel_tol=0.0, abs_tol=tolerance)
            for actual, wanted, tolerance in (
                (state.slide_mm, expected.slide_mm, STARTUP_MATCH_LINEAR_TOLERANCE_MM),
                (state.z_mm, expected.z_mm, STARTUP_MATCH_LINEAR_TOLERANCE_MM),
                (
                    state.shoulder_deg,
                    expected.shoulder_deg,
                    STARTUP_MATCH_ROTARY_TOLERANCE_DEG,
                ),
                (state.elbow_deg, expected.elbow_deg, STARTUP_MATCH_ROTARY_TOLERANCE_DEG),
                (
                    state.rotation_deg,
                    expected.rotation_deg,
                    STARTUP_MATCH_ROTARY_TOLERANCE_DEG,
                ),
            )
        ):
            return False
        actual_pose = self.solver.forward_kinematics_base(state)
        expected_pose = self.startup_pose.base_T_tool_target
        return (
            math.dist(actual_pose.translation_mm[:2], expected_pose.translation_mm[:2])
            <= STARTUP_MATCH_TCP_TOLERANCE_MM
            and abs(angular_difference_deg(actual_pose.yaw_deg, expected_pose.yaw_deg))
            <= STARTUP_MATCH_YAW_TOLERANCE_DEG
        )

    def _verify_startup_fk(self, state: RobotAxisState) -> bool:
        assert self.startup_pose is not None
        expected = self.startup_pose.base_T_tool_target
        actual = self.solver.forward_kinematics_base(state)
        position_residual = math.dist(actual.translation_mm, expected.translation_mm)
        yaw_residual = abs(angular_difference_deg(actual.yaw_deg, expected.yaw_deg))
        self.emit("Initial pose reached")
        self.emit("Expected:")
        self._emit_transform(expected)
        self.emit("Actual FK:")
        self._emit_transform(actual)
        self.emit(
            f"Initial FK residual: position={position_residual:.6f} mm "
            f"yaw={yaw_residual:.6f} deg"
        )
        valid = (
            position_residual <= STARTUP_FK_POSITION_TOLERANCE_MM
            and yaw_residual <= STARTUP_FK_YAW_TOLERANCE_DEG
        )
        self.emit(f"Initial FK validation: {'PASS' if valid else 'FAIL'}")
        return valid

    def _emit_startup_solution(self, solved: SolvedStartupPose) -> None:
        solution = solved.solution
        self.emit("STARTUP_SAFE_POSE solution:")
        self._emit_transform(solved.base_T_tool_target)
        self.emit(
            f"  slide={solution.slide_mm:.6f} mm z={solution.z_mm:.6f} mm "
            f"shoulder={solution.shoulder_deg:.6f} deg "
            f"elbow={solution.elbow_deg:.6f} deg "
            f"rotation={solution.rotation_deg:.6f} deg"
        )
        self.emit(
            f"  IK branch={solution.elbow_branch} "
            f"workspace={solution.workspace_side.name} "
            f"FK residual={solution.position_residual_mm:.9g} mm/"
            f"{solution.yaw_residual_deg:.9g} deg"
        )
        for rejection in solved.branch_rejections:
            self.emit(f"  rejected branch: {rejection}")

    def _emit_regular_plan(self, plan: BaseMovePlan) -> None:
        self.emit(
            f"BaseMoveTransitionPlanner: current={plan.current_workspace_side.name} "
            f"target={plan.target_workspace_side.name} "
            f"stages={','.join(stage.kind.name for stage in plan.stages)}"
        )
        self._emit_demo_stages(
            tuple(
                DemoStage(
                    stage.kind.name,
                    stage.base_T_tool_target,
                    stage.multi_axis_target,
                    stage.solution,
                )
                for stage in plan.stages
            )
        )

    def _emit_demo_stages(self, stages: Sequence[DemoStage]) -> None:
        for index, stage in enumerate(stages, start=1):
            xyz = stage.base_T_tool_target.translation_mm
            targets = ", ".join(
                f"{item.axis.value}={item.position:.6f}"
                for item in stage.multi_axis_target.targets
            )
            self.emit(
                f"  {index}. {stage.name}: Base TCP "
                f"x={xyz[0]:.6f} y={xyz[1]:.6f} z={xyz[2]:.6f} "
                f"yaw={stage.base_T_tool_target.yaw_deg:.6f}; {targets}"
            )

    def _emit_fk_status(self, state: RobotAxisState, *, label: str) -> None:
        pose = self.solver.forward_kinematics_base(state)
        side, local_x, local_y = self.solver.workspace_side_for_state(state)
        self.emit(f"{label}:")
        self._emit_transform(pose)
        self.emit(
            f"  local_x={local_x:.6f} mm local_y={local_y:.6f} mm "
            f"workspace_side={side.name}"
        )

    def _emit_states(self, states: Sequence[AxisState]) -> None:
        for state in states:
            self.emit(f"  {format_axis_state(state)}")

    def _emit_suction_status(self, status: object) -> None:
        self.emit(f"Suction command: {status.mode.value.upper()}")
        self.emit(
            "Command acknowledged: "
            f"{'yes' if status.command_acknowledged else 'no'}"
        )
        self.emit(
            "Physical vacuum verified: "
            f"{'yes' if status.physically_verified else 'no'}"
        )
        self.emit(
            f"Outputs: pump={'on' if status.pump_on else 'off'} "
            f"release_valve={'open' if status.release_open else 'closed'} "
            f"busy={status.busy} fault={status.fault}"
        )

    def _emit_workspace(self, tray_workspace: TrayWorkspace) -> None:
        config = tray_workspace.config
        offset = self.solver.workspace
        self.emit("Cultivation-tray workspace in Base frame:")
        self.emit(f"  X: [{config.x_min_mm:g}, {config.x_max_mm:g}] mm")
        self.emit(f"  Y: [{config.y_min_mm:g}, {config.y_max_mm:g}] mm")
        self.emit(f"  Z: [{config.z_min_mm:g}, {config.z_max_mm:g}] mm")
        self.emit("Positive offset workspace in arm-local frame:")
        self.emit(
            f"  local X: [{offset.local_x_min_mm:g}, "
            f"{offset.local_x_max_mm:g}] mm"
        )
        self.emit(
            f"  local Y: [{offset.positive_y_min_mm:g}, "
            f"{offset.positive_y_max_mm:g}] mm"
        )
        self.emit("Negative offset workspace in arm-local frame:")
        self.emit(
            f"  local X: [{offset.local_x_min_mm:g}, "
            f"{offset.local_x_max_mm:g}] mm"
        )
        self.emit(
            f"  local Y: [{offset.negative_y_min_mm:g}, "
            f"{offset.negative_y_max_mm:g}] mm"
        )
        self.emit("Startup safe pose:")
        self.emit("  explicit exception")

    def _emit_transform(self, transform: RigidTransform) -> None:
        xyz = transform.translation_mm
        self.emit(
            f"  Base X={xyz[0]:.6f} mm Y={xyz[1]:.6f} mm "
            f"Z={xyz[2]:.6f} mm yaw={transform.yaw_deg:.6f} deg"
        )

    def _home_timeout(self, axis: AxisName) -> float:
        profile = (
            self.runtime.motion_config.z
            if axis is AxisName.Z
            else self.runtime.motion_config.slide
        )
        return float(profile.arrival.default_timeout_s)

    def _stage_timeout(self) -> float:
        profiles = (
            self.runtime.motion_config.slide,
            self.runtime.motion_config.z,
            self.runtime.motion_config.shoulder,
            self.runtime.motion_config.elbow,
            self.runtime.motion_config.rotation,
        )
        return max(float(profile.arrival.default_timeout_s) for profile in profiles)

    def _emit_help(self) -> None:
        self.emit("Commands:")
        self.emit("  status")
        self.emit("  workspace")
        self.emit("  move <x_mm> <y_mm> <z_mm> [yaw_deg]")
        self.emit("  init | return-init")
        self.emit("  suction <grip|release|idle|status>")
        self.emit("  joints <enable|disable|status>")
        self.emit("  stop")
        self.emit("  quit")
        self.emit("  help")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Five-axis real startup and Base TCP motion demo"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "explicitly allow real Homing/motion/stop and accept the configured "
            "Rotation backend's lack of a verified independent stop"
        ),
    )
    parser.add_argument(
        "--frame-config",
        type=Path,
        default=_DEFAULT_FRAME_CONFIG,
        help="validated Base-to-Slide-zero transform JSON",
    )
    parser.add_argument(
        "--tray-workspace-config",
        type=Path,
        default=_DEFAULT_TRAY_WORKSPACE_CONFIG,
        help="user-validated Base-frame cultivation-tray workspace JSON",
    )
    return parser


def create_demo_flow(
    *,
    execute: bool,
    frame_config: Path = _DEFAULT_FRAME_CONFIG,
    emit: Callable[[str], None] = print,
) -> tuple[object, DemoMotionFlow]:
    mode = RuntimeMode.MOTION if execute else RuntimeMode.READ_ONLY
    runtime = create_configured_runtime(
        mode,
        allow_unverified_rotation_motion=execute,
    )
    document = load_frame_transforms_document(frame_config)
    solver = _configured_solver(runtime, document, load_local_five_axis_kinematics())
    return runtime, DemoMotionFlow(
        runtime=runtime,
        solver=solver,
        planner=BaseMoveTransitionPlanner(solver),
        execute=execute,
        emit=emit,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        tray_workspace = TrayWorkspace(
            load_tray_workspace_config(args.tray_workspace_config)
        )
        runtime, flow = create_demo_flow(
            execute=args.execute,
            frame_config=args.frame_config,
        )
        backend = DemoFlowApplicationBackend(runtime=runtime, flow=flow)
        robot = MushroomRobotController(
            base_backend=backend,
            tray_workspace=tray_workspace,
        )
        started = False
        try:
            robot.startup()
            started = True
            flow.command_loop(robot)
        except KeyboardInterrupt:
            print("motion demo interrupted; requesting software stop", file=sys.stderr)
            if started:
                robot.stop()
            return 130
        except Exception as exc:
            print(f"startup failed: {exc}", file=sys.stderr)
            if isinstance(exc, StartupPoseNoSolutionError):
                for rejection in exc.rejections:
                    print(f"  rejected: {rejection}", file=sys.stderr)
            if args.execute and started:
                robot.stop()
            return 1
        finally:
            robot.shutdown()
        return 0
    except KeyboardInterrupt:
        print("motion demo interrupted", file=sys.stderr)
        return 130
    except (
        BaseMovePlanningError,
        FiveAxisNoSolutionError,
        MultiAxisSubmissionError,
        TrayWorkspaceConfigError,
        UnifiedMotionError,
        UnvalidatedBaseTransformError,
    ) as exc:
        print(f"motion demo failed: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"motion demo failed: {exc}", file=sys.stderr)
        return 2


def _configured_solver(
    runtime: object,
    document: FrameTransformsDocument,
    model: FiveAxisKinematics,
) -> BaseFrameFiveAxisSolver:
    if document.metadata.get("validated") is not True:
        raise UnvalidatedBaseTransformError(
            "Base–Slide-zero transform must have metadata.validated=true"
        )
    descriptors = {
        descriptor.name: descriptor for descriptor in runtime.controller.list_axes()
    }
    return BaseFrameFiveAxisSolver(
        five_axis_kinematics=model,
        base_T_slide_zero=document.transforms.base_T_slide_zero,
        axis_descriptors=descriptors,
        base_transform_validated=True,
    )


def _startup_target_with_fk_derived_base_z(
    solver: BaseFrameFiveAxisSolver,
    definition: StartupSafePose,
) -> RigidTransform:
    def z_axis_for_base_z(base_z_mm: float) -> float:
        candidate = RigidTransform.from_xyz_yaw_deg(
            x_mm=definition.tcp_base_x_mm,
            y_mm=definition.tcp_base_y_mm,
            z_mm=base_z_mm,
            yaw_deg=definition.tool_yaw_deg,
        )
        slide_zero = solver.transform_base_target_to_slide_zero(candidate)
        return solver.five_axis_kinematics.compute_arm_local_target(
            slide_zero,
            definition.slide_axis_mm,
        ).z_axis_mm

    at_zero = z_axis_for_base_z(0.0)
    per_base_mm = z_axis_for_base_z(1.0) - at_zero
    if abs(per_base_mm) <= 1e-12:
        raise StartupPoseNoSolutionError(
            "Base Z cannot select the configured startup Z logical position",
            (f"Z-axis response per Base-Z mm is {per_base_mm}",),
        )
    base_z = (definition.z_axis_mm - at_zero) / per_base_mm
    return RigidTransform.from_xyz_yaw_deg(
        x_mm=definition.tcp_base_x_mm,
        y_mm=definition.tcp_base_y_mm,
        z_mm=base_z,
        yaw_deg=definition.tool_yaw_deg,
    )


def _rotation_output_yaw_deg(
    solver: BaseFrameFiveAxisSolver,
    slide_zero_target: RigidTransform,
) -> float:
    geometry = solver.five_axis_kinematics.geometry
    output_target = slide_zero_target @ geometry.rotation_output_T_tool.inverse()
    relative_rotation = (
        geometry.slide_zero_T_planar_origin_at_zero.rotation_matrix.T
        @ output_target.rotation_matrix
    )
    matrix = RigidTransform.identity().matrix.copy()
    matrix[:3, :3] = relative_rotation
    roll_deg, pitch_deg, yaw_deg = RigidTransform(matrix).rpy_deg
    if max(abs(float(roll_deg)), abs(float(pitch_deg))) > (
        solver.config.model_roll_pitch_tolerance_deg
    ):
        raise StartupPoseNoSolutionError(
            "startup target roll/pitch is incompatible with the yaw-only model",
            (f"roll={roll_deg:.9f} deg pitch={pitch_deg:.9f} deg",),
        )
    return float(yaw_deg)


def _periodic_rotation_candidates(
    raw_deg: float,
    limits: tuple[float, float],
    current_deg: float,
) -> tuple[float, ...]:
    minimum, maximum = limits
    first = math.ceil((minimum - raw_deg) / 360.0 - 1e-12)
    last = math.floor((maximum - raw_deg) / 360.0 + 1e-12)
    values = [
        raw_deg + period * 360.0
        for period in range(first, last + 1)
        if _within(raw_deg + period * 360.0, limits)
    ]
    values.sort(key=lambda value: (abs(value - current_deg), value))
    return tuple(values)


def _axis_limits(
    solver: BaseFrameFiveAxisSolver,
    axis: AxisName,
) -> tuple[float, float]:
    descriptor = solver.axis_descriptors[axis]
    return descriptor.minimum_position, descriptor.maximum_position


def _within(value: float, limits: tuple[float, float]) -> bool:
    return limits[0] - 1e-9 <= value <= limits[1] + 1e-9


def _elbow_branch(elbow_deg: float) -> str:
    return "elbow-positive" if elbow_deg > 0.0 else "elbow-negative"


def _rotary_seed_state(states: Sequence[AxisState]) -> RobotAxisState:
    by_axis = {state.axis: state for state in states}
    positions: dict[AxisName, float] = {}
    for axis in _ROTARY_AXES:
        state = by_axis.get(axis)
        if state is None:
            raise DemoFlowError(f"missing initial {axis.value} state")
        if not state.connected:
            raise DemoFlowError(f"initial {axis.value} is not connected")
        if state.faulted:
            raise DemoFlowError(
                f"initial {axis.value} fault={state.fault_code!r}: {state.fault_message}"
            )
        if state.busy is not False:
            raise DemoFlowError(
                f"initial {axis.value} moving/busy is not confirmed false"
            )
        if not state.position_valid or state.current_position is None:
            raise DemoFlowError(
                f"initial {axis.value} absolute position is invalid; initialization refused"
            )
        positions[axis] = _finite(f"{axis.value} position", state.current_position)
    return RobotAxisState(
        0.0,
        0.0,
        positions[AxisName.SHOULDER],
        positions[AxisName.ELBOW],
        positions[AxisName.ROTATION],
    )


def _robot_axis_state(states: Sequence[AxisState]) -> RobotAxisState:
    by_axis = {state.axis: state for state in states}
    if set(by_axis) != set(_AXIS_ORDER):
        raise DemoFlowError("five-axis state query did not return exactly all axes")
    positions: dict[AxisName, float] = {}
    for axis in _AXIS_ORDER:
        state = by_axis[axis]
        if not state.connected:
            raise DemoFlowError(f"axis {axis.value} is not connected")
        if state.faulted:
            raise DemoFlowError(
                f"axis {axis.value} fault={state.fault_code!r}: {state.fault_message}"
            )
        if not state.position_valid or state.current_position is None:
            raise DemoFlowError(f"axis {axis.value} position is invalid")
        if state.busy is not False:
            raise DemoFlowError(f"axis {axis.value} moving/busy is not confirmed false")
        if axis in (AxisName.SLIDE, AxisName.Z) and state.homed is not True:
            raise DemoFlowError(f"axis {axis.value} is not homed")
        if axis in _ROTARY_AXES and state.enabled is not True:
            raise DemoFlowError(
                'Rotary joints are disabled. Run "joints enable" before motion.'
            )
        positions[axis] = _finite(f"{axis.value} position", state.current_position)
    return RobotAxisState(
        positions[AxisName.SLIDE],
        positions[AxisName.Z],
        positions[AxisName.SHOULDER],
        positions[AxisName.ELBOW],
        positions[AxisName.ROTATION],
    )


def _target_positions(target: MultiAxisTarget) -> dict[AxisName, float]:
    return {item.axis: item.position for item in target.targets}


def _state_with_overrides(
    state: RobotAxisState,
    overrides: dict[AxisName, float],
) -> RobotAxisState:
    return RobotAxisState(
        overrides.get(AxisName.SLIDE, state.slide_mm),
        overrides.get(AxisName.Z, state.z_mm),
        overrides.get(AxisName.SHOULDER, state.shoulder_deg),
        overrides.get(AxisName.ELBOW, state.elbow_deg),
        overrides.get(AxisName.ROTATION, state.rotation_deg),
    )


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite number")
    return converted


if __name__ == "__main__":
    raise SystemExit(main())

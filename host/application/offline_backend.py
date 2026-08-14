"""纯配置/算法 Base 规划后端；不发现、打开或写入任何硬件。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from application.controller import MushroomRobotController
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import hand_eye_from_frame_document
from config.motion_runtime import load_robot_motion_config
from config.project.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.project.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from config.project.robot_motion_envelope import (
    DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG, RobotMotionEnvelopeConfig,
)
from config.project.workspace_planning import DEFAULT_OFFSET_WORKSPACE_CONFIG, OffsetWorkspaceConfig
from config.robot_runtime import RobotRuntimeConfig
from geometry.rigid_transform import RigidTransform
from kinematics.base_frame_solver import BaseFrameFiveAxisSolver, BaseFrameSolverConfig
from kinematics.base_move_transition_planner import BaseMovePlan, BaseMoveTransitionPlanner
from kinematics.five_axis import FiveAxisKinematics, load_robot_five_axis_kinematics
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import AxisCapabilities, AxisDescriptor, AxisKind, AxisName, AxisState
from vision.observation import CaptureMotionState
from vision.target_resolver import VisionTargetResolver


class OfflinePlanningStateError(RuntimeError):
    pass


@dataclass(frozen=True)
class OfflinePlanningStatus:
    axis_state: RobotAxisState | None
    joints_holding: bool
    started: bool
    submitted_hardware_commands: int = 0


class OfflinePlanningBackend:
    def __init__(self, solver: BaseFrameFiveAxisSolver, planner: BaseMoveTransitionPlanner, *, motion_envelope: RobotMotionEnvelopeConfig) -> None:
        self.solver = solver
        self.planner = planner
        self.motion_envelope = motion_envelope
        self.axis_state: RobotAxisState | None = None
        self.started = False
        self.holding = False

    def startup(self) -> None:
        from scripts.run_motion_demo import solve_startup_safe_pose

        definition = self.motion_envelope.startup_pose
        seed = RobotAxisState(definition.slide_mm, definition.z_axis_mm, 0.0, 0.0, 0.0)
        solved = solve_startup_safe_pose(self.solver, current_state=seed, definition=definition)
        self.axis_state = solved.solution.axis_state()
        self.started = True
        self.holding = True

    def require_base_motion_ready(self) -> None:
        if not self.started or self.axis_state is None:
            raise OfflinePlanningStateError("offline planner has not completed startup")
        if not self.holding:
            raise OfflinePlanningStateError("rotary joints are not holding")

    def plan_to_base_pose(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float | None) -> BaseMovePlan:
        self.require_base_motion_ready()
        assert self.axis_state is not None
        current = self.solver.forward_kinematics_base(self.axis_state)
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=x_mm, y_mm=y_mm, z_mm=z_mm,
            yaw_deg=current.yaw_deg if yaw_deg is None else yaw_deg,
        )
        return self.planner.plan(current_state=self.axis_state, base_T_tool_target=target)

    def execute_base_plan(self, plan: object) -> bool:
        """仅推进虚拟状态；没有可连接的硬件对象或 submit API。"""
        if not isinstance(plan, BaseMovePlan) or not plan.stages:
            raise TypeError("plan must be a non-empty BaseMovePlan")
        self.axis_state = plan.stages[-1].solution.axis_state()
        return True

    def plan_base_sequence(self, targets: tuple[object, ...]) -> tuple[BaseMovePlan, ...]:
        from application.motion_target import BaseToolTarget

        self.require_base_motion_ready()
        assert self.axis_state is not None
        state = self.axis_state
        plans: list[BaseMovePlan] = []
        for target in targets:
            if not isinstance(target, BaseToolTarget):
                raise TypeError("sequence targets must be BaseToolTarget")
            current_pose = self.solver.forward_kinematics_base(state)
            transform = RigidTransform.from_xyz_yaw_deg(
                x_mm=target.x_mm, y_mm=target.y_mm, z_mm=target.z_mm,
                yaw_deg=current_pose.yaw_deg if target.yaw_deg is None else target.yaw_deg,
            )
            plan = self.planner.plan(current_state=state, base_T_tool_target=transform)
            plans.append(plan)
            state = plan.stages[-1].solution.axis_state()
        return tuple(plans)

    def return_to_startup(self) -> None:
        self.startup()

    def stop(self) -> None:
        return None

    def enable_joints(self) -> None:
        self.holding = True

    def disable_joints(self) -> None:
        self.holding = False

    def suction_grip(self) -> None:
        return None

    def suction_release(self) -> None:
        return None

    def suction_idle(self) -> None:
        return None

    def get_status(self) -> OfflinePlanningStatus:
        return OfflinePlanningStatus(self.axis_state, self.holding, self.started)

    def shutdown(self) -> None:
        self.started = False
        self.holding = False

    def capture_state(self) -> tuple[RobotAxisState, CaptureMotionState]:
        self.require_base_motion_ready()
        assert self.axis_state is not None
        return self.axis_state, CaptureMotionState.STATIONARY

    def joints_holding(self) -> bool:
        return self.holding

    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        descriptors = _offline_descriptors()
        return tuple(descriptors[axis] for axis in AxisName)

    def get_state(self, axis: AxisName) -> AxisState:
        return next(state for state in self.get_axis_states((axis,)))

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]:
        if self.axis_state is None:
            raise OfflinePlanningStateError(
                "simulated axis state is unavailable before dry-run startup"
            )
        selected = tuple(AxisName) if axes is None else axes
        values = {
            AxisName.SLIDE: self.axis_state.slide_mm,
            AxisName.Z: self.axis_state.z_mm,
            AxisName.SHOULDER: self.axis_state.shoulder_deg,
            AxisName.ELBOW: self.axis_state.elbow_deg,
            AxisName.ROTATION: self.axis_state.rotation_deg,
        }
        return tuple(
            AxisState(
                axis=axis,
                connected=False,
                enabled=self.holding if axis not in (AxisName.SLIDE, AxisName.Z) else None,
                busy=False,
                homed=None,
                position_valid=True,
                current_position=values[axis],
                position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
                faulted=False,
                fault_code=None,
                fault_message="simulated dry-run state; not hardware feedback",
            )
            for axis in selected
        )


def create_offline_planning_controller(
    *,
    runtime_config: RobotRuntimeConfig,
    offset_workspace_config: OffsetWorkspaceConfig = DEFAULT_OFFSET_WORKSPACE_CONFIG,
    motion_envelope: RobotMotionEnvelopeConfig = DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG,
) -> tuple[MushroomRobotController, OfflinePlanningBackend]:
    solver = BaseFrameFiveAxisSolver(
        five_axis_kinematics=load_robot_five_axis_kinematics(),
        axis_descriptors=_offline_descriptors(),
        config=BaseFrameSolverConfig(workspace=offset_workspace_config),
    )
    backend = OfflinePlanningBackend(
        solver,
        BaseMoveTransitionPlanner(solver, motion_envelope=motion_envelope),
        motion_envelope=motion_envelope,
    )
    resolver = VisionTargetResolver(
        pose_provider=solver,
        hand_eye_calibration=hand_eye_from_frame_document(
            runtime_config.frame_transforms,
            source=f"{runtime_config.source_path}#frame_transforms",
        ),
        camera_frame_id=runtime_config.vision_runtime.camera_frame,
    )
    controller = MushroomRobotController(
        base_backend=backend,
        tray_workspace=TrayWorkspace(runtime_config.tray_workspace),
        target_resolver=resolver,
    )
    return controller, backend


def _offline_descriptors() -> dict[AxisName, AxisDescriptor]:
    motion = load_robot_motion_config()
    limits = {
        AxisName.SLIDE: motion.linear_position_limits()[AxisName.SLIDE],
        AxisName.Z: motion.linear_position_limits()[AxisName.Z],
        AxisName.SHOULDER: (math.degrees(SHOULDER_JOINT_CONFIG.min_position_rad), math.degrees(SHOULDER_JOINT_CONFIG.max_position_rad)),
        AxisName.ELBOW: (math.degrees(ELBOW_JOINT_CONFIG.min_position_rad), math.degrees(ELBOW_JOINT_CONFIG.max_position_rad)),
        AxisName.ROTATION: (math.degrees(END_EFFECTOR_ROTATION_CONFIG.min_position_rad), math.degrees(END_EFFECTOR_ROTATION_CONFIG.max_position_rad)),
    }
    no_hardware = AxisCapabilities(False, False, False, False, False, False, False)
    result = {}
    for axis in AxisName:
        linear = axis in (AxisName.SLIDE, AxisName.Z)
        minimum, maximum = limits[axis]
        result[axis] = AxisDescriptor(
            axis, axis.value, AxisKind.LINEAR if linear else AxisKind.ROTARY,
            "mm" if linear else "deg", "mm/s" if linear else "deg/s",
            "mm/s^2" if linear else "deg/s^2", minimum, maximum, no_hardware,
        )
    return result


__all__ = [
    "OfflinePlanningBackend", "OfflinePlanningStateError", "OfflinePlanningStatus",
    "create_offline_planning_controller",
]

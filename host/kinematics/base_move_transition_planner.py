"""正负偏置工作区之间的只读安全阶段规划。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from config.robot_motion_envelope import RobotMotionEnvelopeConfig
from config.workspace_planning import OffsetWorkspaceSide, SlideSelectionReason
from geometry.rigid_transform import RigidTransform, angular_difference_deg
from kinematics.base_frame_solver import (
    BaseFrameFiveAxisSolver,
    FiveAxisNoSolutionError,
    FiveAxisSolution,
)
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import AxisName, MultiAxisTarget


class BaseMovePlanningError(ValueError):
    """Base 目标无法形成完整、安全、可验证的只读计划。"""


class CurrentStateInvalidError(BaseMovePlanningError):
    """当前五轴状态无法通过完整 FK/限位验证。"""


class ClearanceHeightUnreachableError(BaseMovePlanningError):
    """跨区要求的 Base 安全高度超出 Z 轴可达范围。"""

    def __init__(
        self,
        *,
        required_clearance_base_z_mm: float,
        current_base_z_mm: float,
        target_base_z_mm: float,
        z_logical_limit: tuple[float, float],
        detail: str,
    ) -> None:
        super().__init__(
            "required clearance Base Z is unreachable: "
            f"required={required_clearance_base_z_mm} mm, "
            f"current={current_base_z_mm} mm, target={target_base_z_mm} mm, "
            f"Z logical limit={z_logical_limit} mm; {detail}"
        )
        self.required_clearance_base_z_mm = required_clearance_base_z_mm
        self.current_base_z_mm = current_base_z_mm
        self.target_base_z_mm = target_base_z_mm
        self.z_logical_limit = z_logical_limit


class StageValidationFailedError(BaseMovePlanningError):
    """约束构造的某个阶段未通过完整五轴/FK 验证。"""


class BaseMoveStageKind(str, Enum):
    DIRECT = "direct"
    LIFT = "lift"
    TRANSIT = "transit"
    LOWER = "lower"


@dataclass(frozen=True)
class BaseMoveStage:
    kind: BaseMoveStageKind
    base_T_tool_target: RigidTransform
    solution: FiveAxisSolution
    multi_axis_target: MultiAxisTarget


@dataclass(frozen=True)
class BaseMovePlan:
    current_base_T_tool: RigidTransform
    requested_base_T_tool_target: RigidTransform
    current_local_x_mm: float
    current_local_y_mm: float
    current_workspace_side: OffsetWorkspaceSide
    target_workspace_side: OffsetWorkspaceSide
    requires_side_switch_clearance: bool
    clearance_lift_mm: float
    clearance_base_z_mm: float | None
    stages: tuple[BaseMoveStage, ...]


class BaseMoveTransitionPlanner:
    """不访问硬件的 DIRECT 或 LIFT/TRANSIT/LOWER 计划器。"""

    def __init__(
        self,
        solver: BaseFrameFiveAxisSolver,
        *,
        motion_envelope: RobotMotionEnvelopeConfig,
    ) -> None:
        if not isinstance(solver, BaseFrameFiveAxisSolver):
            raise TypeError("solver must be BaseFrameFiveAxisSolver")
        if not isinstance(motion_envelope, RobotMotionEnvelopeConfig):
            raise TypeError("motion_envelope must be RobotMotionEnvelopeConfig")
        self.solver = solver
        self.motion_envelope = motion_envelope

    def plan(
        self,
        *,
        current_state: RobotAxisState,
        base_T_tool_target: RigidTransform,
    ) -> BaseMovePlan:
        if not isinstance(current_state, RobotAxisState):
            raise TypeError("current_state must be RobotAxisState")
        if not isinstance(base_T_tool_target, RigidTransform):
            raise TypeError("base_T_tool_target must be RigidTransform")
        current_base = self.solver.forward_kinematics_base(current_state)
        current_side, current_local_x, current_local_y = (
            self.solver.workspace_side_for_state(current_state)
        )
        try:
            self.solver.constrained_solution(
                base_T_tool_target=current_base,
                axis_state=current_state,
                slide_selection_reason=SlideSelectionReason.KEEP_CURRENT_SLIDE,
                elbow_branch=_branch_name(current_state.elbow_deg),
                allow_outside_workspace=True,
            )
        except FiveAxisNoSolutionError as exc:
            raise CurrentStateInvalidError(
                f"current five-axis state is invalid: stage={exc.stage}; {exc}"
            ) from exc

        if (
            current_side is OffsetWorkspaceSide.OUTSIDE
            and self._base_planar_pose_equal(current_base, base_T_tool_target)
        ):
            try:
                direct_solution = self._constrained_at_base_pose(
                    base_T_tool_target,
                    current_state,
                    SlideSelectionReason.KEEP_CURRENT_SLIDE,
                    _branch_name(current_state.elbow_deg),
                    allow_outside_workspace=True,
                )
            except FiveAxisNoSolutionError:
                pass
            else:
                return BaseMovePlan(
                    current_base_T_tool=current_base,
                    requested_base_T_tool_target=base_T_tool_target,
                    current_local_x_mm=current_local_x,
                    current_local_y_mm=current_local_y,
                    current_workspace_side=current_side,
                    target_workspace_side=direct_solution.workspace_side,
                    requires_side_switch_clearance=False,
                    clearance_lift_mm=0.0,
                    clearance_base_z_mm=None,
                    stages=(
                        self._stage(
                            BaseMoveStageKind.DIRECT,
                            base_T_tool_target,
                            direct_solution,
                        ),
                    ),
                )

        final_solution = self.solver.solve_base_target(
            base_T_tool_target=base_T_tool_target,
            current_state=current_state,
        )
        target_side = final_solution.workspace_side
        if target_side is OffsetWorkspaceSide.OUTSIDE:
            raise BaseMovePlanningError(
                "solver returned an OUTSIDE final solution; this is an internal error"
            )

        same_allowed_side = (
            current_side is target_side
            and current_side
            in (OffsetWorkspaceSide.POSITIVE, OffsetWorkspaceSide.NEGATIVE)
        )
        if same_allowed_side:
            stage = self._stage(
                BaseMoveStageKind.DIRECT,
                base_T_tool_target,
                final_solution,
            )
            return BaseMovePlan(
                current_base_T_tool=current_base,
                requested_base_T_tool_target=base_T_tool_target,
                current_local_x_mm=current_local_x,
                current_local_y_mm=current_local_y,
                current_workspace_side=current_side,
                target_workspace_side=target_side,
                requires_side_switch_clearance=False,
                clearance_lift_mm=0.0,
                clearance_base_z_mm=None,
                stages=(stage,),
            )

        current_z = float(current_base.translation_mm[2])
        target_z = float(base_T_tool_target.translation_mm[2])
        clearance_z = max(
            current_z,
            target_z,
            self.motion_envelope.side_switch.clearance_base_z_mm,
        )
        clearance_lift = max(0.0, clearance_z - current_z)
        lift_target = _pose_with_z(current_base, clearance_z)
        transit_target = _pose_with_z(base_T_tool_target, clearance_z)

        try:
            lift_solution = self._constrained_at_base_pose(
                lift_target,
                current_state,
                SlideSelectionReason.KEEP_CURRENT_SLIDE,
                _branch_name(current_state.elbow_deg),
                allow_outside_workspace=(
                    current_side is OffsetWorkspaceSide.OUTSIDE
                ),
            )
            transit_planar = RobotAxisState(
                final_solution.slide_mm,
                final_solution.z_mm,
                final_solution.shoulder_deg,
                final_solution.elbow_deg,
                final_solution.rotation_deg,
            )
            transit_solution = self._constrained_at_base_pose(
                transit_target,
                transit_planar,
                final_solution.slide_selection_reason,
                final_solution.elbow_branch,
                allow_outside_workspace=False,
            )
        except FiveAxisNoSolutionError as exc:
            z_descriptor = self.solver.axis_descriptors[AxisName.Z]
            if exc.stage == "z_limit":
                raise ClearanceHeightUnreachableError(
                    required_clearance_base_z_mm=clearance_z,
                    current_base_z_mm=current_z,
                    target_base_z_mm=target_z,
                    z_logical_limit=(
                        z_descriptor.minimum_position,
                        z_descriptor.maximum_position,
                    ),
                    detail=str(exc),
                ) from exc
            raise StageValidationFailedError(
                f"clearance stage validation failed at {exc.stage}: {exc}"
            ) from exc

        try:
            lower_solution = self.solver.constrained_solution(
                base_T_tool_target=base_T_tool_target,
                axis_state=final_solution.axis_state(),
                slide_selection_reason=final_solution.slide_selection_reason,
                elbow_branch=final_solution.elbow_branch,
            )
        except FiveAxisNoSolutionError as exc:
            raise StageValidationFailedError(
                f"LOWER stage validation failed at {exc.stage}: {exc}"
            ) from exc
        stages = (
            self._stage(BaseMoveStageKind.LIFT, lift_target, lift_solution),
            self._stage(
                BaseMoveStageKind.TRANSIT,
                transit_target,
                transit_solution,
            ),
            self._stage(BaseMoveStageKind.LOWER, base_T_tool_target, lower_solution),
        )
        return BaseMovePlan(
            current_base_T_tool=current_base,
            requested_base_T_tool_target=base_T_tool_target,
            current_local_x_mm=current_local_x,
            current_local_y_mm=current_local_y,
            current_workspace_side=current_side,
            target_workspace_side=target_side,
            requires_side_switch_clearance=True,
            clearance_lift_mm=clearance_lift,
            clearance_base_z_mm=clearance_z,
            stages=stages,
        )

    def _constrained_at_base_pose(
        self,
        target: RigidTransform,
        planar_state: RobotAxisState,
        reason: SlideSelectionReason,
        branch: str,
        *,
        allow_outside_workspace: bool,
    ) -> FiveAxisSolution:
        slide_zero_target = self.solver.transform_base_target_to_slide_zero(target)
        local = self.solver.five_axis_kinematics.compute_arm_local_target(
            slide_zero_target,
            planar_state.slide_mm,
        )
        state = RobotAxisState(
            planar_state.slide_mm,
            local.z_axis_mm,
            planar_state.shoulder_deg,
            planar_state.elbow_deg,
            planar_state.rotation_deg,
        )
        return self.solver.constrained_solution(
            base_T_tool_target=target,
            axis_state=state,
            slide_selection_reason=reason,
            elbow_branch=branch,
            allow_outside_workspace=allow_outside_workspace,
        )

    def _stage(
        self,
        kind: BaseMoveStageKind,
        target: RigidTransform,
        solution: FiveAxisSolution,
    ) -> BaseMoveStage:
        return BaseMoveStage(
            kind=kind,
            base_T_tool_target=target,
            solution=solution,
            multi_axis_target=self.solver.solution_to_multi_axis_target(solution),
        )

    def _base_planar_pose_equal(
        self,
        current_base: RigidTransform,
        target_base: RigidTransform,
    ) -> bool:
        linear_tolerance = self.solver.config.position_equality_tolerance_mm
        angular_tolerance = self.solver.config.angle_equality_tolerance_deg
        xy_equal = all(
            math.isclose(
                float(current_base.translation_mm[index]),
                float(target_base.translation_mm[index]),
                rel_tol=0.0,
                abs_tol=linear_tolerance,
            )
            for index in (0, 1)
        )
        yaw_equal = abs(
            angular_difference_deg(current_base.yaw_deg, target_base.yaw_deg)
        ) <= angular_tolerance
        return xy_equal and yaw_equal


def _pose_with_z(source: RigidTransform, z_mm: float) -> RigidTransform:
    x_mm, y_mm, _ = (float(value) for value in source.translation_mm)
    roll_deg, pitch_deg, yaw_deg = (float(value) for value in source.rpy_deg)
    return RigidTransform.from_xyz_rpy_deg(
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
    )


def _branch_name(elbow_deg: float) -> str:
    if math.isclose(elbow_deg, 0.0, abs_tol=1e-9) or math.isclose(
        abs(elbow_deg), 180.0, abs_tol=1e-9
    ):
        return "singular"
    return "elbow-positive" if elbow_deg > 0 else "elbow-negative"


__all__ = [
    "BaseMovePlan",
    "BaseMovePlanningError",
    "BaseMoveStage",
    "BaseMoveStageKind",
    "BaseMoveTransitionPlanner",
    "ClearanceHeightUnreachableError",
    "CurrentStateInvalidError",
    "StageValidationFailedError",
]

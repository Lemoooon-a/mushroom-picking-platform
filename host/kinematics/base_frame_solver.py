"""Base 根目标到受偏置工作区约束的五轴逻辑目标求解。"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
import math

import numpy as np

from config.project.workspace_planning import (
    DEFAULT_OFFSET_WORKSPACE_CONFIG,
    OffsetWorkspaceConfig,
    OffsetWorkspaceSide,
    SlideSelectionReason,
)
from geometry.rigid_transform import RigidTransform, angular_difference_deg
from kinematics.five_axis import FiveAxisKinematics, rotation_deg_for_output_yaw
from kinematics.frame_chain import RobotAxisState
from kinematics.planar_2r import UnreachableTargetError
from motion.unified_protocol import (
    AxisDescriptor,
    AxisName,
    AxisTarget,
    MultiAxisTarget,
)


_AXIS_ORDER = tuple(AxisName)
_LINEAR_AXES = frozenset((AxisName.SLIDE, AxisName.Z))
_SIDE_ORDER = {
    OffsetWorkspaceSide.POSITIVE: 0,
    OffsetWorkspaceSide.NEGATIVE: 1,
    OffsetWorkspaceSide.OUTSIDE: 2,
}


class BaseFrameSolverError(ValueError):
    """Base-frame 五轴求解配置或输入无效。"""


class UnvalidatedBaseTransformError(BaseFrameSolverError):
    """未通过独立验证的 Base–Slide-zero 变换不可用于正式规划。"""


class FiveAxisNoSolutionError(BaseFrameSolverError):
    """分层候选搜索后没有通过工作区、限位和 FK 验证的五轴解。"""

    def __init__(
        self,
        message: str,
        *,
        stage: str,
        stage_counts: Mapping[str, int],
    ) -> None:
        super().__init__(message)
        self.stage = stage
        self.stage_counts = dict(stage_counts)


@dataclass(frozen=True)
class SolverWeights:
    """保留的诊断评分权重；候选优先级不由加权总分决定。"""

    slide: float = 4.0
    shoulder: float = 1.0
    elbow: float = 1.0
    rotation: float = 0.75
    limit_margin: float = 0.1

    def __post_init__(self) -> None:
        for field_name in ("slide", "shoulder", "elbow", "rotation", "limit_margin"):
            _require_nonnegative(field_name, getattr(self, field_name))


@dataclass(frozen=True)
class BaseFrameSolverConfig:
    """模型兼容性、数值容差和集中偏置工作区配置。"""

    model_roll_pitch_tolerance_deg: float = 1e-6
    position_residual_tolerance_mm: float = 1e-6
    yaw_residual_tolerance_deg: float = 1e-6
    linear_solve_tolerance: float = 1e-9
    position_equality_tolerance_mm: float = 1e-6
    angle_equality_tolerance_deg: float = 1e-6
    workspace: OffsetWorkspaceConfig = field(
        default_factory=lambda: DEFAULT_OFFSET_WORKSPACE_CONFIG
    )
    weights: SolverWeights = field(default_factory=SolverWeights)

    def __post_init__(self) -> None:
        for field_name in (
            "model_roll_pitch_tolerance_deg",
            "position_residual_tolerance_mm",
            "yaw_residual_tolerance_deg",
            "linear_solve_tolerance",
            "position_equality_tolerance_mm",
            "angle_equality_tolerance_deg",
        ):
            _require_positive(field_name, getattr(self, field_name))
        if not isinstance(self.workspace, OffsetWorkspaceConfig):
            raise TypeError("workspace must be OffsetWorkspaceConfig")
        if not isinstance(self.weights, SolverWeights):
            raise TypeError("weights must be SolverWeights")


@dataclass(frozen=True)
class FiveAxisSolution:
    """一组通过偏置矩形、五轴限位和完整 FK 重建的逻辑轴目标。"""

    slide_mm: float
    z_mm: float
    shoulder_deg: float
    elbow_deg: float
    rotation_deg: float
    local_x_mm: float
    local_y_mm: float
    workspace_side: OffsetWorkspaceSide
    slide_selection_reason: SlideSelectionReason
    elbow_branch: str
    position_error_xyz_mm: tuple[float, float, float]
    position_residual_mm: float
    yaw_residual_deg: float
    score: float
    limit_margins: tuple[tuple[AxisName, float], ...]

    @property
    def branch(self) -> str:
        """兼容旧诊断字段名。"""

        return self.elbow_branch

    @property
    def fk_translation_residual_mm(self) -> float:
        """完整正运动学（Forward Kinematics, FK）平移重建残差。"""

        return self.position_residual_mm

    @property
    def fk_yaw_residual_deg(self) -> float:
        """完整 FK 的 yaw 重建残差。"""

        return self.yaw_residual_deg

    def axis_state(self) -> RobotAxisState:
        return RobotAxisState(
            self.slide_mm,
            self.z_mm,
            self.shoulder_deg,
            self.elbow_deg,
            self.rotation_deg,
        )


class BaseFrameFiveAxisSolver:
    """按当前 Slide、偏置中心、有限 fallback 的固定层级求五轴解。"""

    def __init__(
        self,
        *,
        five_axis_kinematics: FiveAxisKinematics,
        base_T_slide_zero: RigidTransform,
        axis_descriptors: Mapping[AxisName, AxisDescriptor],
        base_transform_validated: bool,
        allow_unvalidated_base_transform: bool = False,
        config: BaseFrameSolverConfig | None = None,
    ) -> None:
        if not isinstance(five_axis_kinematics, FiveAxisKinematics):
            raise TypeError("five_axis_kinematics must be FiveAxisKinematics")
        if not isinstance(base_T_slide_zero, RigidTransform):
            raise TypeError("base_T_slide_zero must be RigidTransform")
        if not isinstance(base_transform_validated, bool):
            raise TypeError("base_transform_validated must be bool")
        if not isinstance(allow_unvalidated_base_transform, bool):
            raise TypeError("allow_unvalidated_base_transform must be bool")
        if not base_transform_validated and not allow_unvalidated_base_transform:
            raise UnvalidatedBaseTransformError(
                "The Base–Slide-zero transform is provisional and has not passed "
                "an independent pose validation."
            )
        self.five_axis_kinematics = five_axis_kinematics
        self.base_T_slide_zero = base_T_slide_zero
        self.axis_descriptors = _validate_descriptors(axis_descriptors)
        self.base_transform_validated = base_transform_validated
        self.config = config or BaseFrameSolverConfig()
        if not isinstance(self.config, BaseFrameSolverConfig):
            raise TypeError("config must be BaseFrameSolverConfig")
        self._validate_inverse_geometry()

    @property
    def workspace(self) -> OffsetWorkspaceConfig:
        return self.config.workspace

    def transform_base_target_to_slide_zero(
        self,
        base_T_tool_target: RigidTransform,
    ) -> RigidTransform:
        if not isinstance(base_T_tool_target, RigidTransform):
            raise TypeError("base_T_tool_target must be RigidTransform")
        return self.base_T_slide_zero.inverse() @ base_T_tool_target

    def forward_kinematics_base(self, state: RobotAxisState) -> RigidTransform:
        if not isinstance(state, RobotAxisState):
            raise TypeError("state must be RobotAxisState")
        return self.base_T_slide_zero @ self.five_axis_kinematics.forward_kinematics(
            state
        )

    def workspace_side_for_state(
        self,
        state: RobotAxisState,
    ) -> tuple[OffsetWorkspaceSide, float, float]:
        """用当前真实 FK 和统一局部 helper 判断当前偏置区。"""

        slide_zero_T_tool = self.five_axis_kinematics.forward_kinematics(state)
        local = self.five_axis_kinematics.compute_arm_local_target(
            slide_zero_T_tool,
            state.slide_mm,
        )
        side = self.workspace.classify(local.local_x_mm, local.local_y_mm)
        return side, local.local_x_mm, local.local_y_mm

    def solve_base_target(
        self,
        *,
        base_T_tool_target: RigidTransform,
        current_state: RobotAxisState,
    ) -> FiveAxisSolution:
        return self.solve_base_target_candidates(
            base_T_tool_target=base_T_tool_target,
            current_state=current_state,
        )[0]

    def solve_with_fixed_slide(
        self,
        *,
        base_T_tool_target: RigidTransform,
        current_state: RobotAxisState,
        slide_mm: float,
    ) -> FiveAxisSolution:
        return self.solve_base_target_candidates(
            base_T_tool_target=base_T_tool_target,
            current_state=current_state,
            fixed_slide_mm=slide_mm,
        )[0]

    def solve_base_target_candidates(
        self,
        *,
        base_T_tool_target: RigidTransform,
        current_state: RobotAxisState,
        fixed_slide_mm: float | None = None,
    ) -> tuple[FiveAxisSolution, ...]:
        """只返回最高可用优先级内的合法候选。"""

        if not isinstance(current_state, RobotAxisState):
            raise TypeError("current_state must be RobotAxisState")
        slide_zero_target = self.transform_base_target_to_slide_zero(
            base_T_tool_target
        )
        output_yaw_deg = self._rotation_output_yaw(slide_zero_target)
        counts = _empty_counts()

        if fixed_slide_mm is not None:
            fixed = _require_finite("fixed_slide_mm", fixed_slide_mm)
            solutions = self._solutions_for_slide(
                slide_zero_target,
                output_yaw_deg,
                current_state,
                fixed,
                SlideSelectionReason.FIXED_SLIDE,
                counts,
            )
            return self._finish_priority(solutions, current_state, counts, "fixed_slide")

        current_solutions = self._solutions_for_slide(
            slide_zero_target,
            output_yaw_deg,
            current_state,
            current_state.slide_mm,
            SlideSelectionReason.KEEP_CURRENT_SLIDE,
            counts,
        )
        if current_solutions:
            return self._sort_priority(current_solutions, current_state)

        center_solutions: list[FiveAxisSolution] = []
        for side in (OffsetWorkspaceSide.POSITIVE, OffsetWorkspaceSide.NEGATIVE):
            center_slide = self._slide_for_local_y(
                slide_zero_target,
                self.workspace.center_y(side),
            )
            center_solutions.extend(
                self._solutions_for_slide(
                    slide_zero_target,
                    output_yaw_deg,
                    current_state,
                    center_slide,
                    self.workspace.center_reason(side),
                    counts,
                )
            )
        if center_solutions:
            return self._sort_priority(center_solutions, current_state)

        current_local = self.five_axis_kinematics.compute_arm_local_target(
            slide_zero_target,
            current_state.slide_mm,
        )
        fallback_solutions: list[FiveAxisSolution] = []
        seen_slides: list[float] = []
        for side in (OffsetWorkspaceSide.POSITIVE, OffsetWorkspaceSide.NEGATIVE):
            for local_y in self.workspace.fallback_local_y_candidates(
                side,
                current_local.local_y_mm,
            ):
                slide = self._slide_for_local_y(slide_zero_target, local_y)
                if any(
                    math.isclose(
                        slide,
                        existing,
                        rel_tol=0.0,
                        abs_tol=self.workspace.boundary_tolerance_mm,
                    )
                    for existing in seen_slides
                ):
                    continue
                seen_slides.append(slide)
                fallback_solutions.extend(
                    self._solutions_for_slide(
                        slide_zero_target,
                        output_yaw_deg,
                        current_state,
                        slide,
                        self.workspace.fallback_reason(side),
                        counts,
                    )
                )
        return self._finish_priority(
            fallback_solutions,
            current_state,
            counts,
            "offset_fallback",
        )

    def constrained_solution(
        self,
        *,
        base_T_tool_target: RigidTransform,
        axis_state: RobotAxisState,
        slide_selection_reason: SlideSelectionReason,
        elbow_branch: str,
        allow_outside_workspace: bool = False,
    ) -> FiveAxisSolution:
        """验证规划器约束构造的完整五轴阶段，不重新选择平面解。"""

        if not isinstance(axis_state, RobotAxisState):
            raise TypeError("axis_state must be RobotAxisState")
        if not isinstance(slide_selection_reason, SlideSelectionReason):
            raise TypeError("slide_selection_reason must be SlideSelectionReason")
        slide_zero_target = self.transform_base_target_to_slide_zero(
            base_T_tool_target
        )
        local = self.five_axis_kinematics.compute_arm_local_target(
            slide_zero_target,
            axis_state.slide_mm,
        )
        side = self.workspace.classify(local.local_x_mm, local.local_y_mm)
        if side is OffsetWorkspaceSide.OUTSIDE and not allow_outside_workspace:
            raise FiveAxisNoSolutionError(
                "constrained stage is outside both offset workspaces",
                stage="outside_offset_workspace",
                stage_counts={"outside_offset_workspace": 1},
            )
        for axis, value in _state_positions(axis_state).items():
            if not self._within_limit(axis, value):
                raise FiveAxisNoSolutionError(
                    f"constrained stage {axis.value}={value} is outside {self._limits(axis)}",
                    stage=f"{axis.value}_limit",
                    stage_counts={f"{axis.value}_limit": 1},
                )
        return self._solution_from_state(
            slide_zero_target=slide_zero_target,
            state=axis_state,
            local_x_mm=local.local_x_mm,
            local_y_mm=local.local_y_mm,
            workspace_side=side,
            slide_selection_reason=slide_selection_reason,
            elbow_branch=elbow_branch,
            current_state=axis_state,
            reject_residual=True,
        )

    def solution_to_multi_axis_target(
        self,
        solution: FiveAxisSolution,
        *,
        axes: Iterable[AxisName] | None = None,
        velocity_overrides: Mapping[AxisName, float] | None = None,
        acceleration_overrides: Mapping[AxisName, float] | None = None,
    ) -> MultiAxisTarget:
        if not isinstance(solution, FiveAxisSolution):
            raise TypeError("solution must be FiveAxisSolution")
        velocities = dict(velocity_overrides or {})
        accelerations = dict(acceleration_overrides or {})
        _validate_override_axes("velocity_overrides", velocities)
        _validate_override_axes("acceleration_overrides", accelerations)
        positions = _state_positions(solution.axis_state())
        selected_axes = tuple(_AXIS_ORDER if axes is None else axes)
        if not selected_axes or len(set(selected_axes)) != len(selected_axes):
            raise ValueError("axes must contain at least one unique axis")
        if any(not isinstance(axis, AxisName) for axis in selected_axes):
            raise TypeError("axes must contain only AxisName values")
        return MultiAxisTarget(
            tuple(
                AxisTarget(
                    axis,
                    positions[axis],
                    velocities.get(axis),
                    accelerations.get(axis),
                )
                for axis in selected_axes
            )
        )

    def _solutions_for_slide(
        self,
        slide_zero_target: RigidTransform,
        output_yaw_deg: float,
        current_state: RobotAxisState,
        slide_mm: float,
        reason: SlideSelectionReason,
        counts: dict[str, int],
    ) -> list[FiveAxisSolution]:
        counts["slide_candidates"] += 1
        if not self._within_limit(AxisName.SLIDE, slide_mm):
            counts["slide_limit"] += 1
            return []
        local = self.five_axis_kinematics.compute_arm_local_target(
            slide_zero_target,
            slide_mm,
        )
        side = self.workspace.classify(local.local_x_mm, local.local_y_mm)
        if side is OffsetWorkspaceSide.OUTSIDE:
            counts["outside_offset_workspace"] += 1
            return []
        if not self._within_limit(AxisName.Z, local.z_axis_mm):
            counts["z_limit"] += 1
            return []
        try:
            planar_solutions = self.five_axis_kinematics.planar_2r.inverse(
                local.local_x_mm,
                local.local_y_mm,
            )
        except UnreachableTargetError:
            counts["planar_unreachable"] += 1
            return []
        solutions: list[FiveAxisSolution] = []
        for planar_solution in planar_solutions:
            shoulder_deg = math.degrees(planar_solution.shoulder_rad)
            elbow_deg = math.degrees(planar_solution.elbow_rad)
            if not self._within_limit(AxisName.SHOULDER, shoulder_deg):
                counts["shoulder_limit"] += 1
                continue
            if not self._within_limit(AxisName.ELBOW, elbow_deg):
                counts["elbow_limit"] += 1
                continue
            rotation_values = self._rotation_candidates(
                output_yaw_deg,
                shoulder_deg,
                elbow_deg,
                current_state.rotation_deg,
            )
            if not rotation_values:
                counts["rotation_limit"] += 1
                continue
            for rotation_deg in rotation_values:
                state = RobotAxisState(
                    slide_mm,
                    local.z_axis_mm,
                    shoulder_deg,
                    elbow_deg,
                    rotation_deg,
                )
                try:
                    solution = self._solution_from_state(
                        slide_zero_target=slide_zero_target,
                        state=state,
                        local_x_mm=local.local_x_mm,
                        local_y_mm=local.local_y_mm,
                        workspace_side=side,
                        slide_selection_reason=reason,
                        elbow_branch=_branch_name(elbow_deg),
                        current_state=current_state,
                        reject_residual=True,
                    )
                except FiveAxisNoSolutionError as exc:
                    counts[exc.stage] = counts.get(exc.stage, 0) + 1
                    continue
                counts["valid"] += 1
                solutions.append(solution)
        return solutions

    def _solution_from_state(
        self,
        *,
        slide_zero_target: RigidTransform,
        state: RobotAxisState,
        local_x_mm: float,
        local_y_mm: float,
        workspace_side: OffsetWorkspaceSide,
        slide_selection_reason: SlideSelectionReason,
        elbow_branch: str,
        current_state: RobotAxisState,
        reject_residual: bool,
    ) -> FiveAxisSolution:
        reconstructed = self.five_axis_kinematics.forward_kinematics(state)
        delta = reconstructed.translation_mm - slide_zero_target.translation_mm
        error_xyz = tuple(float(value) for value in delta)
        position_residual = float(np.linalg.norm(delta))
        yaw_residual = abs(
            angular_difference_deg(reconstructed.yaw_deg, slide_zero_target.yaw_deg)
        )
        if reject_residual and position_residual > self.config.position_residual_tolerance_mm:
            raise FiveAxisNoSolutionError(
                f"FK translation residual {position_residual} mm exceeds tolerance",
                stage="fk_translation_residual",
                stage_counts={"fk_translation_residual": 1},
            )
        if reject_residual and yaw_residual > self.config.yaw_residual_tolerance_deg:
            raise FiveAxisNoSolutionError(
                f"FK yaw residual {yaw_residual} deg exceeds tolerance",
                stage="fk_yaw_residual",
                stage_counts={"fk_yaw_residual": 1},
            )
        margins = self._limit_margins(state)
        return FiveAxisSolution(
            slide_mm=state.slide_mm,
            z_mm=state.z_mm,
            shoulder_deg=state.shoulder_deg,
            elbow_deg=state.elbow_deg,
            rotation_deg=state.rotation_deg,
            local_x_mm=float(local_x_mm),
            local_y_mm=float(local_y_mm),
            workspace_side=workspace_side,
            slide_selection_reason=slide_selection_reason,
            elbow_branch=elbow_branch,
            position_error_xyz_mm=error_xyz,
            position_residual_mm=position_residual,
            yaw_residual_deg=yaw_residual,
            score=self._score(state, current_state, margins),
            limit_margins=margins,
        )

    def _finish_priority(
        self,
        solutions: list[FiveAxisSolution],
        current_state: RobotAxisState,
        counts: Mapping[str, int],
        priority_name: str,
    ) -> tuple[FiveAxisSolution, ...]:
        if solutions:
            return self._sort_priority(solutions, current_state)
        stage = _failure_stage(counts)
        raise FiveAxisNoSolutionError(
            "no five-axis solution passed offset workspace, axis limits, planar "
            f"IK, and FK reconstruction (priority={priority_name}, stage={stage}, "
            f"counts={dict(counts)})",
            stage=stage,
            stage_counts=counts,
        )

    def _sort_priority(
        self,
        solutions: list[FiveAxisSolution],
        current_state: RobotAxisState,
    ) -> tuple[FiveAxisSolution, ...]:
        solutions.sort(
            key=lambda item: (
                abs(item.slide_mm - current_state.slide_mm),
                abs(item.shoulder_deg - current_state.shoulder_deg),
                abs(item.elbow_deg - current_state.elbow_deg),
                abs(item.rotation_deg - current_state.rotation_deg),
                abs(item.local_y_mm - self.workspace.center_y(item.workspace_side)),
                item.position_residual_mm,
                item.yaw_residual_deg,
                _SIDE_ORDER[item.workspace_side],
                item.elbow_branch,
                item.slide_mm,
                item.shoulder_deg,
                item.elbow_deg,
                item.rotation_deg,
            )
        )
        return tuple(solutions)

    def _slide_for_local_y(
        self,
        slide_zero_target: RigidTransform,
        desired_local_y_mm: float,
    ) -> float:
        local_at_zero = self.five_axis_kinematics.compute_arm_local_target(
            slide_zero_target,
            0.0,
        )
        coefficient = self.five_axis_kinematics.slide_local_y_per_mm()
        if abs(coefficient) <= self.config.linear_solve_tolerance:
            raise BaseFrameSolverError(
                "Slide direction has no mechanical-arm local-y component"
            )
        return (local_at_zero.local_y_mm - desired_local_y_mm) / coefficient

    def _rotation_output_yaw(self, slide_zero_target: RigidTransform) -> float:
        geometry = self.five_axis_kinematics.geometry
        output_target = (
            slide_zero_target @ geometry.rotation_output_T_tool.inverse()
        )
        relative_rotation = (
            geometry.slide_zero_T_planar_origin_at_zero.rotation_matrix.T
            @ output_target.rotation_matrix
        )
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = relative_rotation
        roll_deg, pitch_deg, yaw_deg = (
            float(value) for value in RigidTransform(matrix).rpy_deg
        )
        if max(abs(roll_deg), abs(pitch_deg)) > self.config.model_roll_pitch_tolerance_deg:
            raise BaseFrameSolverError(
                "target roll/pitch is incompatible with the yaw-only Rotation output "
                f"model: roll={roll_deg:.9f} deg pitch={pitch_deg:.9f} deg"
            )
        return yaw_deg

    def _rotation_candidates(
        self,
        output_yaw_deg: float,
        shoulder_deg: float,
        elbow_deg: float,
        current_rotation_deg: float,
    ) -> tuple[float, ...]:
        raw = rotation_deg_for_output_yaw(
            output_yaw_deg,
            shoulder_deg,
            elbow_deg,
        )
        minimum, maximum = self._limits(AxisName.ROTATION)
        first_period = math.ceil((minimum - raw) / 360.0 - 1e-12)
        last_period = math.floor((maximum - raw) / 360.0 + 1e-12)
        values = [raw + period * 360.0 for period in range(first_period, last_period + 1)]
        values = [value for value in values if self._within_limit(AxisName.ROTATION, value)]
        values.sort(key=lambda value: (abs(value - current_rotation_deg), value))
        return tuple(values)

    def _validate_inverse_geometry(self) -> None:
        geometry = self.five_axis_kinematics.geometry
        mount_rotation = geometry.slide_zero_T_planar_origin_at_zero.rotation_matrix
        z_in_mount = mount_rotation.T @ np.asarray(geometry.z_direction_xyz)
        if abs(float(z_in_mount[2])) <= self.config.linear_solve_tolerance:
            raise BaseFrameSolverError(
                "z_direction_xyz has no component normal to the configured planar frame"
            )
        slide_in_mount = mount_rotation.T @ np.asarray(geometry.slide_direction_xyz)
        if (
            abs(float(slide_in_mount[0])) > self.config.linear_solve_tolerance
            or abs(float(slide_in_mount[2])) > self.config.linear_solve_tolerance
            or float(slide_in_mount[1]) <= self.config.linear_solve_tolerance
        ):
            raise BaseFrameSolverError(
                "offset-workspace planning requires Slide logical positive to align "
                "with mechanical-arm local +y"
            )

    def _score(
        self,
        candidate: RobotAxisState,
        current: RobotAxisState,
        margins: tuple[tuple[AxisName, float], ...],
    ) -> float:
        weights = self.config.weights
        changes = {
            AxisName.SLIDE: abs(candidate.slide_mm - current.slide_mm),
            AxisName.SHOULDER: abs(candidate.shoulder_deg - current.shoulder_deg),
            AxisName.ELBOW: abs(candidate.elbow_deg - current.elbow_deg),
            AxisName.ROTATION: abs(candidate.rotation_deg - current.rotation_deg),
        }
        score = (
            weights.slide * changes[AxisName.SLIDE] / self._axis_span(AxisName.SLIDE)
            + weights.shoulder * changes[AxisName.SHOULDER] / self._axis_span(AxisName.SHOULDER)
            + weights.elbow * changes[AxisName.ELBOW] / self._axis_span(AxisName.ELBOW)
            + weights.rotation * changes[AxisName.ROTATION] / self._axis_span(AxisName.ROTATION)
        )
        margin_penalty = 0.0
        for axis, margin in margins:
            normalized = min(1.0, 2.0 * margin / self._axis_span(axis))
            margin_penalty += 1.0 - max(0.0, normalized)
        return score + weights.limit_margin * margin_penalty

    def _limit_margins(self, state: RobotAxisState) -> tuple[tuple[AxisName, float], ...]:
        positions = _state_positions(state)
        return tuple(
            (
                axis,
                min(
                    positions[axis] - self._limits(axis)[0],
                    self._limits(axis)[1] - positions[axis],
                ),
            )
            for axis in _AXIS_ORDER
        )

    def _limits(self, axis: AxisName) -> tuple[float, float]:
        descriptor = self.axis_descriptors[axis]
        return descriptor.minimum_position, descriptor.maximum_position

    def _axis_span(self, axis: AxisName) -> float:
        minimum, maximum = self._limits(axis)
        return maximum - minimum

    def _within_limit(self, axis: AxisName, value: float) -> bool:
        minimum, maximum = self._limits(axis)
        tolerance = (
            self.config.position_equality_tolerance_mm
            if axis in _LINEAR_AXES
            else self.config.angle_equality_tolerance_deg
        )
        return minimum - tolerance <= value <= maximum + tolerance


def _validate_descriptors(
    descriptors: Mapping[AxisName, AxisDescriptor],
) -> dict[AxisName, AxisDescriptor]:
    result = dict(descriptors)
    if set(result) != set(_AXIS_ORDER):
        raise BaseFrameSolverError("axis_descriptors must contain exactly all five axes")
    for axis in _AXIS_ORDER:
        descriptor = result[axis]
        if not isinstance(descriptor, AxisDescriptor) or descriptor.name is not axis:
            raise BaseFrameSolverError(
                f"axis_descriptors[{axis.value}] must match its AxisDescriptor"
            )
        expected_unit = "mm" if axis in _LINEAR_AXES else "deg"
        if descriptor.position_unit != expected_unit:
            raise BaseFrameSolverError(
                f"axis {axis.value} position unit must be {expected_unit}"
            )
    return result


def _validate_override_axes(name: str, values: Mapping[AxisName, float]) -> None:
    for axis in values:
        if not isinstance(axis, AxisName):
            raise TypeError(f"{name} keys must be AxisName")


def _state_positions(state: RobotAxisState) -> dict[AxisName, float]:
    return {
        AxisName.SLIDE: state.slide_mm,
        AxisName.Z: state.z_mm,
        AxisName.SHOULDER: state.shoulder_deg,
        AxisName.ELBOW: state.elbow_deg,
        AxisName.ROTATION: state.rotation_deg,
    }


def _branch_name(elbow_deg: float) -> str:
    if math.isclose(elbow_deg, 0.0, abs_tol=1e-9) or math.isclose(
        abs(elbow_deg), 180.0, abs_tol=1e-9
    ):
        return "singular"
    return "elbow-positive" if elbow_deg > 0 else "elbow-negative"


def _empty_counts() -> dict[str, int]:
    return {
        "slide_candidates": 0,
        "slide_limit": 0,
        "outside_offset_workspace": 0,
        "z_limit": 0,
        "planar_unreachable": 0,
        "shoulder_limit": 0,
        "elbow_limit": 0,
        "rotation_limit": 0,
        "fk_translation_residual": 0,
        "fk_yaw_residual": 0,
        "valid": 0,
    }


def _failure_stage(counts: Mapping[str, int]) -> str:
    rejection_order = (
        "outside_offset_workspace",
        "slide_limit",
        "z_limit",
        "planar_unreachable",
        "shoulder_limit",
        "elbow_limit",
        "rotation_limit",
        "fk_translation_residual",
        "fk_yaw_residual",
    )
    populated = [stage for stage in rejection_order if counts.get(stage, 0) > 0]
    if not populated:
        return "candidate_selection"
    return max(populated, key=lambda stage: (counts.get(stage, 0), -rejection_order.index(stage)))


def _require_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _require_positive(name: str, value: object) -> float:
    converted = _require_finite(name, value)
    if converted <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return converted


def _require_nonnegative(name: str, value: object) -> float:
    converted = _require_finite(name, value)
    if converted < 0:
        raise ValueError(f"{name} must be non-negative")
    return converted


__all__ = [
    "BaseFrameFiveAxisSolver",
    "BaseFrameSolverConfig",
    "BaseFrameSolverError",
    "FiveAxisNoSolutionError",
    "FiveAxisSolution",
    "SolverWeights",
    "UnvalidatedBaseTransformError",
]

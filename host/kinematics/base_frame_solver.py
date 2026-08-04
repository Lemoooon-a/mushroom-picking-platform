"""Base 根目标到五轴逻辑目标的纯数学求解。

``FiveAxisKinematics`` 始终以 Slide-zero 为内部根。本模块只在输入边界把
``base_T_tool_target`` 转为 ``slide_zero_T_tool_target``，随后生成、筛选并用
现有 FK 重建五轴候选。模块不读取硬件、startup position 或本机文件。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
import math

import numpy as np

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


class BaseFrameSolverError(ValueError):
    """Base-frame 五轴求解配置或输入无效。"""


class UnvalidatedBaseTransformError(BaseFrameSolverError):
    """未显式授权使用尚未独立验证的 Base–Slide-zero 变换。"""


class FiveAxisNoSolutionError(BaseFrameSolverError):
    """有限候选搜索后没有通过限位和 FK 复核的五轴解。"""

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
    """候选评分权重；Slide 选择仍先按与当前位置的距离排序。"""

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
    """有限 Slide 搜索、模型兼容性和 FK 重建阈值。"""

    slide_search_step_mm: float = 5.0
    model_roll_pitch_tolerance_deg: float = 1e-6
    position_residual_tolerance_mm: float = 1e-6
    yaw_residual_tolerance_deg: float = 1e-6
    linear_solve_tolerance: float = 1e-9
    weights: SolverWeights = field(default_factory=SolverWeights)

    def __post_init__(self) -> None:
        for field_name in (
            "slide_search_step_mm",
            "model_roll_pitch_tolerance_deg",
            "position_residual_tolerance_mm",
            "yaw_residual_tolerance_deg",
            "linear_solve_tolerance",
        ):
            _require_positive(field_name, getattr(self, field_name))
        if not isinstance(self.weights, SolverWeights):
            raise TypeError("weights must be SolverWeights")


@dataclass(frozen=True)
class FiveAxisSolution:
    """一组通过五轴软限位和整体 FK 重建的逻辑轴候选。"""

    slide_mm: float
    z_mm: float
    shoulder_deg: float
    elbow_deg: float
    rotation_deg: float
    position_error_xyz_mm: tuple[float, float, float]
    position_residual_mm: float
    yaw_residual_deg: float
    score: float
    branch: str
    slide_selection_reason: str
    limit_margins: tuple[tuple[AxisName, float], ...]

    def axis_state(self) -> RobotAxisState:
        return RobotAxisState(
            self.slide_mm,
            self.z_mm,
            self.shoulder_deg,
            self.elbow_deg,
            self.rotation_deg,
        )


class BaseFrameFiveAxisSolver:
    """把 Base TCP 目标确定性地选择为一组五轴逻辑目标。

    Slide 冗余采用 ``KEEP_CURRENT_SLIDE_THEN_NEAREST``：有限候选来自正式
    Slide 软限位和配置步长，当前 Slide 被显式加入并优先；其后按 Slide
    变化量、归一化运动评分和稳定数值 tie-break 选择。该策略不声称数学
    唯一，也不包含碰撞或路径最优保证。
    """

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

    def transform_base_target_to_slide_zero(
        self,
        base_T_tool_target: RigidTransform,
    ) -> RigidTransform:
        """应用 ``inverse(base_T_slide_zero) @ base_T_tool_target``。"""

        if not isinstance(base_T_tool_target, RigidTransform):
            raise TypeError("base_T_tool_target must be RigidTransform")
        return self.base_T_slide_zero.inverse() @ base_T_tool_target

    def solve_base_target(
        self,
        *,
        base_T_tool_target: RigidTransform,
        current_state: RobotAxisState,
    ) -> FiveAxisSolution:
        """返回按确定性策略排序后的最优候选。"""

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
        """在调用方明确给定的 Slide 逻辑位置上求解。"""

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
        """返回全部合法候选，按默认冗余策略稳定排序。"""

        if not isinstance(current_state, RobotAxisState):
            raise TypeError("current_state must be RobotAxisState")
        slide_zero_target = self.transform_base_target_to_slide_zero(
            base_T_tool_target
        )
        output_target, output_yaw_deg = self._rotation_output_target(
            slide_zero_target
        )
        slide_values = self._slide_candidates(current_state, fixed_slide_mm)
        counts = {
            "slide_candidates": len(slide_values),
            "z_within_limits": 0,
            "planar_solutions": 0,
            "joint_limits": 0,
            "rotation_limits": 0,
            "fk_verified": 0,
        }
        solutions: list[FiveAxisSolution] = []
        for slide_mm in slide_values:
            z_and_point = self._solve_z_and_planar_point(
                output_target,
                slide_mm,
            )
            z_mm, planar_x, planar_y = z_and_point
            if not self._within_limit(AxisName.Z, z_mm):
                continue
            counts["z_within_limits"] += 1
            try:
                planar_solutions = self.five_axis_kinematics.planar_2r.inverse(
                    planar_x,
                    planar_y,
                )
            except UnreachableTargetError:
                continue
            counts["planar_solutions"] += len(planar_solutions)
            for planar_solution in planar_solutions:
                shoulder_deg = math.degrees(planar_solution.shoulder_rad)
                elbow_deg = math.degrees(planar_solution.elbow_rad)
                if not (
                    self._within_limit(AxisName.SHOULDER, shoulder_deg)
                    and self._within_limit(AxisName.ELBOW, elbow_deg)
                ):
                    continue
                counts["joint_limits"] += 1
                rotation_values = self._rotation_candidates(
                    output_yaw_deg,
                    shoulder_deg,
                    elbow_deg,
                    current_state.rotation_deg,
                )
                if not rotation_values:
                    continue
                counts["rotation_limits"] += len(rotation_values)
                for rotation_deg in rotation_values:
                    candidate_state = RobotAxisState(
                        slide_mm,
                        z_mm,
                        shoulder_deg,
                        elbow_deg,
                        rotation_deg,
                    )
                    reconstructed = self.five_axis_kinematics.forward_kinematics(
                        candidate_state
                    )
                    delta = reconstructed.translation_mm - slide_zero_target.translation_mm
                    error_xyz = tuple(float(value) for value in delta)
                    position_residual = float(np.linalg.norm(delta))
                    yaw_residual = abs(
                        angular_difference_deg(
                            reconstructed.yaw_deg,
                            slide_zero_target.yaw_deg,
                        )
                    )
                    if (
                        position_residual
                        > self.config.position_residual_tolerance_mm
                        or yaw_residual > self.config.yaw_residual_tolerance_deg
                    ):
                        continue
                    counts["fk_verified"] += 1
                    margins = self._limit_margins(candidate_state)
                    solutions.append(
                        FiveAxisSolution(
                            slide_mm=slide_mm,
                            z_mm=z_mm,
                            shoulder_deg=shoulder_deg,
                            elbow_deg=elbow_deg,
                            rotation_deg=rotation_deg,
                            position_error_xyz_mm=error_xyz,
                            position_residual_mm=position_residual,
                            yaw_residual_deg=yaw_residual,
                            score=self._score(candidate_state, current_state, margins),
                            branch=_branch_name(elbow_deg),
                            slide_selection_reason=(
                                "fixed"
                                if fixed_slide_mm is not None
                                else "current"
                                if math.isclose(
                                    slide_mm,
                                    current_state.slide_mm,
                                    rel_tol=0.0,
                                    abs_tol=1e-9,
                                )
                                else "nearest-discrete-candidate"
                            ),
                            limit_margins=margins,
                        )
                    )

        if not solutions:
            stage = _failure_stage(counts)
            raise FiveAxisNoSolutionError(
                "no five-axis solution passed the finite Slide search, axis "
                f"limits, and FK reconstruction (stage={stage}, counts={counts})",
                stage=stage,
                stage_counts=counts,
            )
        slide_span = self._axis_span(AxisName.SLIDE)
        solutions.sort(
            key=lambda item: (
                abs(item.slide_mm - current_state.slide_mm) / slide_span,
                item.score,
                item.slide_mm,
                item.shoulder_deg,
                item.elbow_deg,
                item.rotation_deg,
            )
        )
        return tuple(solutions)

    def solution_to_multi_axis_target(
        self,
        solution: FiveAxisSolution,
        *,
        velocity_overrides: Mapping[AxisName, float] | None = None,
        acceleration_overrides: Mapping[AxisName, float] | None = None,
    ) -> MultiAxisTarget:
        """生成统一轴空间目标；``MultiAxisTarget`` 不表达笛卡尔 frame。

        frame 只属于已经完成的 IK 输入。输出固定使用 Slide/Z 的 mm 和三个
        旋转轴的 deg，不包含 Base 偏移、startup position 或原始电机单位。
        """

        if not isinstance(solution, FiveAxisSolution):
            raise TypeError("solution must be FiveAxisSolution")
        velocities = dict(velocity_overrides or {})
        accelerations = dict(acceleration_overrides or {})
        _validate_override_axes("velocity_overrides", velocities)
        _validate_override_axes("acceleration_overrides", accelerations)
        positions = {
            AxisName.SLIDE: solution.slide_mm,
            AxisName.Z: solution.z_mm,
            AxisName.SHOULDER: solution.shoulder_deg,
            AxisName.ELBOW: solution.elbow_deg,
            AxisName.ROTATION: solution.rotation_deg,
        }
        return MultiAxisTarget(
            tuple(
                AxisTarget(
                    axis,
                    positions[axis],
                    velocities.get(axis),
                    accelerations.get(axis),
                )
                for axis in _AXIS_ORDER
            )
        )

    def _validate_inverse_geometry(self) -> None:
        geometry = self.five_axis_kinematics.geometry
        mount_rotation = geometry.slide_zero_T_planar_origin_at_zero.rotation_matrix
        z_in_mount = mount_rotation.T @ np.asarray(geometry.z_direction_xyz)
        if abs(float(z_in_mount[2])) <= self.config.linear_solve_tolerance:
            raise BaseFrameSolverError(
                "z_direction_xyz has no component normal to the configured planar frame"
            )

    def _rotation_output_target(
        self,
        slide_zero_T_tool_target: RigidTransform,
    ) -> tuple[RigidTransform, float]:
        geometry = self.five_axis_kinematics.geometry
        output_target = (
            slide_zero_T_tool_target @ geometry.rotation_output_T_tool.inverse()
        )
        relative_rotation = (
            geometry.slide_zero_T_planar_origin_at_zero.rotation_matrix.T
            @ output_target.rotation_matrix
        )
        relative_matrix = np.eye(4, dtype=float)
        relative_matrix[:3, :3] = relative_rotation
        relative = RigidTransform(relative_matrix)
        roll_deg, pitch_deg, yaw_deg = (
            float(value) for value in relative.rpy_deg
        )
        if max(abs(roll_deg), abs(pitch_deg)) > (
            self.config.model_roll_pitch_tolerance_deg
        ):
            raise BaseFrameSolverError(
                "target roll/pitch is incompatible with the yaw-only Rotation output "
                f"model: roll={roll_deg:.9f} deg pitch={pitch_deg:.9f} deg"
            )
        return output_target, yaw_deg

    def _slide_candidates(
        self,
        current_state: RobotAxisState,
        fixed_slide_mm: float | None,
    ) -> tuple[float, ...]:
        minimum, maximum = self._limits(AxisName.SLIDE)
        if fixed_slide_mm is not None:
            fixed = _require_finite("fixed_slide_mm", fixed_slide_mm)
            if not minimum <= fixed <= maximum:
                raise FiveAxisNoSolutionError(
                    f"fixed Slide {fixed} mm is outside [{minimum}, {maximum}] mm",
                    stage="slide_limits",
                    stage_counts={"slide_candidates": 0},
                )
            return (fixed,)
        values = [minimum, maximum]
        step = self.config.slide_search_step_mm
        count = int(math.floor((maximum - minimum) / step))
        values.extend(minimum + index * step for index in range(count + 1))
        if minimum <= current_state.slide_mm <= maximum:
            values.append(float(current_state.slide_mm))
        unique: list[float] = []
        for value in values:
            if not any(math.isclose(value, item, rel_tol=0.0, abs_tol=1e-9) for item in unique):
                unique.append(float(value))
        unique.sort(key=lambda value: (abs(value - current_state.slide_mm), value))
        return tuple(unique)

    def _solve_z_and_planar_point(
        self,
        output_target: RigidTransform,
        slide_mm: float,
    ) -> tuple[float, float, float]:
        geometry = self.five_axis_kinematics.geometry
        mount = geometry.slide_zero_T_planar_origin_at_zero
        mount_rotation = mount.rotation_matrix
        residual_slide_zero = (
            output_target.translation_mm
            - np.asarray(geometry.slide_direction_xyz) * slide_mm
            - mount.translation_mm
        )
        residual_mount = mount_rotation.T @ residual_slide_zero
        z_direction_mount = (
            mount_rotation.T @ np.asarray(geometry.z_direction_xyz)
        )
        z_mm = float(residual_mount[2] / z_direction_mount[2])
        planar = residual_mount - z_direction_mount * z_mm
        return z_mm, float(planar[0]), float(planar[1])

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
        values = [value for value in values if minimum - 1e-9 <= value <= maximum + 1e-9]
        values.sort(key=lambda value: (abs(value - current_rotation_deg), value))
        return tuple(values)

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
            + weights.shoulder
            * changes[AxisName.SHOULDER]
            / self._axis_span(AxisName.SHOULDER)
            + weights.elbow
            * changes[AxisName.ELBOW]
            / self._axis_span(AxisName.ELBOW)
            + weights.rotation
            * changes[AxisName.ROTATION]
            / self._axis_span(AxisName.ROTATION)
        )
        margin_penalty = 0.0
        for axis, margin in margins:
            normalized_margin = min(1.0, 2.0 * margin / self._axis_span(axis))
            margin_penalty += 1.0 - max(0.0, normalized_margin)
        return score + weights.limit_margin * margin_penalty

    def _limit_margins(
        self,
        state: RobotAxisState,
    ) -> tuple[tuple[AxisName, float], ...]:
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
        return minimum - 1e-9 <= value <= maximum + 1e-9


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


def _failure_stage(counts: Mapping[str, int]) -> str:
    for stage in (
        "z_within_limits",
        "planar_solutions",
        "joint_limits",
        "rotation_limits",
        "fk_verified",
    ):
        if counts.get(stage, 0) == 0:
            return stage
    return "candidate_selection"


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

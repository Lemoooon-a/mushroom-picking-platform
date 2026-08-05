from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np

from config.project.workspace_planning import (
    OffsetWorkspaceSide,
    SlideSelectionReason,
)
from geometry.rigid_transform import RigidTransform, angular_difference_deg
from kinematics.base_frame_solver import (
    BaseFrameFiveAxisSolver,
    BaseFrameSolverError,
    FiveAxisNoSolutionError,
    UnvalidatedBaseTransformError,
)
from kinematics.five_axis import FiveAxisGeometry, FiveAxisKinematics
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
)


def geometry(*, tool_z_mm: float = 0.0) -> FiveAxisGeometry:
    return FiveAxisGeometry(
        link1_length_mm=300.0,
        link2_length_mm=300.0,
        slide_direction_xyz=(0.0, 1.0, 0.0),
        z_direction_xyz=(0.0, 0.0, 1.0),
        slide_zero_T_planar_origin_at_zero=RigidTransform.identity(),
        rotation_output_T_tool=RigidTransform.from_xyz_yaw_deg(
            x_mm=0.0,
            y_mm=0.0,
            z_mm=tool_z_mm,
            yaw_deg=0.0,
        ),
    )


def descriptors(
    *,
    slide: tuple[float, float] = (0.0, 800.0),
    z: tuple[float, float] = (-190.0, 0.0),
    shoulder: tuple[float, float] = (-65.0, 65.0),
    elbow: tuple[float, float] = (-160.0, 160.0),
    rotation: tuple[float, float] = (-180.0, 180.0),
) -> dict[AxisName, AxisDescriptor]:
    limits = {
        AxisName.SLIDE: slide,
        AxisName.Z: z,
        AxisName.SHOULDER: shoulder,
        AxisName.ELBOW: elbow,
        AxisName.ROTATION: rotation,
    }
    result: dict[AxisName, AxisDescriptor] = {}
    for axis in AxisName:
        linear = axis in (AxisName.SLIDE, AxisName.Z)
        result[axis] = AxisDescriptor(
            axis,
            axis.value,
            AxisKind.LINEAR if linear else AxisKind.ROTARY,
            "mm" if linear else "deg",
            "mm/s" if linear else "deg/s",
            "mm/s²" if linear else "deg/s²",
            *limits[axis],
            AxisCapabilities(True, True, True, linear, True, linear, True),
        )
    return result


def solver(
    *,
    model: FiveAxisKinematics | None = None,
    base: RigidTransform | None = None,
    limits: dict[AxisName, AxisDescriptor] | None = None,
    validated: bool = True,
    allow_unvalidated: bool = False,
) -> BaseFrameFiveAxisSolver:
    return BaseFrameFiveAxisSolver(
        five_axis_kinematics=model or FiveAxisKinematics(geometry()),
        base_T_slide_zero=base or RigidTransform.identity(),
        axis_descriptors=limits or descriptors(),
        base_transform_validated=validated,
        allow_unvalidated_base_transform=allow_unvalidated,
    )


def state_for_local_point(
    model: FiveAxisKinematics,
    *,
    local_x_mm: float,
    local_y_mm: float,
    slide_mm: float = 0.0,
    z_mm: float = -80.0,
    output_yaw_deg: float = 0.0,
    branch: int = 0,
) -> RobotAxisState:
    joint = model.planar_2r.inverse(local_x_mm, local_y_mm)[branch]
    shoulder = math.degrees(joint.shoulder_rad)
    elbow = math.degrees(joint.elbow_rad)
    return RobotAxisState(
        slide_mm,
        z_mm,
        shoulder,
        elbow,
        output_yaw_deg - shoulder - elbow,
    )


def target_for_state(
    model: FiveAxisKinematics,
    state: RobotAxisState,
    base: RigidTransform | None = None,
) -> RigidTransform:
    return (base or RigidTransform.identity()) @ model.forward_kinematics(state)


class BaseFrameTransformAndGateTests(unittest.TestCase):
    def test_base_target_conversion_round_trip(self) -> None:
        base = RigidTransform.from_xyz_yaw_deg(
            x_mm=10,
            y_mm=20,
            z_mm=30,
            yaw_deg=-35,
        )
        target_slide = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=250,
            z_mm=-80,
            yaw_deg=20,
        )
        subject = solver(base=base)
        converted = subject.transform_base_target_to_slide_zero(base @ target_slide)
        np.testing.assert_allclose(converted.matrix, target_slide.matrix, atol=1e-10)

    def test_unvalidated_transform_is_rejected_by_default(self) -> None:
        with self.assertRaisesRegex(UnvalidatedBaseTransformError, "provisional"):
            solver(validated=False)
        self.assertFalse(
            solver(validated=False, allow_unvalidated=True).base_transform_validated
        )

    def test_import_has_no_hardware_or_local_file_side_effect(self) -> None:
        host_root = Path(__file__).resolve().parents[1]
        command = (
            "import sys; import kinematics.base_frame_solver; "
            "assert 'drivers.can_bus' not in sys.modules; "
            "assert 'config.local.hardware' not in sys.modules; "
            "assert 'config.local.motion' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", command],
            cwd=host_root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_incompatible_roll_pitch_is_rejected(self) -> None:
        with self.assertRaisesRegex(BaseFrameSolverError, "roll/pitch"):
            solver().solve_base_target(
                base_T_tool_target=RigidTransform.from_xyz_rpy_deg(
                    x_mm=400,
                    y_mm=250,
                    z_mm=-80,
                    roll_deg=2,
                    pitch_deg=0,
                    yaw_deg=0,
                ),
                current_state=RobotAxisState(0, -80, 0, 0, 0),
            )


class OffsetWorkspaceSolverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model = FiveAxisKinematics(geometry())

    def test_current_slide_is_immediately_kept_in_positive_and_negative_regions(self) -> None:
        for local_y, side in (
            (200.0, OffsetWorkspaceSide.POSITIVE),
            (-200.0, OffsetWorkspaceSide.NEGATIVE),
        ):
            with self.subTest(side=side):
                current = state_for_local_point(
                    self.model,
                    local_x_mm=400.0,
                    local_y_mm=local_y,
                    slide_mm=120.0,
                )
                result = solver(model=self.model).solve_base_target(
                    base_T_tool_target=target_for_state(self.model, current),
                    current_state=current,
                )
                self.assertAlmostEqual(result.slide_mm, current.slide_mm)
                self.assertIs(
                    result.slide_selection_reason,
                    SlideSelectionReason.KEEP_CURRENT_SLIDE,
                )
                self.assertIs(result.workspace_side, side)

    def test_current_slide_returns_both_elbow_branches_and_prefers_small_change(self) -> None:
        current = state_for_local_point(
            self.model,
            local_x_mm=450.0,
            local_y_mm=200.0,
            branch=1,
        )
        candidates = solver(
            model=self.model,
            limits=descriptors(rotation=(-360.0, 360.0)),
        ).solve_base_target_candidates(
            base_T_tool_target=target_for_state(self.model, current),
            current_state=current,
        )
        self.assertEqual(
            {candidate.elbow_branch for candidate in candidates},
            {"elbow-positive", "elbow-negative"},
        )
        self.assertAlmostEqual(candidates[0].elbow_deg, current.elbow_deg)

    def test_valid_current_slide_does_not_generate_center_or_fallback_candidates(self) -> None:
        current = state_for_local_point(
            self.model,
            local_x_mm=400.0,
            local_y_mm=200.0,
            slide_mm=120.0,
        )
        subject = solver(model=self.model)
        reasons: list[SlideSelectionReason] = []
        original = subject._solutions_for_slide

        def observed(*args: object, **kwargs: object) -> list[object]:
            reasons.append(args[4])
            return original(*args, **kwargs)

        with patch.object(subject, "_solutions_for_slide", side_effect=observed):
            result = subject.solve_base_target(
                base_T_tool_target=target_for_state(self.model, current),
                current_state=current,
            )
        self.assertIs(
            result.slide_selection_reason,
            SlideSelectionReason.KEEP_CURRENT_SLIDE,
        )
        self.assertEqual(reasons, [SlideSelectionReason.KEEP_CURRENT_SLIDE])

    def test_center_candidates_are_used_only_when_current_slide_is_invalid(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=500,
            z_mm=-80,
            yaw_deg=0,
        )
        current = RobotAxisState(0, -80, 0, 0, 0)
        result = solver(model=self.model).solve_base_target(
            base_T_tool_target=target,
            current_state=current,
        )
        self.assertIs(
            result.slide_selection_reason,
            SlideSelectionReason.POSITIVE_OFFSET_CENTER,
        )
        self.assertAlmostEqual(result.slide_mm, 250.0)
        self.assertAlmostEqual(result.local_y_mm, 250.0)

    def test_center_internal_scoring_prefers_less_slide_motion(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=500,
            z_mm=-80,
            yaw_deg=0,
        )
        current = RobotAxisState(600, -80, 0, 0, 0)
        result = solver(model=self.model).solve_base_target(
            base_T_tool_target=target,
            current_state=current,
        )
        self.assertIs(
            result.slide_selection_reason,
            SlideSelectionReason.NEGATIVE_OFFSET_CENTER,
        )
        self.assertAlmostEqual(result.slide_mm, 750.0)

    def test_fallback_is_used_after_both_centers_fail_slide_limits(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=500,
            z_mm=-80,
            yaw_deg=0,
        )
        subject = solver(
            model=self.model,
            limits=descriptors(slide=(0.0, 200.0)),
        )
        result = subject.solve_base_target(
            base_T_tool_target=target,
            current_state=RobotAxisState(0, -80, 0, 0, 0),
        )
        self.assertIs(
            result.slide_selection_reason,
            SlideSelectionReason.POSITIVE_OFFSET_FALLBACK,
        )
        self.assertGreaterEqual(result.local_y_mm, 300.0)
        self.assertLessEqual(result.slide_mm, 200.0)

    def test_negative_fallback_reaches_closed_workspace_boundary(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=160,
            z_mm=-80,
            yaw_deg=0,
        )
        subject = solver(
            model=self.model,
            limits=descriptors(slide=(300.0, 400.0)),
        )
        result = subject.solve_base_target(
            base_T_tool_target=target,
            current_state=RobotAxisState(300, -80, 0, 0, 0),
        )
        self.assertIs(
            result.slide_selection_reason,
            SlideSelectionReason.NEGATIVE_OFFSET_FALLBACK,
        )
        self.assertAlmostEqual(result.local_y_mm, -150.0)
        self.assertAlmostEqual(result.slide_mm, 310.0)

    def test_planar_and_joint_limit_rejections_are_structured(self) -> None:
        cases = (
            (
                FiveAxisKinematics(
                    FiveAxisGeometry(
                        150,
                        150,
                        (0, 1, 0),
                        (0, 0, 1),
                        RigidTransform.identity(),
                        RigidTransform.identity(),
                    )
                ),
                descriptors(),
                "planar_unreachable",
            ),
            (self.model, descriptors(shoulder=(-0.01, 0.01)), "shoulder_limit"),
            (
                self.model,
                descriptors(shoulder=(-180.0, 180.0), elbow=(-1.0, 1.0)),
                "elbow_limit",
            ),
        )
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=250,
            z_mm=-80,
            yaw_deg=0,
        )
        for candidate_model, limits, expected_stage in cases:
            with self.subTest(stage=expected_stage), self.assertRaises(
                FiveAxisNoSolutionError
            ) as raised:
                solver(model=candidate_model, limits=limits).solve_base_target(
                    base_T_tool_target=target,
                    current_state=RobotAxisState(0, -80, 0, 0, 0),
                )
            self.assertEqual(raised.exception.stage, expected_stage)

    def test_fk_translation_and_yaw_residual_rejections_are_structured(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=250,
            z_mm=-80,
            yaw_deg=0,
        )
        original = self.model.forward_kinematics

        def shifted_translation(state: RobotAxisState) -> RigidTransform:
            matrix = original(state).matrix.copy()
            matrix[0, 3] += 0.01
            return RigidTransform(matrix)

        def shifted_yaw(state: RobotAxisState) -> RigidTransform:
            return original(state) @ RigidTransform.from_xyz_yaw_deg(
                x_mm=0,
                y_mm=0,
                z_mm=0,
                yaw_deg=0.01,
            )

        for replacement, expected_stage in (
            (shifted_translation, "fk_translation_residual"),
            (shifted_yaw, "fk_yaw_residual"),
        ):
            with self.subTest(stage=expected_stage), patch.object(
                self.model,
                "forward_kinematics",
                side_effect=replacement,
            ), self.assertRaises(FiveAxisNoSolutionError) as raised:
                solver(
                    model=self.model,
                    limits=descriptors(
                        shoulder=(-180.0, 180.0),
                        elbow=(-180.0, 180.0),
                        rotation=(-540.0, 540.0),
                    ),
                ).solve_base_target(
                    base_T_tool_target=target,
                    current_state=RobotAxisState(0, -80, 0, 0, 0),
                )
            self.assertEqual(raised.exception.stage, expected_stage)

    def test_selection_is_deterministic(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=400,
            y_mm=500,
            z_mm=-80,
            yaw_deg=0,
        )
        subject = solver(model=self.model)
        current = RobotAxisState(400, -80, 0, 0, 0)
        first = subject.solve_base_target(
            base_T_tool_target=target,
            current_state=current,
        )
        second = subject.solve_base_target(
            base_T_tool_target=target,
            current_state=current,
        )
        self.assertEqual(first, second)

    def test_center_gap_x_outside_z_and_rotation_limits_are_diagnostic(self) -> None:
        cases = (
            (
                RigidTransform.from_xyz_yaw_deg(
                    x_mm=20, y_mm=0, z_mm=-80, yaw_deg=0
                ),
                descriptors(),
                "outside_offset_workspace",
            ),
            (
                RigidTransform.from_xyz_yaw_deg(
                    x_mm=400, y_mm=250, z_mm=10, yaw_deg=0
                ),
                descriptors(),
                "z_limit",
            ),
            (
                RigidTransform.from_xyz_yaw_deg(
                    x_mm=450, y_mm=200, z_mm=-80, yaw_deg=170
                ),
                descriptors(rotation=(-5, 5)),
                "rotation_limit",
            ),
        )
        for target, limits, stage in cases:
            with self.subTest(stage=stage), self.assertRaises(
                FiveAxisNoSolutionError
            ) as raised:
                solver(model=self.model, limits=limits).solve_base_target(
                    base_T_tool_target=target,
                    current_state=RobotAxisState(0, -80, 0, 0, 0),
                )
            self.assertEqual(raised.exception.stage, stage)

    def test_fixed_slide_must_still_satisfy_offset_workspace(self) -> None:
        with self.assertRaises(FiveAxisNoSolutionError) as raised:
            solver(model=self.model).solve_with_fixed_slide(
                base_T_tool_target=RigidTransform.from_xyz_yaw_deg(
                    x_mm=400,
                    y_mm=0,
                    z_mm=-80,
                    yaw_deg=0,
                ),
                current_state=RobotAxisState(0, -80, 0, 0, 0),
                slide_mm=0,
            )
        self.assertEqual(raised.exception.stage, "outside_offset_workspace")

    def test_solution_outputs_complete_target_and_fk_residuals(self) -> None:
        current = state_for_local_point(
            self.model,
            local_x_mm=400,
            local_y_mm=250,
        )
        subject = solver(model=self.model)
        solution = subject.solve_base_target(
            base_T_tool_target=target_for_state(self.model, current),
            current_state=current,
        )
        target = subject.solution_to_multi_axis_target(solution)
        self.assertEqual(tuple(item.axis for item in target.targets), tuple(AxisName))
        self.assertLessEqual(solution.position_residual_mm, 1e-6)
        self.assertLessEqual(solution.yaw_residual_deg, 1e-6)


class FrozenJointLimitTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model = FiveAxisKinematics(geometry())
        self.subject = solver(model=self.model)

    def _constrained(self, shoulder: float, elbow: float) -> None:
        state = RobotAxisState(0.0, -80.0, shoulder, elbow, -shoulder - elbow)
        self.subject.constrained_solution(
            base_T_tool_target=target_for_state(self.model, state),
            axis_state=state,
            slide_selection_reason=SlideSelectionReason.KEEP_CURRENT_SLIDE,
            elbow_branch="test",
            allow_outside_workspace=True,
        )

    def test_shoulder_closed_boundaries_and_both_outside_directions(self) -> None:
        self._constrained(-65.0, 0.0)
        self._constrained(65.0, 0.0)
        for value in (-65.001, 65.001):
            with self.subTest(value=value), self.assertRaises(
                FiveAxisNoSolutionError
            ) as raised:
                self._constrained(value, 0.0)
            self.assertEqual(raised.exception.stage, "shoulder_limit")

    def test_elbow_closed_boundaries_and_both_outside_directions(self) -> None:
        self._constrained(0.0, -160.0)
        self._constrained(0.0, 160.0)
        for value in (-160.001, 160.001):
            with self.subTest(value=value), self.assertRaises(
                FiveAxisNoSolutionError
            ) as raised:
                self._constrained(0.0, value)
            self.assertEqual(raised.exception.stage, "elbow_limit")


class ZSignAndToolOffsetTests(unittest.TestCase):
    def test_lower_base_target_produces_more_negative_z(self) -> None:
        model = FiveAxisKinematics(geometry(tool_z_mm=-240.0))
        subject = solver(model=model)
        current = state_for_local_point(
            model,
            local_x_mm=400,
            local_y_mm=250,
            z_mm=-20,
        )
        high = target_for_state(model, current)
        low = RigidTransform.from_xyz_yaw_deg(
            x_mm=high.translation_mm[0],
            y_mm=high.translation_mm[1],
            z_mm=high.translation_mm[2] - 100.0,
            yaw_deg=high.yaw_deg,
        )
        high_solution = subject.solve_base_target(
            base_T_tool_target=high,
            current_state=current,
        )
        low_solution = subject.solve_base_target(
            base_T_tool_target=low,
            current_state=current,
        )
        self.assertLess(low_solution.z_mm, high_solution.z_mm)
        self.assertAlmostEqual(low_solution.z_mm, high_solution.z_mm - 100.0)

    def test_wrapped_yaw_reconstructs(self) -> None:
        model = FiveAxisKinematics(geometry())
        subject = solver(
            model=model,
            limits=descriptors(rotation=(-540.0, 540.0)),
        )
        for yaw in (0.0, 179.0, -179.0, 181.0, -181.0):
            with self.subTest(yaw=yaw):
                target = RigidTransform.from_xyz_yaw_deg(
                    x_mm=400,
                    y_mm=250,
                    z_mm=-80,
                    yaw_deg=yaw,
                )
                solution = subject.solve_with_fixed_slide(
                    base_T_tool_target=target,
                    current_state=RobotAxisState(0, -80, 0, 0, yaw),
                    slide_mm=0,
                )
                reconstructed = model.forward_kinematics(solution.axis_state())
                self.assertAlmostEqual(
                    angular_difference_deg(reconstructed.yaw_deg, target.yaw_deg),
                    0.0,
                    places=7,
                )


if __name__ == "__main__":
    unittest.main()

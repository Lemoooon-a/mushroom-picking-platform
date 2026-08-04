from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from geometry.rigid_transform import RigidTransform, angular_difference_deg
from kinematics.base_frame_solver import (
    BaseFrameFiveAxisSolver,
    BaseFrameSolverConfig,
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


def geometry(
    *,
    mount: RigidTransform | None = None,
    tool: RigidTransform | None = None,
) -> FiveAxisGeometry:
    return FiveAxisGeometry(
        link1_length_mm=100.0,
        link2_length_mm=100.0,
        slide_direction_xyz=(0.0, 1.0, 0.0),
        z_direction_xyz=(0.0, 0.0, 1.0),
        slide_zero_T_planar_origin_at_zero=mount or RigidTransform.identity(),
        rotation_output_T_tool=tool or RigidTransform.identity(),
    )


def descriptors(
    *,
    slide: tuple[float, float] = (0.0, 200.0),
    z: tuple[float, float] = (0.0, 200.0),
    shoulder: tuple[float, float] = (-180.0, 180.0),
    elbow: tuple[float, float] = (-180.0, 180.0),
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
            name=axis,
            display_name=axis.value,
            kind=AxisKind.LINEAR if linear else AxisKind.ROTARY,
            position_unit="mm" if linear else "deg",
            velocity_unit="mm/s" if linear else "deg/s",
            acceleration_unit="mm/s²" if linear else "deg/s²",
            minimum_position=limits[axis][0],
            maximum_position=limits[axis][1],
            capabilities=AxisCapabilities(True, True, True, linear, True, linear, True),
        )
    return result


def solver(
    *,
    model: FiveAxisKinematics | None = None,
    base: RigidTransform | None = None,
    limits: dict[AxisName, AxisDescriptor] | None = None,
    validated: bool = True,
    allow_unvalidated: bool = False,
    step: float = 5.0,
) -> BaseFrameFiveAxisSolver:
    return BaseFrameFiveAxisSolver(
        five_axis_kinematics=model or FiveAxisKinematics(geometry()),
        base_T_slide_zero=base or RigidTransform.identity(),
        axis_descriptors=limits or descriptors(),
        base_transform_validated=validated,
        allow_unvalidated_base_transform=allow_unvalidated,
        config=BaseFrameSolverConfig(slide_search_step_mm=step),
    )


def assert_pose_close(
    case: unittest.TestCase,
    actual: RigidTransform,
    expected: RigidTransform,
) -> None:
    np.testing.assert_allclose(
        actual.translation_mm,
        expected.translation_mm,
        atol=1e-7,
    )
    case.assertAlmostEqual(
        angular_difference_deg(actual.yaw_deg, expected.yaw_deg),
        0.0,
        places=7,
    )


class BaseFrameTransformAndGateTests(unittest.TestCase):
    def test_identity_translation_yaw_and_combined_base_transforms(self) -> None:
        target_slide = RigidTransform.from_xyz_yaw_deg(
            x_mm=40, y_mm=50, z_mm=60, yaw_deg=70
        )
        bases = (
            RigidTransform.identity(),
            RigidTransform.from_xyz_yaw_deg(x_mm=10, y_mm=20, z_mm=30, yaw_deg=0),
            RigidTransform.from_xyz_yaw_deg(x_mm=0, y_mm=0, z_mm=0, yaw_deg=90),
            RigidTransform.from_xyz_yaw_deg(x_mm=10, y_mm=20, z_mm=30, yaw_deg=-35),
        )
        for base in bases:
            with self.subTest(base=base):
                subject = solver(base=base)
                base_target = base @ target_slide
                converted = subject.transform_base_target_to_slide_zero(base_target)
                np.testing.assert_allclose(converted.matrix, target_slide.matrix, atol=1e-10)
                np.testing.assert_allclose((base @ converted).matrix, base_target.matrix, atol=1e-10)

    def test_unvalidated_transform_is_rejected_by_default_and_explicitly_allowed(self) -> None:
        with self.assertRaisesRegex(UnvalidatedBaseTransformError, "provisional"):
            solver(validated=False)
        allowed = solver(validated=False, allow_unvalidated=True)
        self.assertFalse(allowed.base_transform_validated)

    def test_solver_has_no_startup_position_dependency(self) -> None:
        subject = solver()
        self.assertFalse(hasattr(subject, "startup_position"))
        self.assertFalse(hasattr(subject.five_axis_kinematics.geometry, "startup_position"))

    def test_import_has_no_hardware_or_local_file_side_effect(self) -> None:
        host_root = Path(__file__).resolve().parents[1]
        command = (
            "import sys; import kinematics.base_frame_solver; "
            "assert 'drivers.can_bus' not in sys.modules; "
            "assert 'config.hardware_local' not in sys.modules; "
            "assert 'config.motion_local' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", command],
            cwd=host_root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


class FiveAxisRoundTripTests(unittest.TestCase):
    def test_fk_base_ik_fk_round_trip_for_multiple_working_points(self) -> None:
        model = FiveAxisKinematics(
            geometry(
                mount=RigidTransform.from_xyz_yaw_deg(
                    x_mm=10, y_mm=20, z_mm=30, yaw_deg=12
                ),
                tool=RigidTransform.from_xyz_yaw_deg(
                    x_mm=15, y_mm=-4, z_mm=-30, yaw_deg=8
                ),
            )
        )
        base = RigidTransform.from_xyz_yaw_deg(
            x_mm=-40, y_mm=75, z_mm=100, yaw_deg=-23
        )
        subject = solver(model=model, base=base, step=2.0)
        cases = (
            RobotAxisState(0, 20, 20, 60, -35),
            RobotAxisState(40, 75, -30, 90, 140),
            RobotAxisState(120, 5, 50, -100, -70),
        )
        for known in cases:
            with self.subTest(known=known):
                base_target = base @ model.forward_kinematics(known)
                solution = subject.solve_base_target(
                    base_T_tool_target=base_target,
                    current_state=known,
                )
                reconstructed = base @ model.forward_kinematics(solution.axis_state())
                assert_pose_close(self, reconstructed, base_target)
                for axis, margin in solution.limit_margins:
                    self.assertGreaterEqual(margin, -1e-8, axis.value)

    def test_tcp_z_offset_is_inverted_through_geometry(self) -> None:
        model = FiveAxisKinematics(
            geometry(
                tool=RigidTransform.from_xyz_yaw_deg(
                    x_mm=0, y_mm=0, z_mm=-30, yaw_deg=0
                )
            )
        )
        known = RobotAxisState(0, 50, 0, 90, -90)
        target = model.forward_kinematics(known)
        solution = solver(model=model).solve_with_fixed_slide(
            base_T_tool_target=target,
            current_state=known,
            slide_mm=0,
        )
        self.assertAlmostEqual(solution.z_mm, 50.0)
        self.assertAlmostEqual(target.translation_mm[2], 20.0)

    def test_incompatible_roll_pitch_is_rejected_instead_of_ignored(self) -> None:
        target = RigidTransform.from_xyz_rpy_deg(
            x_mm=100,
            y_mm=0,
            z_mm=20,
            roll_deg=2,
            pitch_deg=0,
            yaw_deg=0,
        )
        with self.assertRaisesRegex(BaseFrameSolverError, "roll/pitch"):
            solver().solve_base_target(
                base_T_tool_target=target,
                current_state=RobotAxisState(0, 20, 0, 0, 0),
            )


class RedundancyBranchAndLimitTests(unittest.TestCase):
    def test_current_slide_is_kept_when_reachable(self) -> None:
        model = FiveAxisKinematics(geometry())
        known = RobotAxisState(60, 20, 10, 80, -30)
        target = model.forward_kinematics(known)
        result = solver(model=model).solve_base_target(
            base_T_tool_target=target,
            current_state=known,
        )
        self.assertAlmostEqual(result.slide_mm, 60)
        self.assertEqual(result.slide_selection_reason, "current")

    def test_nearest_other_slide_is_used_when_current_is_unreachable(self) -> None:
        model = FiveAxisKinematics(geometry())
        target = model.forward_kinematics(RobotAxisState(150, 20, 0, 82.819244, 0))
        current = RobotAxisState(0, 20, 0, 82.819244, 0)
        result = solver(model=model, step=10).solve_base_target(
            base_T_tool_target=target,
            current_state=current,
        )
        self.assertGreater(result.slide_mm, 0)
        self.assertEqual(result.slide_selection_reason, "nearest-discrete-candidate")

    def test_fixed_slide_and_slide_limit(self) -> None:
        model = FiveAxisKinematics(geometry())
        known = RobotAxisState(80, 20, 20, 60, -10)
        target = model.forward_kinematics(known)
        subject = solver(model=model)
        result = subject.solve_with_fixed_slide(
            base_T_tool_target=target,
            current_state=known,
            slide_mm=80,
        )
        self.assertAlmostEqual(result.slide_mm, 80)
        self.assertEqual(result.slide_selection_reason, "fixed")
        with self.assertRaisesRegex(FiveAxisNoSolutionError, "outside"):
            subject.solve_with_fixed_slide(
                base_T_tool_target=target,
                current_state=known,
                slide_mm=201,
            )

    def test_slide_soft_limit_endpoint_and_whole_range_unreachable(self) -> None:
        model = FiveAxisKinematics(geometry())
        known = RobotAxisState(200, 20, 0, 90, -90)
        target = model.forward_kinematics(known)
        endpoint = solver(model=model).solve_with_fixed_slide(
            base_T_tool_target=target,
            current_state=known,
            slide_mm=200,
        )
        self.assertAlmostEqual(endpoint.slide_mm, 200)
        self.assertEqual(dict(endpoint.limit_margins)[AxisName.SLIDE], 0)

        with self.assertRaises(FiveAxisNoSolutionError) as raised:
            solver(model=model, step=5).solve_base_target(
                base_T_tool_target=RigidTransform.from_xyz_yaw_deg(
                    x_mm=500, y_mm=500, z_mm=20, yaw_deg=0
                ),
                current_state=RobotAxisState(100, 20, 0, 0, 0),
            )
        self.assertEqual(raised.exception.stage, "planar_solutions")

    def test_candidate_order_uses_nearest_slide_then_stable_score(self) -> None:
        model = FiveAxisKinematics(geometry())
        target = model.forward_kinematics(RobotAxisState(180, 20, 0, 90, -90))
        current = RobotAxisState(0, 20, 0, 90, -90)
        candidates = solver(model=model, step=10).solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=current,
        )
        slide_deltas = [abs(item.slide_mm - current.slide_mm) for item in candidates]
        self.assertEqual(slide_deltas[0], min(slide_deltas))
        self.assertEqual(
            candidates,
            solver(model=model, step=10).solve_base_target_candidates(
                base_T_tool_target=target,
                current_state=current,
            ),
        )

    def test_positive_and_negative_elbow_branches_are_both_returned(self) -> None:
        model = FiveAxisKinematics(geometry())
        known = RobotAxisState(20, 20, 0, 90, -20)
        target = model.forward_kinematics(known)
        candidates = solver(model=model).solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=known,
            fixed_slide_mm=20,
        )
        self.assertEqual(
            {candidate.branch for candidate in candidates},
            {"elbow-positive", "elbow-negative"},
        )

    def test_elbow_limits_select_only_one_branch_and_both_limits_can_reject(self) -> None:
        model = FiveAxisKinematics(geometry())
        known = RobotAxisState(20, 20, 0, 90, 0)
        target = model.forward_kinematics(known)
        positive_only = solver(
            model=model,
            limits=descriptors(elbow=(0, 180)),
        ).solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=known,
            fixed_slide_mm=20,
        )
        self.assertEqual({item.branch for item in positive_only}, {"elbow-positive"})
        negative_only = solver(
            model=model,
            limits=descriptors(elbow=(-180, 0)),
        ).solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=RobotAxisState(20, 20, 90, -90, 0),
            fixed_slide_mm=20,
        )
        self.assertEqual({item.branch for item in negative_only}, {"elbow-negative"})
        with self.assertRaises(FiveAxisNoSolutionError) as raised:
            solver(
                model=model,
                limits=descriptors(elbow=(-10, 10)),
            ).solve_base_target(
                base_T_tool_target=target,
                current_state=known,
            )
        self.assertEqual(raised.exception.stage, "joint_limits")

    def test_outer_singularity_is_deterministic_and_unreachable_target_fails(self) -> None:
        subject = solver(step=10)
        singular_target = RigidTransform.from_xyz_yaw_deg(
            x_mm=200, y_mm=20, z_mm=20, yaw_deg=0
        )
        first = subject.solve_base_target_candidates(
            base_T_tool_target=singular_target,
            current_state=RobotAxisState(20, 20, 0, 0, 0),
            fixed_slide_mm=20,
        )
        second = subject.solve_base_target_candidates(
            base_T_tool_target=singular_target,
            current_state=RobotAxisState(20, 20, 0, 0, 0),
            fixed_slide_mm=20,
        )
        self.assertEqual(first, second)
        self.assertEqual(first[0].branch, "singular")
        with self.assertRaises(FiveAxisNoSolutionError):
            subject.solve_base_target(
                base_T_tool_target=RigidTransform.from_xyz_yaw_deg(
                    x_mm=500, y_mm=500, z_mm=20, yaw_deg=0
                ),
                current_state=RobotAxisState(0, 20, 0, 0, 0),
            )

    def test_completely_folded_singularity_is_deterministic(self) -> None:
        subject = solver(limits=descriptors(rotation=(-360, 360)))
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=0, y_mm=0, z_mm=20, yaw_deg=0
        )
        current = RobotAxisState(0, 20, 0, 180, -180)
        first = subject.solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=current,
            fixed_slide_mm=0,
        )
        second = subject.solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=current,
            fixed_slide_mm=0,
        )
        self.assertEqual(first, second)
        self.assertEqual(first[0].branch, "singular")
        self.assertAlmostEqual(abs(first[0].elbow_deg), 180)

    def test_z_limit_rejection_is_diagnostic(self) -> None:
        subject = solver(limits=descriptors(z=(0, 10)))
        with self.assertRaises(FiveAxisNoSolutionError) as raised:
            subject.solve_base_target(
                base_T_tool_target=RigidTransform.from_xyz_yaw_deg(
                    x_mm=100, y_mm=0, z_mm=50, yaw_deg=0
                ),
                current_state=RobotAxisState(0, 0, 0, 0, 0),
            )
        self.assertEqual(raised.exception.stage, "z_within_limits")


class RotationAndOutputTests(unittest.TestCase):
    def test_zero_positive_negative_and_wrapped_yaw_reconstruct(self) -> None:
        subject = solver(limits=descriptors(rotation=(-540, 540)))
        for yaw_deg in (0, 70, -70, 179, -179, 181, -181):
            with self.subTest(yaw_deg=yaw_deg):
                target = RigidTransform.from_xyz_yaw_deg(
                    x_mm=100, y_mm=100, z_mm=20, yaw_deg=yaw_deg
                )
                result = subject.solve_with_fixed_slide(
                    base_T_tool_target=target,
                    current_state=RobotAxisState(0, 20, 0, 90, yaw_deg - 90),
                    slide_mm=0,
                )
                reconstructed = subject.five_axis_kinematics.forward_kinematics(
                    result.axis_state()
                )
                self.assertAlmostEqual(
                    angular_difference_deg(reconstructed.yaw_deg, target.yaw_deg),
                    0,
                    places=7,
                )

    def test_rotation_compensates_for_both_shoulder_elbow_branches(self) -> None:
        subject = solver(limits=descriptors(rotation=(-540, 540)))
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=100, z_mm=20, yaw_deg=35
        )
        candidates = subject.solve_base_target_candidates(
            base_T_tool_target=target,
            current_state=RobotAxisState(0, 20, 0, 90, -55),
            fixed_slide_mm=0,
        )
        by_branch = {item.branch: item for item in candidates}
        self.assertIn("elbow-positive", by_branch)
        self.assertIn("elbow-negative", by_branch)
        for item in by_branch.values():
            self.assertAlmostEqual(
                angular_difference_deg(
                    item.shoulder_deg + item.elbow_deg + item.rotation_deg,
                    35,
                ),
                0,
                places=7,
            )

    def test_rotation_periodic_equivalent_nearest_current_is_selected(self) -> None:
        limits = descriptors(rotation=(-540, 540))
        subject = solver(limits=limits)
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=200, y_mm=0, z_mm=20, yaw_deg=0
        )
        result = subject.solve_with_fixed_slide(
            base_T_tool_target=target,
            current_state=RobotAxisState(0, 20, 0, 0, 350),
            slide_mm=0,
        )
        self.assertAlmostEqual(result.rotation_deg, 360)
        self.assertAlmostEqual(result.yaw_residual_deg, 0)

    def test_rotation_limit_can_reject_all_periodic_equivalents(self) -> None:
        limits = descriptors(rotation=(-10, 10))
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=100, z_mm=20, yaw_deg=170
        )
        with self.assertRaises(FiveAxisNoSolutionError) as raised:
            solver(limits=limits).solve_with_fixed_slide(
                base_T_tool_target=target,
                current_state=RobotAxisState(0, 20, 0, 90, 0),
                slide_mm=0,
            )
        self.assertEqual(raised.exception.stage, "rotation_limits")

    def test_solution_to_multi_axis_target_uses_mm_deg_and_no_frame(self) -> None:
        subject = solver()
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=100, z_mm=20, yaw_deg=30
        )
        solution = subject.solve_with_fixed_slide(
            base_T_tool_target=target,
            current_state=RobotAxisState(0, 20, 0, 90, -60),
            slide_mm=0,
        )
        multi = subject.solution_to_multi_axis_target(solution)
        self.assertEqual(tuple(item.axis for item in multi.targets), tuple(AxisName))
        self.assertFalse(hasattr(multi, "frame_id"))
        self.assertFalse(hasattr(multi, "base_offset"))
        self.assertFalse(hasattr(multi, "startup_position"))
        self.assertTrue(all(item.velocity is None for item in multi.targets))
        self.assertTrue(all(item.acceleration is None for item in multi.targets))


if __name__ == "__main__":
    unittest.main()

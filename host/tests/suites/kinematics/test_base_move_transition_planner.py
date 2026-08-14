from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from config.project.robot_motion_envelope import (
    RobotMotionEnvelopeConfig,
    SideSwitchClearanceConfig,
    WORKING_HEIGHT_BASE_Z_MM,
)
from config.project.workspace_planning import OffsetWorkspaceSide, SlideSelectionReason
from geometry.rigid_transform import RigidTransform, angular_difference_deg
from kinematics.base_frame_solver import BaseFrameFiveAxisSolver, FiveAxisNoSolutionError
from kinematics.base_move_transition_planner import (
    BaseMoveStageKind,
    BaseMoveTransitionPlanner,
    ClearanceHeightUnreachableError,
    CurrentStateInvalidError,
    StageValidationFailedError,
)
from kinematics.five_axis import FiveAxisGeometry, FiveAxisKinematics
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
)


def model(tcp_height_at_z_zero_mm: float = 300.0) -> FiveAxisKinematics:
    return FiveAxisKinematics(
        FiveAxisGeometry(
            300.0,
            300.0,
            tcp_height_at_z_zero_mm,
        )
    )


def descriptors(
    *,
    z: tuple[float, float] = (-500.0, 0.0),
) -> dict[AxisName, AxisDescriptor]:
    limits = {
        AxisName.SLIDE: (0.0, 800.0),
        AxisName.Z: z,
        AxisName.SHOULDER: (-65.0, 65.0),
        AxisName.ELBOW: (-160.0, 160.0),
        AxisName.ROTATION: (-180.0, 180.0),
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


def planner(
    *,
    z: tuple[float, float] = (-500.0, 0.0),
    base_z_mm: float = 300.0,
    clearance_base_z_mm: float = 150.0,
) -> BaseMoveTransitionPlanner:
    solver = BaseFrameFiveAxisSolver(
        five_axis_kinematics=model(base_z_mm),
        axis_descriptors=descriptors(z=z),
    )
    return BaseMoveTransitionPlanner(
        solver,
        motion_envelope=RobotMotionEnvelopeConfig(
            side_switch=SideSwitchClearanceConfig(clearance_base_z_mm)
        ),
    )


def state_for_point(
    kinematics: FiveAxisKinematics,
    x: float,
    y: float,
    z: float,
    *,
    slide: float = 0.0,
    branch: int | None = None,
) -> RobotAxisState:
    joints = kinematics.planar_2r.inverse(x, y)
    if branch is None:
        joint = next(
            candidate
            for candidate in joints
            if -65.0 <= math.degrees(candidate.shoulder_rad) <= 65.0
            and -160.0 <= math.degrees(candidate.elbow_rad) <= 160.0
        )
    else:
        joint = joints[branch]
    shoulder = math.degrees(joint.shoulder_rad)
    elbow = math.degrees(joint.elbow_rad)
    return RobotAxisState(slide, z, shoulder, elbow, -shoulder - elbow)


class BaseMoveTransitionPlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.subject = planner()
        self.model = self.subject.solver.five_axis_kinematics

    def _target(self, state: RobotAxisState) -> RigidTransform:
        return self.subject.solver.forward_kinematics_base(state)

    def test_same_positive_and_negative_side_are_single_direct(self) -> None:
        for y in (250.0, -250.0):
            with self.subTest(y=y):
                current = state_for_point(self.model, 400, y, -300)
                target_state = state_for_point(self.model, 450, y, -350)
                plan = self.subject.plan(
                    current_state=current,
                    base_T_tool_target=self._target(target_state),
                )
                self.assertEqual(
                    tuple(stage.kind for stage in plan.stages),
                    (BaseMoveStageKind.DIRECT,),
                )
                self.assertFalse(plan.requires_side_switch_clearance)
                self.assertIsNone(plan.clearance_base_z_mm)
                self.assertIs(
                    plan.stages[0].solution.slide_selection_reason,
                    SlideSelectionReason.KEEP_CURRENT_SLIDE,
                )

    def test_positive_to_negative_and_reverse_use_three_fixed_stages(self) -> None:
        for current_y, target_y in ((250.0, -250.0), (-250.0, 250.0)):
            with self.subTest(current_y=current_y, target_y=target_y):
                current = state_for_point(self.model, 400, current_y, -300)
                target_state = state_for_point(self.model, 450, target_y, -350)
                plan = self.subject.plan(
                    current_state=current,
                    base_T_tool_target=self._target(target_state),
                )
                self.assertEqual(
                    tuple(stage.kind for stage in plan.stages),
                    (
                        BaseMoveStageKind.LIFT,
                        BaseMoveStageKind.TRANSIT,
                        BaseMoveStageKind.LOWER,
                    ),
                )
                lift, transit, lower = (stage.solution for stage in plan.stages)
                for field in ("slide_mm", "shoulder_deg", "elbow_deg", "rotation_deg"):
                    self.assertAlmostEqual(getattr(lift, field), getattr(current, field))
                for field in ("slide_mm", "shoulder_deg", "elbow_deg", "rotation_deg"):
                    self.assertAlmostEqual(getattr(transit, field), getattr(lower, field))
                self.assertAlmostEqual(lift.z_mm, transit.z_mm)
                self.assertNotAlmostEqual(lower.z_mm, transit.z_mm)
                self.assertTrue(plan.requires_side_switch_clearance)
                lift_target, transit_target, lower_target = (
                    stage.base_T_tool_target for stage in plan.stages
                )
                requested = self._target(target_state)
                for index in (0, 1):
                    self.assertAlmostEqual(
                        lift_target.translation_mm[index],
                        plan.current_base_T_tool.translation_mm[index],
                    )
                    self.assertAlmostEqual(
                        transit_target.translation_mm[index],
                        requested.translation_mm[index],
                    )
                self.assertAlmostEqual(
                    lift_target.yaw_deg,
                    plan.current_base_T_tool.yaw_deg,
                )
                self.assertAlmostEqual(transit_target.yaw_deg, requested.yaw_deg)
                self.assertAlmostEqual(
                    lift_target.translation_mm[2],
                    plan.clearance_base_z_mm,
                )
                self.assertAlmostEqual(
                    transit_target.translation_mm[2],
                    plan.clearance_base_z_mm,
                )
                self.assertTrue(
                    (abs(lower_target.matrix - requested.matrix) < 1e-9).all()
                )

    def test_clearance_is_absolute_base_z_floor_of_150_mm(self) -> None:
        cases = ((-250.0, -350.0), (-350.0, -250.0), (-300.0, -300.0))
        for current_z, target_z in cases:
            with self.subTest(current_z=current_z, target_z=target_z):
                current = state_for_point(self.model, 400, 250, current_z)
                target = state_for_point(self.model, 400, -250, target_z)
                plan = self.subject.plan(
                    current_state=current,
                    base_T_tool_target=self._target(target),
                )
                self.assertAlmostEqual(
                    plan.clearance_base_z_mm,
                    max(
                        plan.current_base_T_tool.translation_mm[2],
                        self._target(target).translation_mm[2],
                        150.0,
                    ),
                )
                self.assertAlmostEqual(
                    plan.clearance_lift_mm,
                    plan.clearance_base_z_mm
                    - plan.current_base_T_tool.translation_mm[2],
                )
                self.assertGreater(plan.stages[0].solution.z_mm, current.z_mm)

    def test_clearance_uses_injected_motion_envelope(self) -> None:
        subject = planner(clearance_base_z_mm=120.0)
        current = state_for_point(subject.solver.five_axis_kinematics, 400, 250, -250)
        target = state_for_point(subject.solver.five_axis_kinematics, 400, -250, -350)
        plan = subject.plan(
            current_state=current,
            base_T_tool_target=subject.solver.forward_kinematics_base(target),
        )
        self.assertEqual(plan.clearance_base_z_mm, 120.0)

    def test_clearance_floor_at_z_upper_limit_is_allowed_and_above_is_rejected(self) -> None:
        allowed_subject = planner(base_z_mm=150.0)
        allowed_model = allowed_subject.solver.five_axis_kinematics
        current = state_for_point(allowed_model, 400, 250, 0)
        target = state_for_point(allowed_model, 400, -250, -100)
        allowed = allowed_subject.plan(
            current_state=current,
            base_T_tool_target=allowed_subject.solver.forward_kinematics_base(target),
        )
        self.assertAlmostEqual(allowed.stages[0].solution.z_mm, 0.0)
        self.assertAlmostEqual(allowed.clearance_base_z_mm, 150.0)
        self.assertAlmostEqual(allowed.clearance_lift_mm, 0.0)

        rejected_subject = planner(base_z_mm=149.0)
        rejected_model = rejected_subject.solver.five_axis_kinematics
        too_low_current = state_for_point(rejected_model, 400, 250, 0)
        rejected_target = state_for_point(rejected_model, 400, -250, -100)
        with self.assertRaises(ClearanceHeightUnreachableError) as raised:
            rejected_subject.plan(
                current_state=too_low_current,
                base_T_tool_target=rejected_subject.solver.forward_kinematics_base(
                    rejected_target
                ),
            )
        self.assertAlmostEqual(
            raised.exception.required_clearance_base_z_mm,
            150.0,
        )
        self.assertEqual(raised.exception.z_logical_limit, (-500.0, 0.0))

    def test_current_tcp_above_150_mm_switches_without_additional_lift(self) -> None:
        current = state_for_point(self.model, 400, 250, -120)
        target = state_for_point(self.model, 400, -250, -200)
        plan = self.subject.plan(
            current_state=current,
            base_T_tool_target=self._target(target),
        )
        self.assertAlmostEqual(
            plan.current_base_T_tool.translation_mm[2],
            180.0,
        )
        self.assertAlmostEqual(plan.clearance_base_z_mm, 180.0)
        self.assertAlmostEqual(plan.clearance_lift_mm, 0.0)
        self.assertAlmostEqual(plan.stages[0].solution.z_mm, current.z_mm)
        self.assertEqual(
            tuple(stage.kind for stage in plan.stages),
            (
                BaseMoveStageKind.TRANSIT,
                BaseMoveStageKind.LOWER,
            ),
        )

    def test_side_switch_at_shared_working_height_has_no_z_stage(self) -> None:
        current = state_for_point(self.model, 400, 250, -150)
        target = state_for_point(self.model, 400, -250, -150)
        plan = self.subject.plan(
            current_state=current,
            base_T_tool_target=self._target(target),
        )

        self.assertAlmostEqual(
            plan.current_base_T_tool.translation_mm[2],
            WORKING_HEIGHT_BASE_Z_MM,
        )
        self.assertAlmostEqual(plan.clearance_base_z_mm, WORKING_HEIGHT_BASE_Z_MM)
        self.assertAlmostEqual(plan.clearance_lift_mm, 0.0)
        self.assertEqual(
            tuple(stage.kind for stage in plan.stages),
            (BaseMoveStageKind.TRANSIT,),
        )

    def test_outside_calibration_pose_at_top_can_transit_without_lift(self) -> None:
        current = state_for_point(self.model, 400, 0, -120)
        target = state_for_point(self.model, 400, 250, -200)
        plan = self.subject.plan(
            current_state=current,
            base_T_tool_target=self._target(target),
        )
        self.assertIs(plan.current_workspace_side, OffsetWorkspaceSide.OUTSIDE)
        self.assertIs(plan.target_workspace_side, OffsetWorkspaceSide.POSITIVE)
        self.assertAlmostEqual(plan.current_base_T_tool.translation_mm[2], 180.0)
        self.assertAlmostEqual(plan.clearance_base_z_mm, 180.0)
        self.assertAlmostEqual(plan.clearance_lift_mm, 0.0)
        self.assertAlmostEqual(plan.stages[0].solution.z_mm, current.z_mm)

    def test_outside_current_with_planar_change_uses_conservative_three_stages(self) -> None:
        for target_y, expected_side in (
            (250, OffsetWorkspaceSide.POSITIVE),
            (-250, OffsetWorkspaceSide.NEGATIVE),
        ):
            with self.subTest(target_y=target_y):
                current = state_for_point(self.model, 400, 0, -300)
                target = state_for_point(self.model, 450, target_y, -350)
                plan = self.subject.plan(
                    current_state=current,
                    base_T_tool_target=self._target(target),
                )
                self.assertIs(plan.current_workspace_side, OffsetWorkspaceSide.OUTSIDE)
                self.assertEqual(len(plan.stages), 3)
                self.assertIs(
                    plan.stages[0].solution.workspace_side,
                    OffsetWorkspaceSide.OUTSIDE,
                )
                self.assertIs(plan.stages[1].solution.workspace_side, expected_side)

    def test_outside_current_allows_only_proven_pure_z_direct(self) -> None:
        current = state_for_point(self.model, 400, 0, -300)
        target = RobotAxisState(
            current.slide_mm,
            -350,
            current.shoulder_deg,
            current.elbow_deg,
            current.rotation_deg,
        )
        plan = self.subject.plan(
            current_state=current,
            base_T_tool_target=self._target(target),
        )
        self.assertEqual(
            tuple(stage.kind for stage in plan.stages),
            (BaseMoveStageKind.DIRECT,),
        )
        self.assertIs(plan.target_workspace_side, OffsetWorkspaceSide.OUTSIDE)
        solution = plan.stages[0].solution
        for field in ("slide_mm", "shoulder_deg", "elbow_deg", "rotation_deg"):
            self.assertAlmostEqual(getattr(solution, field), getattr(current, field))
        self.assertLess(solution.z_mm, current.z_mm)

    def test_invalid_current_axis_state_is_rejected_before_target_plan(self) -> None:
        current = state_for_point(self.model, 400, 250, 1.0)
        target = state_for_point(self.model, 400, -250, -300)
        with self.assertRaises(CurrentStateInvalidError):
            self.subject.plan(
                current_state=current,
                base_T_tool_target=self._target(target),
            )

    def test_every_stage_fk_reconstructs_its_base_target(self) -> None:
        current = state_for_point(self.model, 400, 250, -300)
        target = state_for_point(self.model, 450, -200, -350)
        plan = self.subject.plan(
            current_state=current,
            base_T_tool_target=self._target(target),
        )
        for stage in plan.stages:
            reconstructed = self.subject.solver.forward_kinematics_base(
                stage.solution.axis_state()
            )
            np_xyz = reconstructed.translation_mm - stage.base_T_tool_target.translation_mm
            self.assertLess(math.sqrt(float(np_xyz @ np_xyz)), 1e-6)
            self.assertLess(
                abs(
                    angular_difference_deg(
                        reconstructed.yaw_deg,
                        stage.base_T_tool_target.yaw_deg,
                    )
                ),
                1e-6,
            )

    def test_stage_validation_failure_rejects_the_whole_plan(self) -> None:
        current = state_for_point(self.model, 400, 250, -300)
        target = state_for_point(self.model, 450, -250, -350)
        original = self.subject.solver.constrained_solution
        call_count = 0

        def fail_transit(*args: object, **kwargs: object) -> object:
            nonlocal call_count
            call_count += 1
            if call_count == 3:
                raise FiveAxisNoSolutionError(
                    "injected transit FK rejection",
                    stage="fk_translation_residual",
                    stage_counts={"fk_translation_residual": 1},
                )
            return original(*args, **kwargs)

        with patch.object(
            self.subject.solver,
            "constrained_solution",
            side_effect=fail_transit,
        ), self.assertRaises(StageValidationFailedError) as raised:
            self.subject.plan(
                current_state=current,
                base_T_tool_target=self._target(target),
            )
        self.assertIn("fk_translation_residual", str(raised.exception))


if __name__ == "__main__":
    unittest.main()

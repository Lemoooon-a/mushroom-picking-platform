"""统一人工运动入口的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from config.frame_transforms import FixedFrameTransforms, FrameTransformsDocument
from geometry.rigid_transform import RigidTransform
from kinematics.base_frame_solver import (
    FiveAxisNoSolutionError,
    UnvalidatedBaseTransformError,
)
from kinematics.base_move_transition_planner import (
    BaseMovePlanningError,
    ClearanceHeightUnreachableError,
    CurrentStateInvalidError,
)
from kinematics.five_axis import FiveAxisGeometry, FiveAxisKinematics
from kinematics.frame_chain import RobotAxisState
from motion.authorization import RuntimeMode
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisTarget,
    MotionCommandStatus,
    MultiAxisTarget,
)
from scripts.manual_motion import (
    main,
    run_plan_base,
    run_home,
    run_inspect,
    run_move,
    run_move_group,
    run_state,
    run_stop,
)
from tests.helpers.motion_cli_test_support import (
    axis_state,
    command_result,
    fake_runtime,
    group_result,
)


class ManualMotionTests(unittest.TestCase):
    @staticmethod
    def _plan_model() -> FiveAxisKinematics:
        return FiveAxisKinematics(
            FiveAxisGeometry(
                link1_length_mm=300,
                link2_length_mm=300,
                slide_direction_xyz=(0, 1, 0),
                z_direction_xyz=(0, 0, 1),
                slide_zero_T_planar_origin_at_zero=RigidTransform.identity(),
                rotation_output_T_tool=RigidTransform.identity(),
            )
        )

    @staticmethod
    def _plan_state(
        model: FiveAxisKinematics,
        x_mm: float,
        y_mm: float,
        z_mm: float,
    ) -> tuple[float, float, float, float, float]:
        joint = next(
            candidate
            for candidate in model.planar_2r.inverse(x_mm, y_mm)
            if -65.0 <= math.degrees(candidate.shoulder_rad) <= 65.0
            and -160.0 <= math.degrees(candidate.elbow_rad) <= 160.0
        )
        shoulder = math.degrees(joint.shoulder_rad)
        elbow = math.degrees(joint.elbow_rad)
        return (0.0, z_mm, shoulder, elbow, -shoulder - elbow)

    @staticmethod
    def _plan_runtime(
        state: tuple[float, float, float, float, float],
    ) -> object:
        runtime = fake_runtime()
        limits = {
            AxisName.SLIDE: (0.0, 800.0),
            AxisName.Z: (-500.0, 0.0),
            AxisName.SHOULDER: (-65.0, 65.0),
            AxisName.ELBOW: (-160.0, 160.0),
            AxisName.ROTATION: (-180.0, 180.0),
        }
        descriptors = []
        states = []
        for axis, position in zip(AxisName, state, strict=True):
            linear = axis in (AxisName.SLIDE, AxisName.Z)
            descriptors.append(
                AxisDescriptor(
                    axis,
                    axis.value,
                    AxisKind.LINEAR if linear else AxisKind.ROTARY,
                    "mm" if linear else "deg",
                    "mm/s" if linear else "deg/s",
                    "mm/s²" if linear else "deg/s²",
                    *limits[axis],
                    AxisCapabilities(True, True, True, linear, True, linear, True),
                )
            )
            states.append(axis_state(axis, position=position))
        runtime.controller.list_axes.return_value = tuple(descriptors)
        runtime.controller.get_axis_states.side_effect = None
        runtime.controller.get_axis_states.return_value = tuple(states)
        return runtime

    @staticmethod
    def _frame_document(
        *,
        validated: bool,
        base_z_mm: float = 300.0,
    ) -> FrameTransformsDocument:
        return FrameTransformsDocument(
            FixedFrameTransforms(
                RigidTransform.from_xyz_yaw_deg(
                    x_mm=0,
                    y_mm=0,
                    z_mm=base_z_mm,
                    yaw_deg=0,
                ),
                None,
            ),
            {"validated": validated},
        )

    @staticmethod
    def _plan_target(
        model: FiveAxisKinematics,
        state: tuple[float, float, float, float, float],
        *,
        base_z_mm: float = 300.0,
    ) -> RigidTransform:
        return RigidTransform.from_xyz_yaw_deg(
            x_mm=0,
            y_mm=0,
            z_mm=base_z_mm,
            yaw_deg=0,
        ) @ model.forward_kinematics(RobotAxisState(*state))

    def test_inspect_and_state_are_read_only(self) -> None:
        runtime = fake_runtime()
        run_inspect(runtime, emit=lambda _line: None)
        runtime.shoulder_joint.initialize.assert_called_once()
        runtime.elbow_joint.initialize.assert_called_once()
        runtime.controller.get_axis_states.assert_called_once_with(tuple(AxisName))
        runtime.controller.submit_absolute.assert_not_called()

        runtime = fake_runtime()
        run_state(runtime, AxisName.SHOULDER, emit=lambda _line: None)
        runtime.shoulder_joint.initialize.assert_called_once()
        runtime.controller.get_state.assert_called_once_with(AxisName.SHOULDER)

    def test_move_preview_and_execute_use_unified_controller_once(self) -> None:
        target = AxisTarget(AxisName.SHOULDER, 20.0, 2.0)
        preview = fake_runtime()
        self.assertTrue(run_move(preview, target, execute=False, timeout_s=5.0, emit=lambda _line: None))
        preview.controller.submit_absolute.assert_not_called()

        runtime = fake_runtime()
        self.assertTrue(run_move(runtime, target, execute=True, timeout_s=5.0, emit=lambda _line: None))
        runtime.controller.submit_absolute.assert_called_once_with(target)
        runtime.controller.wait.assert_called_once()
        runtime.rotation_axis.disable_torque.assert_not_called()

    def test_plan_base_is_read_only_and_prints_complete_five_axis_preview(self) -> None:
        model = self._plan_model()
        current = self._plan_state(model, 300, 250, -300)
        runtime = self._plan_runtime(current)
        output: list[str] = []
        target = self._plan_target(
            model,
            self._plan_state(model, 350, -250, -350),
        )
        self.assertTrue(
            run_plan_base(
                runtime,
                target,
                frame_document=self._frame_document(validated=True),
                five_axis_kinematics=model,
                emit=output.append,
            )
        )
        runtime.shoulder_joint.initialize.assert_called_once()
        runtime.elbow_joint.initialize.assert_called_once()
        runtime.controller.get_axis_states.assert_called_once_with(tuple(AxisName))
        self.assertEqual(runtime.controller.validate_positions.call_count, 3)
        runtime.controller.submit_absolute.assert_not_called()
        runtime.controller.submit_positions.assert_not_called()
        runtime.controller.wait.assert_not_called()
        runtime.controller.wait_group.assert_not_called()
        runtime.controller.home_reference.assert_not_called()
        runtime.controller.stop.assert_not_called()
        runtime.rotation_axis.enable_torque.assert_not_called()
        rendered = "\n".join(output)
        for expected in (
            "Current Base TCP pose:",
            "Current workspace side: POSITIVE",
            "Target workspace side: NEGATIVE",
            "Slide selection reason: KEEP_CURRENT_SLIDE",
            "Clearance lift: 150.000000000 mm",
            "Stage count: 3",
            "Stage 1: LIFT",
            "Stage 2: TRANSIT",
            "Stage 3: LOWER",
            "slide=",
            "translation_residual=",
            "Planning only. No real motion command was issued.",
        ):
            self.assertIn(expected, rendered)

    def test_plan_base_same_side_prints_one_direct_stage(self) -> None:
        model = self._plan_model()
        current = self._plan_state(model, 300, 250, -300)
        runtime = self._plan_runtime(current)
        target = self._plan_target(
            model,
            self._plan_state(model, 350, 250, -350),
        )
        output: list[str] = []
        self.assertTrue(
            run_plan_base(
                runtime,
                target,
                frame_document=self._frame_document(validated=True),
                five_axis_kinematics=model,
                emit=output.append,
            )
        )
        rendered = "\n".join(output)
        self.assertIn("Stage count: 1", rendered)
        self.assertIn("Stage 1: DIRECT", rendered)
        self.assertIn("Target workspace side: POSITIVE", rendered)
        runtime.controller.validate_positions.assert_called_once()
        runtime.controller.submit_positions.assert_not_called()

    def test_plan_base_clearance_and_current_state_failures_are_explicit(self) -> None:
        model = self._plan_model()
        target = self._plan_target(
            model,
            self._plan_state(model, 350, -250, -200),
            base_z_mm=149.0,
        )
        with self.assertRaises(ClearanceHeightUnreachableError):
            run_plan_base(
                self._plan_runtime(self._plan_state(model, 300, 250, -100)),
                target,
                frame_document=self._frame_document(validated=True, base_z_mm=149.0),
                five_axis_kinematics=model,
                emit=lambda _line: None,
            )
        with self.assertRaises(CurrentStateInvalidError):
            run_plan_base(
                self._plan_runtime(self._plan_state(model, 300, 250, 1)),
                target,
                frame_document=self._frame_document(validated=True),
                five_axis_kinematics=model,
                emit=lambda _line: None,
            )

    def test_plan_base_unvalidated_transform_is_always_rejected(self) -> None:
        model = self._plan_model()
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=300, y_mm=250, z_mm=-300, yaw_deg=0
        )
        runtime = self._plan_runtime(self._plan_state(model, 300, 250, -300))
        with self.assertRaisesRegex(ValueError, "provisional"):
            run_plan_base(
                runtime,
                target,
                frame_document=self._frame_document(validated=False),
                five_axis_kinematics=model,
                emit=lambda _line: None,
            )
        runtime.controller.submit_positions.assert_not_called()
        runtime.__enter__.assert_not_called()

    def test_plan_base_missing_or_corrupt_frame_config_is_explicit(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=100, z_mm=0, yaw_deg=0
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame_transforms.json"
            with self.assertRaisesRegex(ValueError, "cannot read"):
                run_plan_base(
                    fake_runtime(),
                    target,
                    frame_config=path,
                    five_axis_kinematics=self._plan_model(),
                    emit=lambda _line: None,
                )
            path.write_text("{not-json", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "invalid JSON"):
                run_plan_base(
                    fake_runtime(),
                    target,
                    frame_config=path,
                    five_axis_kinematics=self._plan_model(),
                    emit=lambda _line: None,
                )

    def test_plan_base_does_not_modify_provisional_frame_config(self) -> None:
        target = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=100, z_mm=0, yaw_deg=0
        )
        payload = """{
  "schema_version": 1,
  "base_T_slide_zero": {
    "translation_mm": [0, 0, 0],
    "rotation_rpy_deg": [0, 0, 0]
  },
  "tool_T_camera": null,
  "metadata": {"validated": false}
}\n"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame_transforms.json"
            path.write_text(payload, encoding="utf-8")
            before = path.read_bytes()
            with self.assertRaisesRegex(ValueError, "provisional"):
                run_plan_base(
                    fake_runtime(),
                    target,
                    frame_config=path,
                    five_axis_kinematics=self._plan_model(),
                    emit=lambda _line: None,
                )
            self.assertEqual(path.read_bytes(), before)

    @patch("scripts.manual_motion.create_configured_runtime")
    def test_plan_base_parser_has_no_execute_and_uses_read_only_runtime(self, create) -> None:
        base_argv = [
            "plan-base",
            "--tcp-x-mm", "100",
            "--tcp-y-mm", "100",
            "--tcp-z-mm", "0",
            "--tcp-yaw-deg", "0",
        ]
        for removed_option in (
            ("--execute",),
            ("--slide-mm", "0"),
            ("--allow-unvalidated-frame-transform",),
        ):
            with self.subTest(option=removed_option), redirect_stderr(
                io.StringIO()
            ), self.assertRaises(SystemExit):
                main(base_argv + list(removed_option))
        create.assert_not_called()

        with patch("scripts.manual_motion.run_plan_base", return_value=True) as plan:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(main(base_argv), 0)
        create.assert_called_once_with(
            RuntimeMode.READ_ONLY,
            allow_unverified_rotation_motion=False,
        )
        plan.assert_called_once()

    @patch("scripts.manual_motion.run_plan_base")
    @patch("scripts.manual_motion.create_configured_runtime")
    def test_plan_base_no_solution_has_distinct_exit_code(self, create, plan) -> None:
        plan.side_effect = FiveAxisNoSolutionError(
            "unreachable",
            stage="planar_solutions",
            stage_counts={"planar_solutions": 0},
        )
        argv = [
            "plan-base",
            "--tcp-x-mm", "999",
            "--tcp-y-mm", "999",
            "--tcp-z-mm", "0",
            "--tcp-yaw-deg", "0",
        ]
        error = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(error):
            self.assertEqual(main(argv), 1)
        self.assertIn("stage=planar_solutions", error.getvalue())
        create.return_value.controller.submit_positions.assert_not_called()

    @patch("scripts.manual_motion.run_plan_base")
    @patch("scripts.manual_motion.create_configured_runtime")
    def test_plan_base_transition_failure_has_distinct_exit_code(self, create, plan) -> None:
        plan.side_effect = BaseMovePlanningError("clearance stage rejected")
        argv = [
            "plan-base",
            "--tcp-x-mm", "300",
            "--tcp-y-mm", "250",
            "--tcp-z-mm", "-300",
            "--tcp-yaw-deg", "0",
        ]
        error = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(error):
            self.assertEqual(main(argv), 1)
        self.assertIn("transition planning failed", error.getvalue())
        create.assert_called_once_with(
            RuntimeMode.READ_ONLY,
            allow_unverified_rotation_motion=False,
        )

    @patch("scripts.manual_motion.run_plan_base")
    @patch("scripts.manual_motion.create_configured_runtime")
    def test_plan_base_unvalidated_calibration_has_config_exit_code(self, create, plan) -> None:
        plan.side_effect = UnvalidatedBaseTransformError("provisional calibration")
        argv = [
            "plan-base",
            "--tcp-x-mm", "300",
            "--tcp-y-mm", "250",
            "--tcp-z-mm", "-300",
            "--tcp-yaw-deg", "0",
        ]
        error = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(error):
            self.assertEqual(main(argv), 2)
        self.assertIn("calibration unavailable", error.getvalue())

    @patch("scripts.manual_motion.create_configured_runtime")
    def test_motion_and_rotation_confirmation_gates_precede_runtime(self, create) -> None:
        cases = (
            ["move", "--axis", "shoulder", "--position", "2", "--execute"],
            ["move", "--axis", "rotation", "--position", "2", "--execute", "--confirm-motion"],
            [
                "move", "--axis", "rotation", "--position", "2", "--execute",
                "--confirm-motion", "--allow-rotation-motion", "--confirm-rotation-no-stop",
            ],
            [
                "move", "--axis", "rotation", "--position", "2", "--execute",
                "--confirm-motion", "--allow-rotation-motion", "--enable-rotation-torque",
            ],
            [
                "move", "--axis", "rotation", "--position", "2", "--execute",
                "--confirm-motion", "--confirm-rotation-no-stop", "--enable-rotation-torque",
            ],
            ["move-group"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()

    def test_move_group_keeps_subset_and_terminal_failure_does_not_repeat_stop(self) -> None:
        target = MultiAxisTarget((
            AxisTarget(AxisName.SHOULDER, 20.0, 2.0),
            AxisTarget(AxisName.ELBOW, -40.0, 2.0),
        ))
        runtime = fake_runtime()
        runtime.controller.wait_group.side_effect = lambda handle, timeout_s=None: group_result(
            target, MotionCommandStatus.TIMEOUT
        )
        self.assertFalse(run_move_group(runtime, target, execute=True, timeout_s=10.0, emit=lambda _line: None))
        runtime.controller.get_axis_states.assert_called_once_with((AxisName.SHOULDER, AxisName.ELBOW))
        runtime.controller.submit_positions.assert_called_once_with(target)
        runtime.controller.wait_group.assert_called_once()
        runtime.controller.stop.assert_not_called()

    def test_move_group_interrupt_stops_each_stoppable_axis_once(self) -> None:
        target = MultiAxisTarget((
            AxisTarget(AxisName.SLIDE, 2.0),
            AxisTarget(AxisName.ELBOW, -4.0, 2.0),
            AxisTarget(AxisName.ROTATION, 1.0),
        ))
        runtime = fake_runtime()
        runtime.controller.wait_group.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_move_group(
                runtime,
                target,
                execute=True,
                timeout_s=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        self.assertEqual(
            tuple(call.args[0] for call in runtime.controller.stop.call_args_list),
            (AxisName.SLIDE, AxisName.ELBOW),
        )

    def test_home_terminal_failures_do_not_repeat_stop(self) -> None:
        for status in (MotionCommandStatus.TIMEOUT, MotionCommandStatus.FAULT, MotionCommandStatus.ABORTED):
            runtime = fake_runtime()
            runtime.controller.home_reference.side_effect = None
            runtime.controller.home_reference.return_value = command_result(AxisName.SLIDE, status, target=0.0)
            self.assertFalse(run_home(runtime, AxisName.SLIDE, execute=True, timeout_s=15.0, emit=lambda _line: None))
            runtime.controller.stop.assert_not_called()

    def test_home_preview_and_success_call_home_reference_once(self) -> None:
        preview = fake_runtime()
        self.assertTrue(
            run_home(preview, AxisName.Z, execute=False, timeout_s=60.0, emit=lambda _line: None)
        )
        preview.controller.home_reference.assert_not_called()

        runtime = fake_runtime()
        runtime.controller.home_reference.return_value = command_result(
            AxisName.SLIDE, MotionCommandStatus.ARRIVED, target=0.0
        )
        self.assertTrue(
            run_home(runtime, AxisName.SLIDE, execute=True, timeout_s=15.0, emit=lambda _line: None)
        )
        runtime.controller.home_reference.assert_called_once_with(AxisName.SLIDE, timeout_s=15.0)

    def test_stop_support_and_wording(self) -> None:
        for axis in (AxisName.SLIDE, AxisName.Z, AxisName.SHOULDER, AxisName.ELBOW):
            runtime = fake_runtime()
            output: list[str] = []
            self.assertTrue(run_stop(runtime, axis, execute=True, emit=output.append))
            runtime.controller.stop.assert_called_once_with(axis)
            rendered = "\n".join(output).lower()
            self.assertNotIn("disable", rendered)
            self.assertNotIn("emergency stop", rendered)
            self.assertNotIn("torque disable", rendered)
        runtime = fake_runtime()
        self.assertFalse(run_stop(runtime, AxisName.ROTATION, execute=True, emit=lambda _line: None))
        runtime.controller.stop.assert_not_called()

    @patch("scripts.manual_motion.run_inspect")
    @patch("scripts.manual_motion.create_configured_runtime")
    def test_inspect_main_uses_read_only_runtime(self, create, inspect) -> None:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main(["inspect"]), 0)
        create.assert_called_once_with(RuntimeMode.READ_ONLY, allow_unverified_rotation_motion=False)
        inspect.assert_called_once_with(create.return_value)


if __name__ == "__main__":
    unittest.main()

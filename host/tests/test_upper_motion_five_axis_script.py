"""统一 Runtime 五轴联动测试程序的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from motion.authorization import RuntimeMode
from motion.unified_controller import MultiAxisSubmissionError, UnifiedMotionError
from motion.unified_protocol import (
    AxisKind,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
)
from scripts.test_upper_motion_five_axis import main, run_five_axis_test


def target() -> MultiAxisTarget:
    return MultiAxisTarget(
        (
            AxisTarget(AxisName.SLIDE, 2.0),
            AxisTarget(AxisName.Z, 3.0),
            AxisTarget(AxisName.SHOULDER, 4.0),
            AxisTarget(AxisName.ELBOW, -5.0),
            AxisTarget(AxisName.ROTATION, 6.0),
        )
    )


def states(*, unhealthy_axis: AxisName | None = None) -> tuple[AxisState, ...]:
    result: list[AxisState] = []
    for axis in AxisName:
        linear = axis in (AxisName.SLIDE, AxisName.Z)
        can_axis = axis in (AxisName.SHOULDER, AxisName.ELBOW)
        unhealthy = axis is unhealthy_axis
        result.append(
            AxisState(
                axis=axis,
                connected=True,
                enabled=True if can_axis else False if linear else None,
                busy=False,
                homed=not unhealthy if linear else None,
                position_valid=not unhealthy,
                current_position=None if unhealthy else 0.0,
                position_unit="mm" if linear else "deg",
                faulted=False,
                fault_code=None,
                fault_message=None,
            )
        )
    return tuple(result)


def arrived_result(planned: MultiAxisTarget) -> MultiAxisCommandResult:
    results = tuple(
        MotionCommandResult(
            command_id=f"cmd-{item.axis.value}",
            axis=item.axis,
            status=MotionCommandStatus.ARRIVED,
            accepted=True,
            completed=True,
            target_position=item.position,
            final_position=item.position,
            position_error=0.0,
            error_code=None,
            message="arrived",
        )
        for item in planned.targets
    )
    return MultiAxisCommandResult(
        group_id="group",
        status=MotionCommandStatus.ARRIVED,
        results=results,
        accepted=True,
        completed=True,
        message="all arrived",
    )


def failed_result(planned: MultiAxisTarget) -> MultiAxisCommandResult:
    results = tuple(
        MotionCommandResult(
            command_id=f"cmd-{item.axis.value}",
            axis=item.axis,
            status=MotionCommandStatus.REJECTED,
            accepted=False,
            completed=False,
            target_position=item.position,
            final_position=None,
            position_error=None,
            error_code=MotionErrorCode.BACKEND_ERROR,
            message="not submitted",
        )
        for item in planned.targets
    )
    return MultiAxisCommandResult(
        group_id="group",
        status=MotionCommandStatus.REJECTED,
        results=results,
        accepted=False,
        completed=False,
        message="submission failed",
    )


def stopped(axis: AxisName) -> MotionCommandResult:
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
        message="software stop accepted",
    )


def fake_runtime() -> MagicMock:
    runtime = MagicMock()
    runtime.__enter__.return_value = runtime
    runtime.__exit__.return_value = None
    runtime.shoulder_joint.initialize.return_value = SimpleNamespace(position_rad=0.0)
    runtime.elbow_joint.initialize.return_value = SimpleNamespace(position_rad=0.0)
    runtime.shoulder_joint.config = SimpleNamespace(max_velocity_rad_s=1.0)
    runtime.elbow_joint.config = SimpleNamespace(max_velocity_rad_s=1.0)
    runtime.rotation_axis.config = SimpleNamespace(max_speed_raw=100)
    runtime.controller.get_axis_states.return_value = states()
    runtime.motion_config.profiles.return_value = {
        axis: SimpleNamespace(
            default_velocity=(None if axis is AxisName.ROTATION else 1.0),
            default_acceleration=(
                1.0 if axis in (AxisName.SLIDE, AxisName.Z) else None
            ),
        )
        for axis in AxisName
    }
    runtime.motion_config.linear_motion_limits.return_value = {
        AxisName.SLIDE: (72.0, 180.0),
        AxisName.Z: (10.0, 25.0),
    }

    def descriptor(axis: AxisName) -> object:
        linear = axis in (AxisName.SLIDE, AxisName.Z)
        return SimpleNamespace(
            name=axis,
            kind=AxisKind.LINEAR if linear else AxisKind.ROTARY,
            minimum_position=0.0 if linear else -180.0,
            maximum_position=100.0 if linear else 180.0,
            position_unit="mm" if linear else "deg",
            velocity_unit="mm/s" if linear else "deg/s",
            acceleration_unit="mm/s²" if linear else "deg/s²",
            capabilities=SimpleNamespace(
                query_state=True,
                move_absolute=True,
                stop=axis is not AxisName.ROTATION,
                reference_home=linear,
                configurable_velocity=axis is not AxisName.ROTATION,
                configurable_acceleration=linear,
                arrival_confirmation=True,
            ),
        )

    runtime.controller.describe_axis.side_effect = descriptor
    runtime.controller.stop.side_effect = lambda axis: stopped(axis)
    planned = target()
    runtime.controller.submit_positions.return_value = MultiAxisCommandHandle(
        "group",
        tuple(
            MotionCommandHandle(f"cmd-{item.axis.value}", item.axis, item.position)
            for item in planned.targets
        ),
    )
    runtime.controller.wait_group.return_value = arrived_result(planned)
    return runtime


class RunFiveAxisTestTests(unittest.TestCase):
    def test_read_only_preflight_reads_all_axes_without_control_writes(self) -> None:
        runtime = fake_runtime()
        output: list[str] = []

        self.assertTrue(
            run_five_axis_test(
                runtime,
                target(),
                execute=False,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                emit=output.append,
            )
        )

        runtime.controller.get_axis_states.assert_called_once_with(tuple(AxisName))
        runtime.rotation_axis.command_position.assert_not_called()
        runtime.rotation_axis.enable_torque.assert_not_called()
        runtime.controller.submit_positions.assert_not_called()
        runtime.controller.stop.assert_not_called()
        self.assertIn("READ_ONLY", "\n".join(output))
        rendered = "\n".join(output)
        self.assertIn("axis=slide", rendered)
        self.assertIn("axis=rotation", rendered)
        self.assertIn("only explicitly listed axes participate", rendered)

    def test_preflight_rejects_unhealthy_axis_before_any_control_write(self) -> None:
        runtime = fake_runtime()
        runtime.controller.get_axis_states.return_value = states(
            unhealthy_axis=AxisName.Z
        )

        self.assertFalse(
            run_five_axis_test(
                runtime,
                target(),
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        )

        runtime.rotation_axis.enable_torque.assert_not_called()
        runtime.controller.submit_positions.assert_not_called()

    def test_preflight_enforces_per_invocation_delta_limits(self) -> None:
        runtime = fake_runtime()
        planned = MultiAxisTarget(
            tuple(
                AxisTarget(item.axis, 20.0 if item.axis is AxisName.SLIDE else item.position)
                for item in target().targets
            )
        )

        self.assertFalse(
            run_five_axis_test(
                runtime,
                planned,
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.submit_positions.assert_not_called()

    def test_rotation_confirmation_precedes_torque_enable(self) -> None:
        runtime = fake_runtime()
        runtime.motion_config.profiles.return_value[AxisName.SLIDE] = SimpleNamespace(
            default_velocity=None,
            default_acceleration=1.0,
        )

        with self.assertRaises(ValueError):
            run_five_axis_test(
                runtime,
                target(),
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                emit=lambda _line: None,
            )
        runtime.rotation_axis.enable_torque.assert_not_called()
        runtime.controller.submit_positions.assert_not_called()

    def test_unified_target_validation_failure_precedes_rotation_writes(self) -> None:
        runtime = fake_runtime()
        runtime.controller.validate_positions.side_effect = UnifiedMotionError(
            MotionErrorCode.SOFT_LIMIT,
            "axis z velocity exceeds Host limit",
            axis=AxisName.Z,
        )

        self.assertFalse(
            run_five_axis_test(
                runtime,
                target(),
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        )

        runtime.rotation_axis.command_position.assert_not_called()
        runtime.rotation_axis.enable_torque.assert_not_called()
        runtime.controller.submit_positions.assert_not_called()

    def test_execute_preloads_rotation_enables_torque_then_submits_once(self) -> None:
        runtime = fake_runtime()
        runtime.controller.get_axis_states.side_effect = [states(), states()]
        planned = target()

        self.assertTrue(
            run_five_axis_test(
                runtime,
                planned,
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        )

        runtime.rotation_axis.command_position.assert_called_once_with(0.0, 100)
        runtime.rotation_axis.enable_torque.assert_called_once_with()
        runtime.controller.submit_positions.assert_called_once_with(planned)
        handle = runtime.controller.submit_positions.return_value
        runtime.controller.wait_group.assert_called_once_with(handle, timeout_s=10.0)
        runtime.controller.stop.assert_not_called()

    def test_partial_submission_failure_is_not_stopped_twice_by_script(self) -> None:
        runtime = fake_runtime()
        planned = target()
        error = MultiAxisSubmissionError(
            MotionErrorCode.BACKEND_ERROR,
            "submission failed",
            axis=AxisName.ELBOW,
            result=failed_result(planned),
        )
        runtime.controller.submit_positions.side_effect = error

        self.assertFalse(
            run_five_axis_test(
                runtime,
                planned,
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        )
        runtime.controller.stop.assert_not_called()

    def test_wait_interruption_attempts_each_available_stop_once(self) -> None:
        runtime = fake_runtime()
        runtime.controller.wait_group.side_effect = KeyboardInterrupt()

        with self.assertRaises(KeyboardInterrupt):
            run_five_axis_test(
                runtime,
                target(),
                execute=True,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )

        self.assertEqual(
            tuple(call.args[0] for call in runtime.controller.stop.call_args_list),
            (
                AxisName.SLIDE,
                AxisName.Z,
                AxisName.SHOULDER,
                AxisName.ELBOW,
            ),
        )

    def test_axis_subsets_and_order_are_supported_but_invalid_limits_fail_early(self) -> None:
        runtime = fake_runtime()
        incomplete = MultiAxisTarget((AxisTarget(AxisName.SLIDE, 1.0),))
        runtime.controller.get_axis_states.return_value = (states()[0],)
        self.assertTrue(
            run_five_axis_test(
                runtime,
                incomplete,
                execute=False,
                timeout_s=10.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
            )
        )
        runtime.controller.get_axis_states.assert_called_once_with((AxisName.SLIDE,))

        unopened_runtime = fake_runtime()
        with self.assertRaises(ValueError):
            run_five_axis_test(
                unopened_runtime,
                target(),
                execute=False,
                timeout_s=0.0,
                max_linear_delta_mm=5.0,
                max_rotary_delta_deg=10.0,
            )
        unopened_runtime.__enter__.assert_not_called()


class FiveAxisScriptMainTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base_args = [
            "--slide-mm",
            "2",
            "--z-mm",
            "3",
            "--shoulder-deg",
            "4",
            "--elbow-deg",
            "-5",
            "--rotation-deg",
            "6",
            "--timeout",
            "10",
            "--max-linear-delta-mm",
            "5",
            "--max-rotary-delta-deg",
            "10",
        ]

    @patch("scripts.debug_motion.debug_multi_axis_motion.run_multi_axis_test", return_value=True)
    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_read_only_main_uses_read_only_runtime(
        self,
        create: MagicMock,
        run: MagicMock,
    ) -> None:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main(self.base_args), 0)
        create.assert_called_once_with(
            RuntimeMode.READ_ONLY,
            allow_unverified_rotation_motion=False,
        )
        self.assertFalse(run.call_args.kwargs["execute"])

    @patch("scripts.debug_motion.debug_multi_axis_motion.run_multi_axis_test", return_value=True)
    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_execute_requires_all_confirmations_and_authorizes_rotation(
        self,
        create: MagicMock,
        run: MagicMock,
    ) -> None:
        args = self.base_args + [
            "--execute",
            "--confirm-five-axis-motion",
            "--confirm-emergency-stop-ready",
            "--accept-nonstrict-synchronization",
            "--accept-unverified-rotation-stop",
            "--enable-rotation-torque",
        ]
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main(args), 0)
        create.assert_called_once_with(
            RuntimeMode.MOTION,
            allow_unverified_rotation_motion=True,
        )
        self.assertTrue(run.call_args.kwargs["execute"])

    @patch("scripts.debug_motion.debug_multi_axis_motion.run_multi_axis_test", return_value=True)
    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_explicit_motion_parameters_are_forwarded_in_axis_targets(
        self,
        _create: MagicMock,
        run: MagicMock,
    ) -> None:
        args = self.base_args + [
            "--slide-speed-mm-s",
            "60",
            "--slide-accel-mm-s2",
            "180",
            "--z-speed-mm-s",
            "8",
            "--z-accel-mm-s2",
            "25",
            "--shoulder-speed-deg-s",
            "10",
            "--elbow-speed-deg-s",
            "11",
        ]
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main(args), 0)

        planned = run.call_args.args[1]
        self.assertEqual(
            tuple((item.velocity, item.acceleration) for item in planned.targets),
            (
                (60.0, 180.0),
                (8.0, 25.0),
                (10.0, None),
                (11.0, None),
                (None, None),
            ),
        )

    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_execute_missing_confirmation_fails_before_runtime_creation(
        self,
        create: MagicMock,
    ) -> None:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(self.base_args + ["--execute"])
        create.assert_not_called()


if __name__ == "__main__":
    unittest.main()

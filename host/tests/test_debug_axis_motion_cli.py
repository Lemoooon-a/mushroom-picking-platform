"""统一单轴 state/move/stop CLI 的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import patch

from motion.unified_controller import UnifiedMotionError
from motion.unified_protocol import (
    AxisName,
    AxisTarget,
    MotionCommandHandle,
    MotionErrorCode,
    MultiAxisTarget,
)
from scripts.debug_motion.debug_axis_motion import (
    main,
    run_axis_move,
    run_axis_state,
    run_axis_stop,
)
from tests.motion_cli_test_support import fake_runtime


class DebugAxisMotionTests(unittest.TestCase):
    def test_state_supports_every_axis_without_writes(self) -> None:
        for axis in AxisName:
            with self.subTest(axis=axis):
                runtime = fake_runtime()
                output: list[str] = []
                self.assertTrue(run_axis_state(runtime, axis, emit=output.append))
                runtime.controller.get_state.assert_called_once_with(axis)
                runtime.controller.submit_absolute.assert_not_called()
                runtime.controller.stop.assert_not_called()
                self.assertIn("capabilities=", "\n".join(output))

    def test_move_preview_validates_but_never_submits(self) -> None:
        runtime = fake_runtime()
        target = AxisTarget(AxisName.SHOULDER, 20.0, 2.0)
        self.assertTrue(
            run_axis_move(
                runtime,
                target,
                execute=False,
                timeout_s=5.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.validate_positions.assert_called_once_with(
            MultiAxisTarget((target,))
        )
        runtime.controller.submit_absolute.assert_not_called()
        runtime.rotation_axis.enable_torque.assert_not_called()

    def test_execute_submits_and_waits_once(self) -> None:
        runtime = fake_runtime()
        target = AxisTarget(AxisName.ELBOW, -30.0, 2.0)
        self.assertTrue(
            run_axis_move(
                runtime,
                target,
                execute=True,
                timeout_s=5.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.submit_absolute.assert_called_once_with(target)
        runtime.controller.wait.assert_called_once_with(
            MotionCommandHandle("single", AxisName.ELBOW, -30.0),
            timeout_s=5.0,
        )
        runtime.controller.stop.assert_not_called()

    def test_controller_rejects_unsupported_acceleration_before_submit(self) -> None:
        runtime = fake_runtime()
        runtime.controller.validate_positions.side_effect = UnifiedMotionError(
            MotionErrorCode.UNSUPPORTED_PARAMETER,
            "acceleration unsupported",
            axis=AxisName.SHOULDER,
        )
        with self.assertRaises(UnifiedMotionError):
            run_axis_move(
                runtime,
                AxisTarget(AxisName.SHOULDER, 10.0, 2.0, 1.0),
                execute=False,
                timeout_s=5.0,
                emit=lambda _line: None,
            )
        runtime.controller.submit_absolute.assert_not_called()

    def test_rotation_preparation_is_explicit_and_has_no_fake_stop(self) -> None:
        runtime = fake_runtime()
        target = AxisTarget(AxisName.ROTATION, 10.0)
        self.assertTrue(
            run_axis_move(
                runtime,
                target,
                execute=True,
                timeout_s=5.0,
                confirm_rotation_no_stop=True,
                confirm_rotation_torque_enable=True,
                emit=lambda _line: None,
            )
        )
        runtime.rotation_axis.command_position.assert_called_once()
        runtime.rotation_axis.enable_torque.assert_called_once()
        runtime.controller.stop.assert_not_called()

    def test_wait_interrupt_stops_submitted_stoppable_axis_once(self) -> None:
        runtime = fake_runtime()
        runtime.controller.wait.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_axis_move(
                runtime,
                AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                execute=True,
                timeout_s=5.0,
                emit=lambda _line: None,
            )
        runtime.controller.stop.assert_called_once_with(AxisName.SHOULDER)

    def test_stop_semantics_and_rotation_rejection(self) -> None:
        for axis in (
            AxisName.SLIDE,
            AxisName.Z,
            AxisName.SHOULDER,
            AxisName.ELBOW,
        ):
            runtime = fake_runtime()
            output: list[str] = []
            self.assertTrue(run_axis_stop(runtime, axis, execute=True, emit=output.append))
            runtime.controller.stop.assert_called_once_with(axis)
            rendered = "\n".join(output).lower()
            self.assertNotIn("power disabled", rendered)
            self.assertIn("not disable", rendered)
            self.assertNotIn("emergency stop", rendered)
        runtime = fake_runtime()
        self.assertFalse(
            run_axis_stop(
                runtime,
                AxisName.ROTATION,
                execute=True,
                emit=lambda _line: None,
            )
        )
        runtime.controller.stop.assert_not_called()

    @patch("scripts.debug_motion.debug_axis_motion.create_configured_runtime")
    def test_missing_motion_or_rotation_confirmation_fails_before_runtime(self, create) -> None:
        cases = (
            ["move", "--axis", "shoulder", "--position", "10", "--execute"],
            [
                "move", "--axis", "rotation", "--position", "10",
                "--execute", "--confirm-motion",
            ],
            ["stop", "--axis", "slide", "--execute"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()


if __name__ == "__main__":
    unittest.main()

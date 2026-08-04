"""Slide/Z reference home CLI stop 所有权的纯 mock 测试。"""

from __future__ import annotations

from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import patch

from motion.unified_protocol import AxisName, MotionCommandStatus
from scripts.debug_motion.home_linear_axis import main, run_home_test
from tests.motion_cli_test_support import axis_state, command_result, fake_runtime


class HomeLinearAxisTests(unittest.TestCase):
    def test_preview_never_homes_or_stops(self) -> None:
        runtime = fake_runtime()
        self.assertTrue(
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=False,
                timeout_s=15.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.home_reference.assert_not_called()
        runtime.controller.stop.assert_not_called()

    def test_success_calls_home_once_and_checks_final_state(self) -> None:
        runtime = fake_runtime()
        runtime.controller.get_state.side_effect = [
            axis_state(AxisName.Z),
            axis_state(AxisName.Z),
        ]
        self.assertTrue(
            run_home_test(
                runtime,
                AxisName.Z,
                execute=True,
                timeout_s=60.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.home_reference.assert_called_once_with(
            AxisName.Z,
            timeout_s=60.0,
        )
        self.assertEqual(runtime.controller.get_state.call_count, 2)
        runtime.controller.stop.assert_not_called()

    def test_terminal_timeout_fault_and_abort_do_not_repeat_stop(self) -> None:
        for status in (
            MotionCommandStatus.TIMEOUT,
            MotionCommandStatus.FAULT,
            MotionCommandStatus.ABORTED,
        ):
            with self.subTest(status=status):
                runtime = fake_runtime()
                runtime.controller.home_reference.return_value = command_result(
                    AxisName.SLIDE,
                    status,
                    target=0.0,
                )
                runtime.controller.home_reference.side_effect = None
                self.assertFalse(
                    run_home_test(
                        runtime,
                        AxisName.SLIDE,
                        execute=True,
                        timeout_s=15.0,
                        emit=lambda _line: None,
                    )
                )
                runtime.controller.stop.assert_not_called()

    def test_keyboard_interrupt_attempts_one_stop(self) -> None:
        runtime = fake_runtime()
        runtime.controller.home_reference.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=True,
                timeout_s=15.0,
                emit=lambda _line: None,
            )
        runtime.controller.stop.assert_called_once_with(AxisName.SLIDE)

    def test_stop_failure_does_not_replace_original_home_error(self) -> None:
        runtime = fake_runtime()
        runtime.controller.home_reference.side_effect = RuntimeError("home failed")
        runtime.controller.stop.side_effect = RuntimeError("stop failed")
        with self.assertRaisesRegex(RuntimeError, "home failed"):
            run_home_test(
                runtime,
                AxisName.Z,
                execute=True,
                timeout_s=60.0,
                emit=lambda _line: None,
            )

    @patch("scripts.debug_motion.home_linear_axis.create_configured_runtime")
    def test_only_slide_z_and_both_confirmation_flags_are_accepted(self, create) -> None:
        cases = (
            ["--axis", "shoulder"],
            ["--axis", "slide", "--execute"],
            ["--axis", "z", "--confirm-home-motion"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()


if __name__ == "__main__":
    unittest.main()

"""统一 Runtime 单轴回零测试程序的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import MagicMock, Mock, patch

from motion.authorization import RuntimeMode
from motion.unified_protocol import (
    AxisName,
    AxisState,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
)
from scripts.test_upper_motion_home import main, run_home_test


def axis_state(
    axis: AxisName,
    *,
    homed: bool,
    position_valid: bool,
) -> AxisState:
    return AxisState(
        axis=axis,
        connected=True,
        enabled=False,
        busy=False,
        homed=homed,
        position_valid=position_valid,
        current_position=0.0 if position_valid else None,
        position_unit="mm",
        faulted=not position_valid,
        fault_code=None if position_valid else 2,
        fault_message=None if position_valid else "position invalid",
    )


def arrived(axis: AxisName) -> MotionCommandResult:
    return MotionCommandResult(
        command_id="home",
        axis=axis,
        status=MotionCommandStatus.ARRIVED,
        accepted=True,
        completed=True,
        target_position=0.0,
        final_position=0.0,
        position_error=0.0,
        error_code=None,
        message="home arrived",
    )


def timed_out(axis: AxisName) -> MotionCommandResult:
    return MotionCommandResult(
        command_id="home",
        axis=axis,
        status=MotionCommandStatus.TIMEOUT,
        accepted=True,
        completed=False,
        target_position=0.0,
        final_position=None,
        position_error=None,
        error_code=MotionErrorCode.TIMEOUT,
        message="home timeout",
    )


def stopped(axis: AxisName) -> MotionCommandResult:
    return MotionCommandResult(
        command_id="stop",
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


def fake_runtime(axis: AxisName) -> MagicMock:
    runtime = MagicMock()
    runtime.__enter__.return_value = runtime
    runtime.__exit__.return_value = None
    runtime.controller.get_state.return_value = axis_state(
        axis,
        homed=False,
        position_valid=False,
    )
    runtime.controller.stop.return_value = stopped(axis)
    return runtime


class RunHomeTestTests(unittest.TestCase):
    def test_read_only_preflight_queries_once_and_never_controls(self) -> None:
        runtime = fake_runtime(AxisName.SLIDE)
        output: list[str] = []

        self.assertTrue(
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=False,
                timeout_s=15.0,
                emit=output.append,
            )
        )

        runtime.controller.get_state.assert_called_once_with(AxisName.SLIDE)
        runtime.controller.home_reference.assert_not_called()
        runtime.controller.stop.assert_not_called()
        runtime.__exit__.assert_called_once()
        self.assertIn("READ_ONLY", "\n".join(output))

    def test_slide_home_requires_arrived_homed_and_valid(self) -> None:
        runtime = fake_runtime(AxisName.SLIDE)
        runtime.controller.get_state.side_effect = [
            axis_state(AxisName.SLIDE, homed=False, position_valid=False),
            axis_state(AxisName.SLIDE, homed=True, position_valid=True),
        ]
        runtime.controller.home_reference.return_value = arrived(AxisName.SLIDE)

        self.assertTrue(
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=True,
                timeout_s=15.0,
                emit=lambda _line: None,
            )
        )

        runtime.controller.home_reference.assert_called_once_with(
            AxisName.SLIDE,
            timeout_s=15.0,
        )
        runtime.controller.stop.assert_not_called()

    def test_z_home_uses_same_unified_entry(self) -> None:
        runtime = fake_runtime(AxisName.Z)
        runtime.controller.get_state.side_effect = [
            axis_state(AxisName.Z, homed=False, position_valid=False),
            axis_state(AxisName.Z, homed=True, position_valid=True),
        ]
        runtime.controller.home_reference.return_value = arrived(AxisName.Z)

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

    def test_busy_or_non_position_fault_rejects_before_home(self) -> None:
        for state in (
            AxisState(
                axis=AxisName.SLIDE,
                connected=True,
                enabled=True,
                busy=True,
                homed=False,
                position_valid=False,
                current_position=None,
                position_unit="mm",
                faulted=False,
                fault_code=None,
                fault_message=None,
            ),
            AxisState(
                axis=AxisName.SLIDE,
                connected=True,
                enabled=False,
                busy=False,
                homed=False,
                position_valid=False,
                current_position=None,
                position_unit="mm",
                faulted=True,
                fault_code=3,
                fault_message="driver fault",
            ),
        ):
            with self.subTest(state=state):
                runtime = fake_runtime(AxisName.SLIDE)
                runtime.controller.get_state.return_value = state
                self.assertFalse(
                    run_home_test(
                        runtime,
                        AxisName.SLIDE,
                        execute=True,
                        timeout_s=15.0,
                        emit=lambda _line: None,
                    )
                )
                runtime.controller.home_reference.assert_not_called()
                runtime.controller.stop.assert_not_called()

    def test_non_arrived_terminal_result_is_not_stopped_twice(self) -> None:
        runtime = fake_runtime(AxisName.SLIDE)
        runtime.controller.get_state.side_effect = [
            axis_state(AxisName.SLIDE, homed=False, position_valid=False),
            axis_state(AxisName.SLIDE, homed=False, position_valid=False),
        ]
        runtime.controller.home_reference.return_value = timed_out(AxisName.SLIDE)

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

    def test_arrived_without_valid_final_state_is_not_success(self) -> None:
        runtime = fake_runtime(AxisName.Z)
        runtime.controller.get_state.side_effect = [
            axis_state(AxisName.Z, homed=False, position_valid=False),
            axis_state(AxisName.Z, homed=False, position_valid=False),
        ]
        runtime.controller.home_reference.return_value = arrived(AxisName.Z)

        self.assertFalse(
            run_home_test(
                runtime,
                AxisName.Z,
                execute=True,
                timeout_s=60.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.stop.assert_not_called()

    def test_home_exception_attempts_stop_and_preserves_error(self) -> None:
        runtime = fake_runtime(AxisName.SLIDE)
        runtime.controller.home_reference.side_effect = RuntimeError("home failed")
        with self.assertRaisesRegex(RuntimeError, "home failed"):
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=True,
                timeout_s=15.0,
                emit=lambda _line: None,
            )
        runtime.controller.stop.assert_called_once_with(AxisName.SLIDE)
        runtime.__exit__.assert_called_once()

    def test_keyboard_interrupt_attempts_stop_and_closes_runtime(self) -> None:
        runtime = fake_runtime(AxisName.Z)
        runtime.controller.home_reference.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_home_test(
                runtime,
                AxisName.Z,
                execute=True,
                timeout_s=60.0,
                emit=lambda _line: None,
            )
        runtime.controller.stop.assert_called_once_with(AxisName.Z)
        runtime.__exit__.assert_called_once()

    def test_invalid_axis_and_timeout_fail_before_open(self) -> None:
        runtime = fake_runtime(AxisName.SLIDE)
        with self.assertRaises(ValueError):
            run_home_test(
                runtime,
                AxisName.SHOULDER,
                execute=False,
                timeout_s=15.0,
            )
        with self.assertRaises(ValueError):
            run_home_test(
                runtime,
                AxisName.SLIDE,
                execute=False,
                timeout_s=0.0,
            )
        runtime.__enter__.assert_not_called()


class HomeScriptMainTests(unittest.TestCase):
    @patch("scripts.debug_motion.home_linear_axis.run_home_test", return_value=True)
    @patch("scripts.debug_motion.home_linear_axis.create_configured_runtime")
    def test_default_is_read_only_slide_preflight(
        self,
        create_runtime: Mock,
        run_test: Mock,
    ) -> None:
        runtime = MagicMock()
        create_runtime.return_value = runtime
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--axis", "slide"]), 0)
        create_runtime.assert_called_once_with(RuntimeMode.READ_ONLY)
        run_test.assert_called_once_with(
            runtime,
            AxisName.SLIDE,
            execute=False,
            timeout_s=15.0,
        )

    @patch("scripts.debug_motion.home_linear_axis.run_home_test", return_value=True)
    @patch("scripts.debug_motion.home_linear_axis.create_configured_runtime")
    def test_execute_z_requires_both_flags_and_uses_motion_mode(
        self,
        create_runtime: Mock,
        run_test: Mock,
    ) -> None:
        runtime = MagicMock()
        create_runtime.return_value = runtime
        with redirect_stdout(io.StringIO()):
            self.assertEqual(
                main(
                    [
                        "--axis",
                        "z",
                        "--execute",
                        "--confirm-home-motion",
                        "--timeout",
                        "45",
                    ]
                ),
                0,
            )
        create_runtime.assert_called_once_with(RuntimeMode.MOTION)
        run_test.assert_called_once_with(
            runtime,
            AxisName.Z,
            execute=True,
            timeout_s=45.0,
        )

    @patch("scripts.debug_motion.home_linear_axis.create_configured_runtime")
    def test_one_motion_flag_is_rejected_before_config_or_hardware(
        self,
        create_runtime: Mock,
    ) -> None:
        for flag in ("--execute", "--confirm-home-motion"):
            with self.subTest(flag=flag):
                with redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as failure:
                        main(["--axis", "slide", flag])
                self.assertEqual(failure.exception.code, 2)
        create_runtime.assert_not_called()

    @patch("scripts.debug_motion.home_linear_axis.run_home_test", return_value=False)
    @patch("scripts.debug_motion.home_linear_axis.create_configured_runtime")
    def test_unverified_home_returns_failure_exit_code(
        self,
        create_runtime: Mock,
        _run_test: Mock,
    ) -> None:
        create_runtime.return_value = MagicMock()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(
                main(
                    [
                        "--axis",
                        "slide",
                        "--execute",
                        "--confirm-home-motion",
                    ]
                ),
                1,
            )


if __name__ == "__main__":
    unittest.main()

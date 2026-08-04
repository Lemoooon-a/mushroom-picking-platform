"""统一人工运动入口的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import patch

from motion.authorization import RuntimeMode
from motion.unified_protocol import AxisName, AxisTarget, MotionCommandStatus, MultiAxisTarget
from scripts.manual_motion import (
    main,
    run_home,
    run_inspect,
    run_move,
    run_move_group,
    run_state,
    run_stop,
)
from tests.motion_cli_test_support import command_result, fake_runtime, group_result


class ManualMotionTests(unittest.TestCase):
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

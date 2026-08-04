"""任意轴子集多轴点到点 CLI 的纯 mock 安全测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import patch

from motion.unified_protocol import (
    AxisName,
    AxisTarget,
    MotionCommandStatus,
    MultiAxisTarget,
)
from scripts.debug_motion.debug_multi_axis_motion import (
    main,
    run_multi_axis_test,
)
from tests.motion_cli_test_support import fake_runtime, group_result


class DebugMultiAxisMotionTests(unittest.TestCase):
    def test_preview_keeps_only_explicit_shoulder_elbow_targets(self) -> None:
        runtime = fake_runtime()
        target = MultiAxisTarget(
            (
                AxisTarget(AxisName.SHOULDER, 20.0, 2.0),
                AxisTarget(AxisName.ELBOW, -40.0, 2.0),
            )
        )
        self.assertTrue(
            run_multi_axis_test(
                runtime,
                target,
                execute=False,
                timeout_s=10.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.get_axis_states.assert_called_once_with(
            (AxisName.SHOULDER, AxisName.ELBOW)
        )
        runtime.controller.validate_positions.assert_called_once_with(target)
        runtime.controller.submit_positions.assert_not_called()

    def test_execute_supports_two_three_and_five_axis_subsets(self) -> None:
        targets = (
            MultiAxisTarget((AxisTarget(AxisName.SLIDE, 2.0), AxisTarget(AxisName.Z, 3.0))),
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.SLIDE, 2.0),
                    AxisTarget(AxisName.SHOULDER, 3.0, 2.0),
                    AxisTarget(AxisName.ELBOW, -4.0, 2.0),
                )
            ),
            MultiAxisTarget(tuple(AxisTarget(axis, 1.0) for axis in AxisName)),
        )
        for target in targets:
            with self.subTest(axes=tuple(item.axis for item in target.targets)):
                runtime = fake_runtime()
                self.assertTrue(
                    run_multi_axis_test(
                        runtime,
                        target,
                        execute=True,
                        timeout_s=10.0,
                        confirm_rotation_no_stop=True,
                        confirm_rotation_torque_enable=True,
                        emit=lambda _line: None,
                    )
                )
                runtime.controller.submit_positions.assert_called_once_with(target)
                runtime.controller.wait_group.assert_called_once()

    def test_terminal_failure_does_not_repeat_controller_stop(self) -> None:
        runtime = fake_runtime()
        target = MultiAxisTarget((AxisTarget(AxisName.SLIDE, 2.0),))
        runtime.controller.wait_group.side_effect = lambda handle, timeout_s=None: group_result(
            target,
            MotionCommandStatus.TIMEOUT,
        )
        self.assertFalse(
            run_multi_axis_test(
                runtime,
                target,
                execute=True,
                timeout_s=10.0,
                emit=lambda _line: None,
            )
        )
        runtime.controller.stop.assert_not_called()

    def test_wait_interrupt_stops_each_participating_stoppable_axis_once(self) -> None:
        runtime = fake_runtime()
        target = MultiAxisTarget(
            (
                AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                AxisTarget(AxisName.ELBOW, -10.0, 2.0),
                AxisTarget(AxisName.ROTATION, 5.0),
            )
        )
        runtime.controller.wait_group.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_multi_axis_test(
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
            (AxisName.SHOULDER, AxisName.ELBOW),
        )

    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_empty_or_unconfirmed_targets_fail_before_runtime(self, create) -> None:
        cases = (
            [],
            ["--shoulder", "20", "--execute"],
            ["--rotation", "10", "--execute", "--confirm-motion"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()

    @patch("scripts.debug_motion.debug_multi_axis_motion.run_multi_axis_test", return_value=True)
    @patch("scripts.debug_motion.debug_multi_axis_motion.create_configured_runtime")
    def test_cli_builds_stable_subset_without_unspecified_axes(self, create, run) -> None:
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--shoulder", "20", "--elbow", "-40"]), 0)
        planned = run.call_args.args[1]
        self.assertEqual(
            tuple(item.axis for item in planned.targets),
            (AxisName.SHOULDER, AxisName.ELBOW),
        )


if __name__ == "__main__":
    unittest.main()

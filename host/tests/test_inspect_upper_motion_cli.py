"""长期只读上层运动诊断入口的纯 mock 测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import patch

from motion.authorization import RuntimeMode
from scripts.diagnostics.inspect_upper_motion import main, run_read_only_inspection
from tests.motion_cli_test_support import fake_runtime


class InspectUpperMotionTests(unittest.TestCase):
    def test_reads_all_axes_and_never_calls_control_writes(self) -> None:
        runtime = fake_runtime()
        output: list[str] = []
        run_read_only_inspection(runtime, emit=output.append)

        runtime.__enter__.assert_called_once()
        runtime.__exit__.assert_called_once()
        runtime.shoulder_joint.initialize.assert_called_once()
        runtime.elbow_joint.initialize.assert_called_once()
        runtime.controller.list_axes.assert_called_once()
        runtime.controller.get_axis_states.assert_called_once_with(tuple(__import__("motion").AxisName))
        for forbidden in (
            runtime.controller.submit_absolute,
            runtime.controller.submit_positions,
            runtime.controller.home_reference,
            runtime.controller.stop,
            runtime.rotation_axis.enable_torque,
            runtime.rotation_axis.disable_torque,
        ):
            forbidden.assert_not_called()
        rendered = "\n".join(output)
        self.assertIn("capabilities=", rendered)
        self.assertIn("axis=rotation", rendered)

    def test_read_failure_still_closes_runtime(self) -> None:
        runtime = fake_runtime()
        runtime.stm32_client.version.side_effect = RuntimeError("read failed")
        with self.assertRaisesRegex(RuntimeError, "read failed"):
            run_read_only_inspection(runtime)
        runtime.__exit__.assert_called_once()

    @patch("scripts.diagnostics.inspect_upper_motion.run_read_only_inspection")
    @patch("scripts.diagnostics.inspect_upper_motion.create_configured_runtime")
    def test_main_always_uses_read_only_runtime(self, create, run) -> None:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main([]), 0)
        create.assert_called_once_with(RuntimeMode.READ_ONLY)
        run.assert_called_once_with(create.return_value)


if __name__ == "__main__":
    unittest.main()

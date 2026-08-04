"""STM32 maintenance CLI 的纯 mock 测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import MagicMock, patch

from motion.authorization import RuntimeMode
from scripts.maintenance.stm32_motion import main, mm_to_um


class STM32MaintenanceTests(unittest.TestCase):
    def runtime(self) -> MagicMock:
        runtime = MagicMock()
        runtime.__enter__.return_value = runtime
        runtime.__exit__.return_value = None
        return runtime

    def test_units_convert_from_mm_to_um(self) -> None:
        self.assertEqual(mm_to_um(1.234), 1234)
        self.assertEqual(mm_to_um(-0.5), -500)

    @patch("scripts.maintenance.stm32_motion.create_configured_runtime")
    def test_version_and_state_are_read_only(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        for argv in (["version"], ["state", "--axis", "slide"]):
            create.reset_mock()
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(argv), 0)
            create.assert_called_once_with(RuntimeMode.READ_ONLY)

    @patch("scripts.maintenance.stm32_motion.create_configured_runtime")
    def test_move_preview_sends_nothing_and_execute_uses_converted_values(self, create) -> None:
        args = ["move", "--axis", "z", "--position-mm", "1.5", "--velocity-mm-s", "2", "--acceleration-mm-s2", "3"]
        preview_output = io.StringIO()
        with redirect_stdout(preview_output):
            self.assertEqual(main(args), 0)
        create.assert_not_called()
        self.assertIn("axis=z axis_code=Z", preview_output.getvalue())
        self.assertIn("engineering=(position=1.5 mm", preview_output.getvalue())
        self.assertIn("protocol=(position=1500 um", preview_output.getvalue())

        runtime = self.runtime()
        create.return_value = runtime
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(args + ["--execute", "--confirm-motion"]), 0)
        runtime.stm32_client.move_absolute.assert_called_once_with(
            "z", 1500, 2000, 3000, event_timeout=120.0
        )

    @patch("scripts.maintenance.stm32_motion.create_configured_runtime")
    def test_all_write_commands_require_specific_confirmation(self, create) -> None:
        cases = (
            ["home", "--axis", "slide", "--execute"],
            ["stop", "--axis", "slide", "--execute"],
            ["enable", "--axis", "z", "--execute"],
            ["disable", "--axis", "z", "--execute"],
            ["clear-fault", "--axis", "z", "--execute"],
            ["suction-start", "--execute"],
            ["suction-release", "--execute"],
            ["suction-stop", "--execute"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()


if __name__ == "__main__":
    unittest.main()

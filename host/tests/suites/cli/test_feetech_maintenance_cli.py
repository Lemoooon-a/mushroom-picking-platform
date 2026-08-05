"""Feetech maintenance CLI 的纯 mock 测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

from motion.authorization import RuntimeMode
from scripts.maintenance.feetech_rotation import build_parser, main


class FeetechMaintenanceTests(unittest.TestCase):
    def runtime(self) -> MagicMock:
        runtime = MagicMock()
        runtime.__enter__.return_value = runtime
        runtime.__exit__.return_value = None
        runtime.rotation_axis.config.servo_id = 1
        runtime.rotation_axis.config.expect_write_status = True
        runtime.rotation_axis.command_position.return_value = 123
        return runtime

    @patch("scripts.maintenance.feetech_rotation.create_configured_runtime")
    def test_read_commands_use_discovered_runtime(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        for argv in (["ping"], ["state"], ["feedback"], ["read-register", "--address", "0x38", "--length", "2"]):
            create.reset_mock()
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(argv), 0)
            create.assert_called_once_with(RuntimeMode.READ_ONLY)

    @patch("scripts.maintenance.feetech_rotation.create_configured_runtime")
    def test_move_requires_confirmations_and_never_auto_disables_torque(self, create) -> None:
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(["move", "--position-deg", "5", "--execute"])
        create.assert_not_called()
        runtime = self.runtime()
        create.return_value = runtime
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main([
                "move", "--position-deg", "5", "--execute", "--confirm-motion",
                "--confirm-rotation-no-stop", "--enable-torque"
            ]), 0)
        runtime.rotation_axis.enable_torque.assert_called_once_with()
        runtime.rotation_axis.command_position.assert_called_once()
        runtime.rotation_axis.disable_torque.assert_not_called()

    @patch("scripts.maintenance.feetech_rotation.create_configured_runtime")
    def test_torque_and_register_writes_have_independent_gates(self, create) -> None:
        cases = (
            ["torque-enable", "--execute"],
            ["torque-disable", "--execute"],
            ["write-register", "--address", "40", "--data-hex", "01", "--execute"],
        )
        for argv in cases:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(argv)
        create.assert_not_called()

    @patch("scripts.maintenance.feetech_rotation.create_configured_runtime")
    def test_torque_disable_warns_about_free_motion(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(main([
                "torque-disable", "--execute", "--confirm-free-motion-risk"
            ]), 0)
        runtime.rotation_axis.disable_torque.assert_called_once_with()
        self.assertIn("rotate freely or lose holding force", output.getvalue())

    @patch("scripts.maintenance.feetech_rotation.create_configured_runtime")
    def test_torque_enable_and_register_write_execute_explicit_operations(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main([
                "torque-enable", "--execute", "--confirm-torque-enable"
            ]), 0)
            self.assertEqual(main([
                "write-register", "--address", "0x28", "--data-hex", "01 02",
                "--execute", "--confirm-register-write",
            ]), 0)
        runtime.rotation_axis.enable_torque.assert_called_once_with()
        runtime.feetech_bus.write_registers.assert_called_once_with(
            1, 0x28, b"\x01\x02", expect_status=True
        )

    def test_no_stop_subcommand(self) -> None:
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                build_parser().parse_args(["stop"])
        source = Path(__file__).resolve().parents[3] / "scripts/maintenance/feetech_rotation.py"
        source_text = source.read_text(encoding="utf-8")
        self.assertNotRegex(source_text, r"/dev/(?:cu|tty)\.")
        self.assertNotIn("FeetechBus(", source_text)


if __name__ == "__main__":
    unittest.main()

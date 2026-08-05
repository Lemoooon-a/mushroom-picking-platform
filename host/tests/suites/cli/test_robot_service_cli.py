from __future__ import annotations

import argparse
from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import Mock

from application.robot_service import RobotServiceCapabilities
from application.runtime_state import RobotServiceMode
from calibration.hand_eye import HandEyeCalibrationStatus
from scripts.robot_service import RobotServiceShell, build_parser, format_capabilities, validate_args


class RobotServiceCliTests(unittest.TestCase):
    def test_modes_and_execute_gate(self) -> None:
        parser = build_parser()
        read_only = parser.parse_args(["--mode", "read-only"])
        validate_args(parser, read_only)
        dry_run = parser.parse_args(["--mode", "dry-run", "--fake-position", "1", "2", "3"])
        validate_args(parser, dry_run)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            validate_args(parser, parser.parse_args(["--mode", "execute"]))
        execute = parser.parse_args(["--mode", "execute", "--confirm-motion", "--confirm-rotation-no-stop"])
        validate_args(parser, execute)

    def test_shell_dispatches_status_startup_observe_and_pick(self) -> None:
        service = Mock()
        service.status.return_value = {"state": "created"}
        service.request_observation.return_value = {"request_id": "capture-1"}
        service.pick.return_value = {"outcome": "planned"}
        output = []
        shell = RobotServiceShell(service, emit=output.append)
        self.assertTrue(shell.run_command("status"))
        self.assertTrue(shell.run_command("startup"))
        self.assertTrue(shell.run_command("observe"))
        self.assertTrue(shell.run_command("pick"))
        self.assertFalse(shell.run_command("quit"))
        service.startup.assert_called_once()
        service.request_observation.assert_called_once()
        service.pick.assert_called_once()
        service.shutdown.assert_called_once()

    def test_capability_output_has_all_fail_closed_lines(self) -> None:
        service = Mock()
        service.capabilities = RobotServiceCapabilities(
            True, True, True, True, True, True, "fake available", True,
            HandEyeCalibrationStatus.MISSING, False, False, False, False,
        )
        output = "\n".join(format_capabilities(service))
        self.assertIn("Base-frame motion: available", output)
        self.assertIn("Vision gateway: fake available", output)
        self.assertIn("Hand-eye calibration: missing", output)
        self.assertIn("Pick planning: unavailable", output)
        self.assertIn("Physical pick verification: unavailable", output)


if __name__ == "__main__":
    unittest.main()

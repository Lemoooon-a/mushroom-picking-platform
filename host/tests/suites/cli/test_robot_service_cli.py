from __future__ import annotations

import argparse
from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import Mock

from application.robot_service import ResolvedCameraPoint, RobotServiceCapabilities
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

    def test_shell_resolves_and_formats_camera_point(self) -> None:
        service = Mock()
        service.resolve_camera_point.return_value = ResolvedCameraPoint(
            camera_point_mm=(20.0, -15.0, 450.0),
            base_point_mm=(185.0, 212.5, 44.85),
            frame_id="camera_color_optical_frame",
            tool_camera_source="manual_measurement",
            tool_camera_validated=False,
        )
        output = []
        shell = RobotServiceShell(service, emit=output.append)

        self.assertTrue(shell.run_command("resolve-camera-point 20 -15 450"))

        service.resolve_camera_point.assert_called_once_with(20.0, -15.0, 450.0)
        rendered = "\n".join(output)
        self.assertIn("Camera point:", rendered)
        self.assertIn("frame: camera_color_optical_frame", rendered)
        self.assertIn("x: 20.000 mm", rendered)
        self.assertIn("y: -15.000 mm", rendered)
        self.assertIn("z: 450.000 mm", rendered)
        self.assertIn("Base point:", rendered)
        self.assertIn("frame: base", rendered)
        self.assertIn("x: 185.000 mm", rendered)
        self.assertIn("y: 212.500 mm", rendered)
        self.assertIn("z: 44.850 mm", rendered)
        self.assertIn("tool_T_camera source: manual_measurement", rendered)
        self.assertIn("tool_T_camera validated: false", rendered)
        self.assertIn("result: PROVISIONAL", rendered)

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

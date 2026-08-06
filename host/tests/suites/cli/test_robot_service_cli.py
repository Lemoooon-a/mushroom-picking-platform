from __future__ import annotations

import argparse
from contextlib import redirect_stderr
import io
import unittest
from unittest.mock import Mock

from application.robot_service import ResolvedCameraPoint, RobotServiceCapabilities
from application.runtime_state import RobotServiceMode
from calibration.hand_eye import HandEyeCalibrationStatus
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
    MotionCommandResult,
    MotionCommandStatus,
)
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

    def test_axis_commands_only_dispatch_through_service(self) -> None:
        service = Mock()
        descriptor = AxisDescriptor(
            AxisName.Z,
            "Z",
            AxisKind.LINEAR,
            "mm",
            "mm/s",
            "mm/s²",
            -190.0,
            0.0,
            AxisCapabilities(True, True, True, True, True, True, True),
        )
        state = AxisState(
            AxisName.Z, True, True, False, True, True, -50.0, "mm",
            False, None, None,
        )
        result = MotionCommandResult(
            "command", AxisName.Z, MotionCommandStatus.ARRIVED, True, True,
            -60.0, -60.0, 0.0, None, "arrived",
        )
        service.list_axes.return_value = (descriptor,)
        service.get_axis_state.return_value = state
        service.get_axis_states.return_value = (state,)
        service.move_axis_absolute.return_value = result
        service.move_axis_relative.return_value = result
        output: list[str] = []
        shell = RobotServiceShell(service, emit=output.append)

        shell.run_command("axes")
        shell.run_command("axis state z")
        shell.run_command("axis states z")
        shell.run_command(
            "axis move-abs z -60 --velocity 2 --acceleration 3 --timeout 4"
        )
        shell.run_command("axis move-rel z -10 --timeout 5")

        service.list_axes.assert_called_once_with()
        service.get_axis_state.assert_called_once_with(AxisName.Z)
        service.get_axis_states.assert_called_once_with((AxisName.Z,))
        service.move_axis_absolute.assert_called_once_with(
            AxisName.Z,
            -60.0,
            velocity=2.0,
            acceleration=3.0,
            timeout_s=4.0,
        )
        service.move_axis_relative.assert_called_once_with(
            AxisName.Z,
            -10.0,
            velocity=None,
            acceleration=None,
            timeout_s=5.0,
        )
        rendered = "\n".join(output)
        self.assertIn('"unit": "mm"', rendered)
        self.assertIn('"requested_delta": -10.0', rendered)
        self.assertNotIn("controller", service.method_calls)

    def test_help_contains_raw_manual_safety_warning(self) -> None:
        output: list[str] = []
        RobotServiceShell(Mock(), emit=output.append).run_command("help")
        rendered = "\n".join(output)
        self.assertIn("raw/manual maintenance", rendered)
        self.assertIn("no Base-frame workspace", rendered)

    def test_unknown_axis_and_non_finite_values_are_rejected(self) -> None:
        shell = RobotServiceShell(Mock(), emit=lambda _line: None)
        with self.assertRaisesRegex(ValueError, "unknown axis"):
            shell.run_command("axis state unknown")
        with self.assertRaises(ValueError):
            shell.run_command("axis move-rel z not-a-number")


if __name__ == "__main__":
    unittest.main()

"""MG4010 maintenance CLI 的纯 mock 测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

from motion.authorization import RuntimeMode
from config.project.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from scripts.maintenance.mg4010_joint import main


class MG4010MaintenanceTests(unittest.TestCase):
    def runtime(self) -> MagicMock:
        runtime = MagicMock()
        runtime.__enter__.return_value = runtime
        runtime.__exit__.return_value = None
        runtime.shoulder_joint.config = SHOULDER_JOINT_CONFIG
        runtime.elbow_joint.config = ELBOW_JOINT_CONFIG
        for joint in (runtime.shoulder_joint, runtime.elbow_joint):
            joint.driver.motor_id = 1
            joint.driver.request_id = 0x141
            joint.driver.response_id = 0x241
            joint.driver.read_single_turn_position.return_value.motor_cycle_deg = (
                joint.config.encoder_zero_output_deg * joint.config.gear_ratio
            )
        return runtime

    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_read_commands_use_one_runtime_and_joint(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        for command in (
            "raw-status",
            "basic-parameters",
            "logical-angle",
            "initialize",
            "state",
        ):
            create.reset_mock()
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main([command, "--joint", "shoulder"]), 0)
            create.assert_called_once_with(RuntimeMode.READ_ONLY)

    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_logical_angle_reports_position_without_initialization(
        self, create
    ) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        runtime.elbow_joint.config = ELBOW_JOINT_CONFIG
        runtime.elbow_joint.driver.read_single_turn_position.return_value.motor_cycle_deg = (
            316.846389 * ELBOW_JOINT_CONFIG.gear_ratio
        )

        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(main(["logical-angle", "--joint", "elbow"]), 0)

        rendered = output.getvalue()
        self.assertIn("logical_position_deg=-158.846389", rendered)
        self.assertIn("output_abs_deg=316.846389", rendered)
        self.assertIn("within_limits=true", rendered)
        runtime.elbow_joint.initialize.assert_not_called()

    @patch(
        "scripts.maintenance.mg4010_joint.time.sleep",
        side_effect=KeyboardInterrupt,
    )
    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_logical_angle_watch_reuses_open_runtime_until_interrupted(
        self, create, sleep
    ) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        output = io.StringIO()

        with redirect_stdout(output), redirect_stderr(io.StringIO()):
            self.assertEqual(
                main(
                    [
                        "logical-angle",
                        "--joint",
                        "shoulder",
                        "--watch",
                        "--interval",
                        "0.25",
                    ]
                ),
                130,
            )

        self.assertIn("logical_position_deg=0.000000", output.getvalue())
        runtime.shoulder_joint.driver.read_single_turn_position.assert_called_once_with()
        create.assert_called_once_with(RuntimeMode.READ_ONLY)
        sleep.assert_called_once_with(0.25)

    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_move_preview_and_execute_use_can_rotary_joint(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        args = ["move", "--joint", "elbow", "--position-deg", "10", "--velocity-deg-s", "2"]
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(args), 0)
        runtime.elbow_joint.command_position.assert_not_called()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(args + ["--execute", "--confirm-motion"]), 0)
        runtime.elbow_joint.command_position.assert_called_once()

    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_software_stop_is_one_0x81_semantic_call(self, create) -> None:
        runtime = self.runtime()
        create.return_value = runtime
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(main([
                "software-stop", "--joint", "shoulder", "--execute", "--confirm-software-stop"
            ]), 0)
        runtime.shoulder_joint.stop.assert_called_once_with()
        self.assertIn("software stop (0x81)", output.getvalue())
        self.assertNotIn("emergency", output.getvalue().lower())
        self.assertNotIn("disabled", output.getvalue().lower())
        self.assertNotIn("torque", output.getvalue().lower())

    @patch("scripts.maintenance.mg4010_joint.create_configured_runtime")
    def test_confirmation_gate_and_no_disable_option(self, create) -> None:
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(["software-stop", "--joint", "elbow", "--execute"])
            with self.assertRaises(SystemExit):
                main(["--disable"])
        create.assert_not_called()
        source = Path(__file__).resolve().parents[3] / "scripts/maintenance/mg4010_joint.py"
        source_text = source.read_text(encoding="utf-8")
        self.assertNotIn('add_argument("--disable"', source_text)
        self.assertNotIn("CanMotorBus(", source_text)


if __name__ == "__main__":
    unittest.main()

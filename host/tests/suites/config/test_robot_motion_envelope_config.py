from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest

import config
import config.project.robot_motion_envelope as robot_motion_envelope
import config.project.workspace_planning as workspace_planning
from config.project.robot_motion_envelope import (
    DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG,
    RobotMotionEnvelopeConfig,
    SideSwitchClearanceConfig,
    StartupSafePoseConfig,
)


class RobotMotionEnvelopeConfigTests(unittest.TestCase):
    def test_project_defaults_are_frozen_without_changing_values(self) -> None:
        envelope = DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG
        self.assertEqual(
            envelope.startup_pose,
            StartupSafePoseConfig(
                base_x_mm=200.0,
                base_y_mm=0.0,
                tool_yaw_deg=0.0,
                slide_mm=0.0,
                z_axis_mm=0.0,
            ),
        )
        self.assertEqual(envelope.side_switch.clearance_base_z_mm, 150.0)

    def test_invalid_nested_types_and_clearance_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SideSwitchClearanceConfig(0.0)
        with self.assertRaises(TypeError):
            RobotMotionEnvelopeConfig(startup_pose=object())  # type: ignore[arg-type]

    def test_config_exports_are_consistent(self) -> None:
        self.assertIs(config.OffsetWorkspaceConfig, workspace_planning.OffsetWorkspaceConfig)
        self.assertIs(config.RobotMotionEnvelopeConfig, RobotMotionEnvelopeConfig)
        self.assertIs(config.StartupSafePoseConfig, StartupSafePoseConfig)
        self.assertIs(config.SideSwitchClearanceConfig, SideSwitchClearanceConfig)

    def test_pure_config_imports_do_not_read_local_files(self) -> None:
        script = """
import sys
from pathlib import Path
def blocked(*args, **kwargs):
    raise AssertionError(f'unexpected file read: {args!r}')
Path.open = blocked
import config
import config.project.robot_motion_envelope
import config.project.workspace_planning
assert 'bootstrap' not in sys.modules
assert 'config.local.hardware' not in sys.modules
assert 'config.local.motion' not in sys.modules
assert not any(name == 'drivers' or name.startswith('drivers.') for name in sys.modules)
"""
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).parents[3],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_demo_script_does_not_define_a_second_startup_config(self) -> None:
        source = (
            Path(__file__).parents[3] / "scripts" / "run_motion_demo.py"
        ).read_text(encoding="utf-8")
        for removed_name in (
            "class StartupSafePose:",
            "INITIAL_TCP_X_MM",
            "INITIAL_TCP_Y_MM",
            "INITIAL_Z_AXIS_MM",
            "INITIAL_SLIDE_AXIS_MM",
            "INITIAL_TOOL_YAW_DEG",
        ):
            self.assertNotIn(removed_name, source)


if __name__ == "__main__":
    unittest.main()

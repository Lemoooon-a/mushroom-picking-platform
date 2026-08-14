"""人工控制入口收敛和导入副作用检查。"""

from __future__ import annotations

import importlib
from pathlib import Path
import re
import unittest
from unittest.mock import patch


HOST_ROOT = Path(__file__).resolve().parents[3]
REPO_ROOT = HOST_ROOT.parent


class ScriptCleanupTests(unittest.TestCase):
    def test_only_four_control_entry_points_remain(self) -> None:
        expected = (
            "scripts/manual_motion.py",
            "scripts/maintenance/stm32_motion.py",
            "scripts/maintenance/mg4010_joint.py",
            "scripts/maintenance/feetech_rotation.py",
        )
        for relative in expected:
            self.assertTrue((HOST_ROOT / relative).is_file(), relative)

        removed = (
            "scripts/diagnostics/inspect_upper_motion.py",
            "scripts/debug_motion/debug_axis_motion.py",
            "scripts/debug_motion/debug_multi_axis_motion.py",
            "scripts/debug_motion/home_linear_axis.py",
            "scripts/test_upper_motion_runtime.py",
            "scripts/test_upper_motion_home.py",
            "scripts/test_upper_motion_five_axis.py",
            "scripts/read_motor_basic_params.py",
            "scripts/test_joint_position.py",
            "scripts/test_planar_2r_motion.py",
            "scripts/test_feetech_rotation.py",
        )
        for relative in removed:
            self.assertFalse((HOST_ROOT / relative).exists(), relative)
        self.assertEqual(tuple((HOST_ROOT / "scripts").glob("test_*.py")), ())

    def test_non_control_tools_and_algorithm_tests_remain(self) -> None:
        for relative in (
            "scripts/list_hardware_devices.py",
            "scripts/calibrate_base_slide_frame.py",
            "scripts/verify_base_slide_frame.py",
            "scripts/set_tool_camera_transform.py",
            "kinematics/planar_2r.py",
            "tests/suites/kinematics/test_planar_2r_kinematics.py",
        ):
            self.assertTrue((HOST_ROOT / relative).is_file(), relative)

        automated_tests = tuple((HOST_ROOT / "tests" / "suites").rglob("test_*.py"))
        self.assertTrue(automated_tests)
        self.assertFalse(tuple((HOST_ROOT / "tests" / "helpers").rglob("test_*.py")))

    def test_python_sources_do_not_import_removed_control_modules(self) -> None:
        removed_modules = (
            "scripts.diagnostics.inspect_upper_motion",
            "scripts.debug_motion.debug_axis_motion",
            "scripts.debug_motion.debug_multi_axis_motion",
            "scripts.debug_motion.home_linear_axis",
            "scripts.test_upper_motion_runtime",
            "scripts.test_upper_motion_home",
            "scripts.test_upper_motion_five_axis",
            "scripts.read_motor_basic_params",
            "scripts.test_joint_position",
            "scripts.test_planar_2r_motion",
            "scripts.test_feetech_rotation",
        )
        import_pattern = re.compile(r"^\s*(?:from|import)\s+([^\s]+)", re.MULTILINE)
        for source in HOST_ROOT.rglob("*.py"):
            imported = import_pattern.findall(source.read_text(encoding="utf-8"))
            for removed in removed_modules:
                self.assertNotIn(removed, imported, f"{source}: {removed}")

    def test_entry_points_reuse_runtime_instead_of_constructing_backends(self) -> None:
        sources = {
            relative: (HOST_ROOT / relative).read_text(encoding="utf-8")
            for relative in (
                "scripts/manual_motion.py",
                "scripts/maintenance/stm32_motion.py",
                "scripts/maintenance/mg4010_joint.py",
                "scripts/maintenance/feetech_rotation.py",
            )
        }
        self.assertNotIn("from drivers", sources["scripts/manual_motion.py"])
        self.assertNotIn("import drivers", sources["scripts/manual_motion.py"])
        for relative, source in sources.items():
            self.assertNotRegex(source, r"/dev/(?:cu|tty)\.")
            self.assertNotIn("CanMotorBus(", source, relative)
            self.assertNotIn("FeetechBus(", source, relative)
            self.assertNotIn("STM32MotionClient(", source, relative)

    def test_current_recommendation_docs_do_not_reference_removed_commands(self) -> None:
        documents = (
            REPO_ROOT / "host/README.md",
            REPO_ROOT / "docs/handoffs/UPPER_MOTION_DEBUG_CLI_GUIDE.md",
            REPO_ROOT / "docs/handoffs/UPPER_MOTION_RUNTIME_TEST_GUIDE.md",
            REPO_ROOT / "docs/calibration/BASE_SLIDE_FRAME_CALIBRATION.md",
            REPO_ROOT / "docs/progress/CURRENT_STATUS.md",
        )
        removed_names = (
            "inspect_upper_motion.py",
            "debug_axis_motion.py",
            "debug_multi_axis_motion.py",
            "home_linear_axis.py",
            "test_upper_motion_runtime.py",
            "test_upper_motion_home.py",
            "test_upper_motion_five_axis.py",
            "read_motor_basic_params.py",
            "test_joint_position.py",
            "test_planar_2r_motion.py",
            "test_feetech_rotation.py",
        )
        for document in documents:
            text = document.read_text(encoding="utf-8")
            for removed in removed_names:
                self.assertNotIn(removed, text, f"{document}: {removed}")

    @patch("bootstrap.create_upper_motion_runtime")
    @patch("config.hardware.load_robot_hardware_config")
    @patch("config.motion_runtime.load_robot_motion_config")
    def test_importing_four_entries_has_no_hardware_side_effect(
        self,
        load_motion,
        load_hardware,
        create_runtime,
    ) -> None:
        for name in (
            "scripts.manual_motion",
            "scripts.maintenance.stm32_motion",
            "scripts.maintenance.mg4010_joint",
            "scripts.maintenance.feetech_rotation",
        ):
            importlib.reload(importlib.import_module(name))
        load_motion.assert_not_called()
        load_hardware.assert_not_called()
        create_runtime.assert_not_called()


if __name__ == "__main__":
    unittest.main()

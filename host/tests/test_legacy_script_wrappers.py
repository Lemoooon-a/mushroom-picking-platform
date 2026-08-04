"""旧 upper-motion 人工入口薄包装和无导入副作用测试。"""

from __future__ import annotations

from contextlib import redirect_stderr
import importlib
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import scripts.test_upper_motion_five_axis as legacy_five_axis
import scripts.test_upper_motion_home as legacy_home
import scripts.test_upper_motion_runtime as legacy_runtime


class LegacyWrapperTests(unittest.TestCase):
    @patch.object(legacy_runtime, "_new_legacy_main", return_value=7)
    def test_runtime_wrapper_warns_and_forwards(self, forwarded) -> None:
        errors = io.StringIO()
        with redirect_stderr(errors):
            self.assertEqual(legacy_runtime.main(["--execute"]), 7)
        forwarded.assert_called_once_with(["--execute"])
        self.assertIn("DEPRECATED", errors.getvalue())

    @patch.object(legacy_home, "_new_main", return_value=8)
    def test_home_wrapper_warns_and_forwards_unchanged_args(self, forwarded) -> None:
        argv = ["--axis", "slide", "--execute", "--confirm-home-motion"]
        errors = io.StringIO()
        with redirect_stderr(errors):
            self.assertEqual(legacy_home.main(argv), 8)
        forwarded.assert_called_once_with(argv)
        self.assertIn("DEPRECATED", errors.getvalue())

    @patch.object(legacy_five_axis, "_new_legacy_main", return_value=9)
    def test_five_axis_wrapper_warns_and_forwards(self, forwarded) -> None:
        argv = ["--slide-mm", "1"]
        errors = io.StringIO()
        with redirect_stderr(errors):
            self.assertEqual(legacy_five_axis.main(argv), 9)
        forwarded.assert_called_once_with(argv)
        self.assertIn("DEPRECATED", errors.getvalue())

    def test_wrappers_have_no_runtime_or_control_body(self) -> None:
        scripts_root = Path(__file__).resolve().parents[1] / "scripts"
        for name in (
            "test_upper_motion_runtime.py",
            "test_upper_motion_home.py",
            "test_upper_motion_five_axis.py",
        ):
            source = (scripts_root / name).read_text(encoding="utf-8")
            self.assertNotIn("create_upper_motion_runtime", source)
            self.assertNotIn("submit_positions(", source)
            self.assertNotIn("home_reference(", source)
            self.assertNotIn(".stop(", source)

    @patch("bootstrap.create_upper_motion_runtime")
    @patch("config.hardware.load_local_hardware_config")
    @patch("config.motion_runtime.load_local_motion_config")
    def test_importing_new_modules_has_no_runtime_or_config_side_effect(
        self,
        load_motion,
        load_hardware,
        create_runtime,
    ) -> None:
        modules = (
            "scripts.diagnostics.inspect_upper_motion",
            "scripts.debug_motion.debug_axis_motion",
            "scripts.debug_motion.debug_multi_axis_motion",
            "scripts.debug_motion.home_linear_axis",
        )
        for name in modules:
            importlib.reload(importlib.import_module(name))
        load_motion.assert_not_called()
        load_hardware.assert_not_called()
        create_runtime.assert_not_called()


if __name__ == "__main__":
    unittest.main()

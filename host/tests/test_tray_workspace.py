from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

from application.tray_workspace import TargetOutsideTrayWorkspace, TrayWorkspace
from config.tray_workspace import (
    TrayWorkspaceConfig,
    TrayWorkspaceConfigError,
    load_tray_workspace_config,
)


class TrayWorkspaceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.workspace = TrayWorkspace(
            TrayWorkspaceConfig(10, 20, 30, 40, 50, 60)
        )

    def test_all_closed_interval_boundaries_are_allowed(self) -> None:
        points = (
            (10, 35, 55),
            (20, 35, 55),
            (15, 30, 55),
            (15, 40, 55),
            (15, 35, 50),
            (15, 35, 60),
        )
        for point in points:
            with self.subTest(point=point):
                self.assertTrue(self.workspace.check_xyz(*point).allowed)
                self.workspace.require_xyz_allowed(*point)

    def test_each_dimension_below_and_above_is_rejected_without_clamp(self) -> None:
        points = (
            ((9, 35, 55), ("x",)),
            ((21, 35, 55), ("x",)),
            ((15, 29, 55), ("y",)),
            ((15, 41, 55), ("y",)),
            ((15, 35, 49), ("z",)),
            ((15, 35, 61), ("z",)),
            ((9, 41, 49), ("x", "y", "z")),
        )
        for point, failed in points:
            with self.subTest(point=point):
                check = self.workspace.check_xyz(*point)
                self.assertFalse(check.allowed)
                self.assertEqual(check.failed_dimensions, failed)
                with self.assertRaises(TargetOutsideTrayWorkspace) as captured:
                    self.workspace.require_xyz_allowed(*point)
                self.assertIn("Requested:", str(captured.exception))
                self.assertIn("Allowed:", str(captured.exception))

    def test_nan_and_infinities_are_rejected(self) -> None:
        for invalid in (math.nan, math.inf, -math.inf):
            for index in range(3):
                point = [15.0, 35.0, 55.0]
                point[index] = invalid
                with self.subTest(invalid=invalid, index=index):
                    check = self.workspace.check_xyz(*point)
                    self.assertFalse(check.allowed)
                    self.assertEqual(check.failed_dimensions, ("xyz"[index],))
                    with self.assertRaises(TargetOutsideTrayWorkspace):
                        self.workspace.require_xyz_allowed(*point)

    def test_tolerance_allows_small_numeric_overshoot_without_clamping(self) -> None:
        workspace = TrayWorkspace(
            TrayWorkspaceConfig(10, 20, 30, 40, 50, 60, 1e-3)
        )
        self.assertTrue(workspace.check_xyz(9.9995, 40.0005, 50).allowed)
        self.assertFalse(workspace.check_xyz(9.998, 35, 55).allowed)


class TrayWorkspaceConfigTests(unittest.TestCase):
    def test_loader_requires_explicit_validation(self) -> None:
        payload = {
            "schema_version": 1,
            "frame": "base",
            "x_min_mm": 10,
            "x_max_mm": 20,
            "y_min_mm": 30,
            "y_max_mm": 40,
            "z_min_mm": 50,
            "z_max_mm": 60,
            "metadata": {"validated": False},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tray.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(
                TrayWorkspaceConfigError,
                "not user-validated",
            ):
                load_tray_workspace_config(path)

    def test_loader_accepts_validated_base_frame_config(self) -> None:
        payload = {
            "schema_version": 1,
            "frame": "base",
            "x_min_mm": 10,
            "x_max_mm": 20,
            "y_min_mm": 30,
            "y_max_mm": 40,
            "z_min_mm": 50,
            "z_max_mm": 60,
            "boundary_tolerance_mm": 0,
            "metadata": {"validated": True},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tray.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            config = load_tray_workspace_config(path)
        self.assertEqual(config, TrayWorkspaceConfig(10, 20, 30, 40, 50, 60, 0))


if __name__ == "__main__":
    unittest.main()

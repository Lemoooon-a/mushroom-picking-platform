from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from config.project.scan_pick import (
    ScanPickConfigError,
    load_validated_scan_pick_profile,
)


class ScanPickConfigTests(unittest.TestCase):
    @staticmethod
    def _load(payload):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan_pick.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            return load_validated_scan_pick_profile(path)

    def test_loads_two_by_four_scan_poses_in_fixed_order(self) -> None:
        payload = {
            "schema_version": 1,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_z_mm": 30,
            "scan_yaw_deg": 0,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "place_approach_height_mm": 40,
            "scan_settle_time_s": 0.5,
            "max_picks_per_scan_pose": 5,
        }
        profile = self._load(payload)

        self.assertEqual(
            [(pose.x_mm, pose.y_mm) for pose in profile.scan_poses],
            [
                (10.0, 1.0),
                (10.0, 2.0),
                (10.0, 3.0),
                (10.0, 4.0),
                (20.0, 1.0),
                (20.0, 2.0),
                (20.0, 3.0),
                (20.0, 4.0),
            ],
        )
        self.assertTrue(all(pose.yaw_deg == 0.0 for pose in profile.scan_poses))
        self.assertEqual(profile.place_pre_pose.z_mm, 110.0)
        self.assertEqual(profile.scan_settle_time_s, 0.5)

    def test_old_validated_profile_defaults_to_no_extra_settle_delay(self) -> None:
        payload = {
            "schema_version": 1,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_z_mm": 30,
            "scan_yaw_deg": 0,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "place_approach_height_mm": 40,
            "max_picks_per_scan_pose": 5,
        }

        self.assertEqual(self._load(payload).scan_settle_time_s, 0.0)

    def test_rejects_unvalidated_or_nonzero_yaw(self) -> None:
        example = (
            Path(__file__).resolve().parents[3]
            / "config"
            / "examples"
            / "scan_pick.json"
        )
        with self.assertRaisesRegex(ScanPickConfigError, "not validated"):
            load_validated_scan_pick_profile(example)

        payload = {
            "schema_version": 1,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_z_mm": 30,
            "scan_yaw_deg": 1,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "place_approach_height_mm": 40,
            "scan_settle_time_s": 0.5,
            "max_picks_per_scan_pose": 5,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan_pick.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ScanPickConfigError, "scan_yaw_deg"):
                load_validated_scan_pick_profile(path)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from config.project.scan_pick import (
    ScanPickConfigError,
    load_validated_scan_pick_profile,
)
from vision.target_size import TargetSizeClass


class ScanPickConfigTests(unittest.TestCase):
    @staticmethod
    def _load(payload):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan_pick.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            return load_validated_scan_pick_profile(path)

    def test_loads_two_by_four_scan_poses_in_fixed_order(self) -> None:
        payload = {
            "schema_version": 4,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 0,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "oversized_place_pose": {
                "x_mm": 80,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
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
        self.assertTrue(all(pose.z_mm == 150.0 for pose in profile.scan_poses))
        self.assertEqual(profile.scan_z_mm, 150.0)
        self.assertEqual(
            (
                profile.place_pose.x_mm,
                profile.place_pose.y_mm,
                profile.place_pose.z_mm,
                profile.place_pose.yaw_deg,
            ),
            (50.0, 60.0, 70.0, 0.0),
        )
        self.assertEqual(profile.scan_settle_time_s, 0.5)
        self.assertEqual(profile.oversized_place_pose.x_mm, 80.0)
        self.assertIs(
            profile.place_pose_for(TargetSizeClass.NORMAL),
            profile.place_pose,
        )
        self.assertIs(
            profile.place_pose_for(TargetSizeClass.OVERSIZED),
            profile.oversized_place_pose,
        )

    def test_oversized_place_pose_is_required(self) -> None:
        payload = {
            "schema_version": 4,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 0,
            "place_pose": {
                "x_mm": 50,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
            "max_picks_per_scan_pose": 5,
        }

        with self.assertRaisesRegex(
            ScanPickConfigError,
            "oversized_place_pose",
        ):
            self._load(payload)

    def test_missing_settle_time_defaults_to_no_extra_delay(self) -> None:
        payload = {
            "schema_version": 4,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 0,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "oversized_place_pose": {
                "x_mm": 80,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
            "max_picks_per_scan_pose": 5,
        }

        self.assertEqual(self._load(payload).scan_settle_time_s, 0.0)

    def test_rejects_unvalidated_or_nonzero_yaw(self) -> None:
        unvalidated = {
            "schema_version": 4,
            "validated": False,
        }
        with self.assertRaisesRegex(ScanPickConfigError, "not validated"):
            self._load(unvalidated)

        payload = {
            "schema_version": 4,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 1,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "oversized_place_pose": {
                "x_mm": 80,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
            "scan_settle_time_s": 0.5,
            "max_picks_per_scan_pose": 5,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan_pick.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ScanPickConfigError, "scan_yaw_deg"):
                load_validated_scan_pick_profile(path)

    def test_rejects_old_schema_and_removed_fields(self) -> None:
        payload = {
            "schema_version": 3,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 0,
            "place_pose": {"x_mm": 50, "y_mm": 60, "z_mm": 70, "yaw_deg": 0},
            "oversized_place_pose": {
                "x_mm": 80,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
            "max_picks_per_scan_pose": 5,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan_pick.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ScanPickConfigError, "schema_version"):
                load_validated_scan_pick_profile(path)

            payload["schema_version"] = 4
            payload["place_approach_height_mm"] = 40
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(
                ScanPickConfigError,
                "place_approach_height_mm was removed",
            ):
                load_validated_scan_pick_profile(path)

            payload.pop("place_approach_height_mm")
            payload["scan_z_mm"] = 150
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ScanPickConfigError, "scan_z_mm was removed"):
                load_validated_scan_pick_profile(path)

    def test_rejects_invalid_place_pose_fields(self) -> None:
        payload = {
            "schema_version": 4,
            "validated": True,
            "scan_x_positions_mm": [10, 20],
            "scan_y_positions_mm": [1, 2, 3, 4],
            "scan_yaw_deg": 0,
            "place_pose": {
                "x_mm": 50,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
                "approach_mm": 40,
            },
            "oversized_place_pose": {
                "x_mm": 80,
                "y_mm": 60,
                "z_mm": 70,
                "yaw_deg": 0,
            },
            "max_picks_per_scan_pose": 5,
        }

        with self.assertRaisesRegex(ScanPickConfigError, "exactly"):
            self._load(payload)


if __name__ == "__main__":
    unittest.main()

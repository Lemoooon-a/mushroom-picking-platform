from __future__ import annotations

import math
import json
from pathlib import Path
import tempfile
import unittest

from application.grasp_profile import GraspProfile, GraspYawMode
from config.project.grasp_strategy import (
    GraspStrategyConfigError,
    load_validated_grasp_profile,
)


class GraspProfileTests(unittest.TestCase):
    def profile(self, **changes) -> GraspProfile:
        values = dict(
            contact_offset_mm=0,
            yaw_mode=GraspYawMode.FIXED, fixed_yaw_deg=10,
            minimum_confidence=0.8, maximum_observation_age_s=2,
            suction_settle_time_s=2,
        )
        values.update(changes)
        return GraspProfile(**values)

    def test_yaw_modes(self) -> None:
        self.assertEqual(self.profile().fixed_yaw_deg, 10)
        self.assertIsNone(self.profile(yaw_mode=GraspYawMode.KEEP_CURRENT, fixed_yaw_deg=None).fixed_yaw_deg)
        self.assertIsNone(self.profile(yaw_mode=GraspYawMode.FROM_OBSERVATION, fixed_yaw_deg=None).fixed_yaw_deg)

    def test_suction_settle_defaults_to_two_seconds(self) -> None:
        profile = GraspProfile(
            contact_offset_mm=0,
            yaw_mode=GraspYawMode.FIXED,
            fixed_yaw_deg=10,
            minimum_confidence=0.8,
            maximum_observation_age_s=2,
        )
        self.assertEqual(profile.suction_settle_time_s, 2.0)

    def test_invalid_values(self) -> None:
        for changes in (
            {"contact_offset_mm": math.inf},
            {"suction_settle_time_s": -1},
            {"suction_settle_time_s": math.nan},
            {"minimum_confidence": 1.1}, {"maximum_observation_age_s": 0},
        ):
            with self.subTest(changes=changes), self.assertRaises((TypeError, ValueError)):
                self.profile(**changes)
        with self.assertRaisesRegex(ValueError, "required"):
            self.profile(fixed_yaw_deg=None)

    def test_loader_requires_schema_two_and_loads_settle_time(self) -> None:
        payload = {
            "schema_version": 2,
            "validated": True,
            "contact_offset_mm": 1,
            "yaw_mode": "fixed",
            "fixed_yaw_deg": 0,
            "minimum_confidence": 0.8,
            "maximum_observation_age_s": 8,
            "suction_settle_time_s": 2,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_profile.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            profile = load_validated_grasp_profile(path)
            self.assertEqual(profile.suction_settle_time_s, 2.0)
            payload["schema_version"] = 1
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(GraspStrategyConfigError, "schema_version"):
                load_validated_grasp_profile(path)
            payload["schema_version"] = 2
            payload["approach_offset_mm"] = 80
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(GraspStrategyConfigError, "removed fields"):
                load_validated_grasp_profile(path)


if __name__ == "__main__":
    unittest.main()

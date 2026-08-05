from __future__ import annotations

import math
import unittest

from application.grasp_profile import GraspProfile, GraspYawMode


class GraspProfileTests(unittest.TestCase):
    def profile(self, **changes) -> GraspProfile:
        values = dict(
            approach_offset_mm=20, contact_offset_mm=0, retreat_offset_mm=30,
            yaw_mode=GraspYawMode.FIXED, fixed_yaw_deg=10,
            minimum_confidence=0.8, maximum_observation_age_s=2,
        )
        values.update(changes)
        return GraspProfile(**values)

    def test_yaw_modes(self) -> None:
        self.assertEqual(self.profile().fixed_yaw_deg, 10)
        self.assertIsNone(self.profile(yaw_mode=GraspYawMode.KEEP_CURRENT, fixed_yaw_deg=None).fixed_yaw_deg)
        self.assertIsNone(self.profile(yaw_mode=GraspYawMode.FROM_OBSERVATION, fixed_yaw_deg=None).fixed_yaw_deg)

    def test_invalid_offsets_and_non_finite_values(self) -> None:
        for changes in (
            {"approach_offset_mm": -1}, {"retreat_offset_mm": -1},
            {"approach_offset_mm": math.nan}, {"contact_offset_mm": math.inf},
            {"minimum_confidence": 1.1}, {"maximum_observation_age_s": 0},
        ):
            with self.subTest(changes=changes), self.assertRaises((TypeError, ValueError)):
                self.profile(**changes)
        with self.assertRaisesRegex(ValueError, "required"):
            self.profile(fixed_yaw_deg=None)


if __name__ == "__main__":
    unittest.main()

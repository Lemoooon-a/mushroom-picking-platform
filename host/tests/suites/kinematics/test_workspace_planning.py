from __future__ import annotations

import unittest

from config.project.workspace_planning import (
    ArmLocalWorkspaceConfig,
    ArmLocalWorkspaceStatus,
)


class ArmLocalWorkspaceClassificationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ArmLocalWorkspaceConfig()

    def test_default_workspace_is_unified_x_range(self) -> None:
        self.assertEqual(
            (self.config.local_x_min_mm, self.config.local_x_max_mm),
            (100.0, 600.0),
        )

    def test_closed_workspace_boundaries(self) -> None:
        for x in (self.config.local_x_min_mm, self.config.local_x_max_mm):
            for y in (150.0, 250.0, 350.0):
                with self.subTest(x=x, y=y):
                    self.assertIs(
                        self.config.classify(x, y),
                        ArmLocalWorkspaceStatus.INSIDE,
                    )

    def test_negative_outer_y_and_x_outside_are_rejected(self) -> None:
        for y in (-350.0, -150.0, 0.0, 149.0, 351.0):
            with self.subTest(y=y):
                self.assertIs(
                    self.config.classify(200.0, y),
                    ArmLocalWorkspaceStatus.OUTSIDE,
                )
        for x in (
            self.config.local_x_min_mm - 0.001,
            self.config.local_x_max_mm + 0.001,
        ):
            for y in (250.0, -250.0):
                with self.subTest(x=x, y=y):
                    self.assertIs(
                        self.config.classify(x, y),
                        ArmLocalWorkspaceStatus.OUTSIDE,
                    )

    def test_fallback_candidates_are_finite_in_bounds_and_deterministic(self) -> None:
        first = self.config.fallback_local_y_candidates(170.0)
        self.assertEqual(
            first,
            self.config.fallback_local_y_candidates(170.0),
        )
        self.assertEqual(first[0], 170.0)
        self.assertIn(150.0, first)
        self.assertIn(350.0, first)
        self.assertNotIn(250.0, first)
        self.assertLessEqual(
            len(first),
            self.config.max_fallback_candidates,
        )
        self.assertTrue(all(150.0 <= value <= 350.0 for value in first))

    def test_arm_local_workspace_has_no_base_frame_safety_policy(self) -> None:
        self.assertFalse(hasattr(self.config, "workspace_entry_clearance_base_z_mm"))


if __name__ == "__main__":
    unittest.main()

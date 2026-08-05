from __future__ import annotations

import unittest

from config.workspace_planning import OffsetWorkspaceConfig, OffsetWorkspaceSide


class OffsetWorkspaceClassificationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = OffsetWorkspaceConfig()

    def test_closed_positive_and_negative_boundaries(self) -> None:
        for x in (50.0, 450.0):
            for y in (150.0, 250.0, 350.0):
                with self.subTest(x=x, y=y):
                    self.assertIs(
                        self.config.classify(x, y),
                        OffsetWorkspaceSide.POSITIVE,
                    )
            for y in (-350.0, -250.0, -150.0):
                with self.subTest(x=x, y=y):
                    self.assertIs(
                        self.config.classify(x, y),
                        OffsetWorkspaceSide.NEGATIVE,
                    )

    def test_center_gap_outer_y_and_x_outside_are_rejected(self) -> None:
        for y in (0.0, 149.0, -149.0, 351.0, -351.0):
            with self.subTest(y=y):
                self.assertIs(
                    self.config.classify(200.0, y),
                    OffsetWorkspaceSide.OUTSIDE,
                )
        for x in (49.999, 450.001):
            for y in (250.0, -250.0):
                with self.subTest(x=x, y=y):
                    self.assertIs(
                        self.config.classify(x, y),
                        OffsetWorkspaceSide.OUTSIDE,
                    )

    def test_fallback_candidates_are_finite_in_bounds_and_deterministic(self) -> None:
        first = self.config.fallback_local_y_candidates(
            OffsetWorkspaceSide.POSITIVE,
            170.0,
        )
        self.assertEqual(
            first,
            self.config.fallback_local_y_candidates(
                OffsetWorkspaceSide.POSITIVE,
                170.0,
            ),
        )
        self.assertEqual(first[0], 170.0)
        self.assertIn(150.0, first)
        self.assertIn(350.0, first)
        self.assertNotIn(250.0, first)
        self.assertLessEqual(
            len(first),
            self.config.max_fallback_candidates_per_side,
        )
        self.assertTrue(all(150.0 <= value <= 350.0 for value in first))

    def test_offset_workspace_has_no_base_frame_safety_policy(self) -> None:
        self.assertFalse(hasattr(self.config, "side_switch_clearance_base_z_mm"))


if __name__ == "__main__":
    unittest.main()

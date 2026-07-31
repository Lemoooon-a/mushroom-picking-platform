"""平面二旋转关节运动学的纯离线测试。"""

from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys
import unittest

from kinematics import (
    KinematicsError,
    Planar2RKinematics,
    UnreachableTargetError,
)


class ForwardKinematicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.kinematics = Planar2RKinematics(2.0, 1.0)

    def test_both_zero_aligns_links_with_positive_x(self) -> None:
        point = self.kinematics.forward(0.0, 0.0)
        self.assertAlmostEqual(point.x, 3.0)
        self.assertAlmostEqual(point.y, 0.0)

    def test_positive_shoulder_rotates_toward_positive_y(self) -> None:
        point = self.kinematics.forward(math.pi / 2, 0.0)
        self.assertAlmostEqual(point.x, 0.0)
        self.assertAlmostEqual(point.y, 3.0)

    def test_positive_elbow_is_relative_to_link1(self) -> None:
        point = self.kinematics.forward(0.0, math.pi / 2)
        self.assertAlmostEqual(point.x, 2.0)
        self.assertAlmostEqual(point.y, 1.0)

    def test_rejects_non_finite_angles(self) -> None:
        for shoulder, elbow in (
            (math.nan, 0.0),
            (math.inf, 0.0),
            (0.0, math.nan),
            (0.0, -math.inf),
        ):
            with self.subTest(shoulder=shoulder, elbow=elbow):
                with self.assertRaises(KinematicsError):
                    self.kinematics.forward(shoulder, elbow)


class InverseKinematicsTests(unittest.TestCase):
    def test_regular_target_returns_positive_then_negative_elbow(self) -> None:
        kinematics = Planar2RKinematics(1.0, 1.0)
        solutions = kinematics.inverse(1.0, 1.0)
        self.assertEqual(len(solutions), 2)
        self.assertGreater(solutions[0].elbow_rad, 0.0)
        self.assertLess(solutions[1].elbow_rad, 0.0)
        self.assertAlmostEqual(solutions[0].shoulder_rad, 0.0)
        self.assertAlmostEqual(solutions[0].elbow_rad, math.pi / 2)
        self.assertAlmostEqual(solutions[1].shoulder_rad, math.pi / 2)
        self.assertAlmostEqual(solutions[1].elbow_rad, -math.pi / 2)

    def test_every_inverse_solution_round_trips_through_forward(self) -> None:
        kinematics = Planar2RKinematics(2.3, 1.7)
        target = kinematics.forward(0.7, 1.1)
        solutions = kinematics.inverse(target.x, target.y)
        self.assertEqual(len(solutions), 2)
        for solution in solutions:
            with self.subTest(solution=solution):
                recovered = kinematics.forward(
                    solution.shoulder_rad, solution.elbow_rad
                )
                self.assertAlmostEqual(recovered.x, target.x, places=12)
                self.assertAlmostEqual(recovered.y, target.y, places=12)

    def test_outer_boundary_returns_one_straight_solution(self) -> None:
        solutions = Planar2RKinematics(2.0, 1.0).inverse(3.0, 0.0)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].shoulder_rad, 0.0)
        self.assertAlmostEqual(solutions[0].elbow_rad, 0.0)

    def test_inner_boundary_returns_one_folded_solution(self) -> None:
        solutions = Planar2RKinematics(2.0, 1.0).inverse(1.0, 0.0)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].shoulder_rad, 0.0)
        self.assertAlmostEqual(solutions[0].elbow_rad, math.pi)

    def test_equal_links_at_origin_return_one_canonical_solution(self) -> None:
        solutions = Planar2RKinematics(1.0, 1.0).inverse(0.0, 0.0)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].shoulder_rad, 0.0)
        self.assertAlmostEqual(solutions[0].elbow_rad, math.pi)
        recovered = Planar2RKinematics(1.0, 1.0).forward(
            solutions[0].shoulder_rad, solutions[0].elbow_rad
        )
        self.assertAlmostEqual(recovered.x, 0.0)
        self.assertAlmostEqual(recovered.y, 0.0)

    def test_inner_boundary_when_second_link_is_longer(self) -> None:
        solutions = Planar2RKinematics(1.0, 2.0).inverse(1.0, 0.0)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].shoulder_rad, math.pi)
        self.assertAlmostEqual(solutions[0].elbow_rad, math.pi)

    def test_small_roundoff_outside_outer_boundary_is_clamped(self) -> None:
        kinematics = Planar2RKinematics(1.0, 1.0)
        solutions = kinematics.inverse(2.0 + 1e-13, 0.0)
        self.assertEqual(len(solutions), 1)
        self.assertAlmostEqual(solutions[0].elbow_rad, 0.0)

    def test_rejects_points_outside_outer_and_inner_boundaries(self) -> None:
        kinematics = Planar2RKinematics(2.0, 1.0)
        for x, y in ((3.1, 0.0), (0.9, 0.0)):
            with self.subTest(x=x, y=y):
                with self.assertRaises(UnreachableTargetError):
                    kinematics.inverse(x, y)

    def test_rejects_non_finite_coordinates(self) -> None:
        kinematics = Planar2RKinematics(1.0, 1.0)
        for x, y in (
            (math.nan, 0.0),
            (math.inf, 0.0),
            (0.0, math.nan),
            (0.0, -math.inf),
        ):
            with self.subTest(x=x, y=y):
                with self.assertRaises(KinematicsError):
                    kinematics.inverse(x, y)


class ValidationAndIsolationTests(unittest.TestCase):
    def test_rejects_invalid_link_lengths(self) -> None:
        for link1, link2 in (
            (0.0, 1.0),
            (-1.0, 1.0),
            (math.nan, 1.0),
            (1.0, math.inf),
        ):
            with self.subTest(link1=link1, link2=link2):
                with self.assertRaises(KinematicsError):
                    Planar2RKinematics(link1, link2)

    def test_importing_kinematics_does_not_import_can_stack(self) -> None:
        host_root = Path(__file__).resolve().parents[1]
        command = (
            "import sys; import kinematics; "
            "assert 'can' not in sys.modules; "
            "assert 'drivers.can_bus' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", command],
            cwd=host_root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import math
import unittest

import numpy as np

from geometry.rigid_transform import (
    RigidTransform,
    angular_difference_deg,
)


class RigidTransformTests(unittest.TestCase):
    def test_identity(self) -> None:
        np.testing.assert_allclose(RigidTransform.identity().matrix, np.eye(4))

    def test_translation(self) -> None:
        transform = RigidTransform.from_xyz_yaw_deg(
            x_mm=1,
            y_mm=2,
            z_mm=3,
            yaw_deg=0,
        )
        np.testing.assert_allclose(transform.translation_mm, (1, 2, 3))

    def test_positive_yaw_rotates_x_to_y(self) -> None:
        transform = RigidTransform.from_xyz_yaw_deg(
            x_mm=0,
            y_mm=0,
            z_mm=0,
            yaw_deg=90,
        )
        np.testing.assert_allclose(
            transform.transform_point((1, 0, 0)),
            (0, 1, 0),
            atol=1e-12,
        )

    def test_full_rpy_uses_rz_ry_rx_order(self) -> None:
        transform = RigidTransform.from_xyz_rpy_deg(
            x_mm=0,
            y_mm=0,
            z_mm=0,
            roll_deg=20,
            pitch_deg=-30,
            yaw_deg=40,
        )
        np.testing.assert_allclose(transform.rpy_deg, (20, -30, 40), atol=1e-10)

    def test_inverse_round_trip(self) -> None:
        transform = RigidTransform.from_xyz_rpy_deg(
            x_mm=10,
            y_mm=-20,
            z_mm=30,
            roll_deg=4,
            pitch_deg=5,
            yaw_deg=6,
        )
        np.testing.assert_allclose(
            (transform @ transform.inverse()).matrix,
            np.eye(4),
            atol=1e-12,
        )

    def test_compose_order(self) -> None:
        a_T_b = RigidTransform.from_xyz_yaw_deg(
            x_mm=10, y_mm=0, z_mm=0, yaw_deg=90
        )
        b_T_c = RigidTransform.from_xyz_yaw_deg(
            x_mm=1, y_mm=0, z_mm=0, yaw_deg=0
        )
        np.testing.assert_allclose(
            (a_T_b @ b_T_c).translation_mm,
            (10, 1, 0),
            atol=1e-12,
        )

    def test_point_transform_and_inverse(self) -> None:
        transform = RigidTransform.from_xyz_yaw_deg(
            x_mm=5, y_mm=8, z_mm=13, yaw_deg=-75
        )
        point = np.array((2.0, 3.0, 4.0))
        np.testing.assert_allclose(
            transform.inverse().transform_point(transform.transform_point(point)),
            point,
            atol=1e-12,
        )

    def test_rejects_bad_shape(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            RigidTransform(np.eye(3))

    def test_rejects_non_finite_matrix(self) -> None:
        matrix = np.eye(4)
        matrix[0, 3] = math.inf
        with self.assertRaisesRegex(ValueError, "finite"):
            RigidTransform(matrix)

    def test_rejects_invalid_last_row(self) -> None:
        matrix = np.eye(4)
        matrix[3, 0] = 1
        with self.assertRaisesRegex(ValueError, "last row"):
            RigidTransform(matrix)

    def test_rejects_non_orthogonal_rotation(self) -> None:
        matrix = np.eye(4)
        matrix[0, 0] = 2
        with self.assertRaisesRegex(ValueError, "orthogonal"):
            RigidTransform(matrix)

    def test_rejects_reflection(self) -> None:
        matrix = np.eye(4)
        matrix[0, 0] = -1
        with self.assertRaisesRegex(ValueError, "reflections"):
            RigidTransform(matrix)

    def test_rejects_invalid_point(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            RigidTransform.identity().transform_point((1, 2))

    def test_matrix_is_defensively_copied_and_read_only(self) -> None:
        matrix = np.eye(4)
        transform = RigidTransform(matrix)
        matrix[0, 3] = 9
        self.assertEqual(transform.translation_mm[0], 0)
        with self.assertRaises(ValueError):
            transform.matrix[0, 3] = 1

    def test_equality_is_not_ambiguous_for_numpy_matrix(self) -> None:
        self.assertEqual(RigidTransform.identity(), RigidTransform.identity())

    def test_angular_difference_wraps_near_180(self) -> None:
        self.assertAlmostEqual(angular_difference_deg(-179, 179), 2)


if __name__ == "__main__":
    unittest.main()

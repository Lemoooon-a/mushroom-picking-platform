from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from geometry.rigid_transform import RigidTransform
from kinematics.five_axis import (
    FiveAxisGeometry,
    FiveAxisGeometryError,
    FiveAxisKinematics,
    load_five_axis_geometry,
)
from kinematics.frame_chain import RobotAxisState


HOST_ROOT = Path(__file__).resolve().parents[2]


def geometry(
    *,
    mount: RigidTransform | None = None,
    tool: RigidTransform | None = None,
) -> FiveAxisGeometry:
    return FiveAxisGeometry(
        link1_length_mm=100.0,
        link2_length_mm=50.0,
        slide_direction_xyz=(0.0, 1.0, 0.0),
        z_direction_xyz=(0.0, 0.0, 1.0),
        slide_zero_T_planar_origin_at_zero=mount or RigidTransform.identity(),
        rotation_output_T_tool=tool or RigidTransform.identity(),
    )


class FiveAxisKinematicsTests(unittest.TestCase):
    def test_all_zero_places_output_at_sum_of_links(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (150, 0, 0), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 0)

    def test_slide_and_z_follow_configured_directions(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(25, 40, 0, 0, 0)
        )
        np.testing.assert_allclose(
            result.translation_mm,
            (150, 25, 40),
            atol=1e-12,
        )

    def test_positive_shoulder_rotates_both_links(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 90, 0, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (0, 150, 0), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 90)

    def test_positive_elbow_is_relative_to_link1(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 90, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (100, 50, 0), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 90)

    def test_rotation_changes_orientation_not_planar_endpoint(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 30)
        )
        np.testing.assert_allclose(result.translation_mm, (150, 0, 0), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 30)

    def test_tcp_offset_rotates_with_rotation_output(self) -> None:
        tool = RigidTransform.from_xyz_yaw_deg(
            x_mm=10,
            y_mm=0,
            z_mm=5,
            yaw_deg=15,
        )
        result = FiveAxisKinematics(geometry(tool=tool)).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 90)
        )
        np.testing.assert_allclose(
            result.translation_mm,
            (150, 10, 5),
            atol=1e-12,
        )
        self.assertAlmostEqual(result.yaw_deg, 105)

    def test_planar_mount_transform_is_applied_before_arm(self) -> None:
        mount = RigidTransform.from_xyz_yaw_deg(
            x_mm=10,
            y_mm=20,
            z_mm=30,
            yaw_deg=90,
        )
        result = FiveAxisKinematics(geometry(mount=mount)).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 0)
        )
        np.testing.assert_allclose(
            result.translation_mm,
            (10, 170, 30),
            atol=1e-12,
        )
        self.assertAlmostEqual(result.yaw_deg, 90)

    def test_arm_local_target_uses_same_geometry_and_slide_sign_as_fk(self) -> None:
        model = FiveAxisKinematics(geometry())
        state = RobotAxisState(40.0, -30.0, 20.0, 60.0, -10.0)
        target = model.forward_kinematics(state)
        local = model.compute_arm_local_target(target, state.slide_mm)
        planar = model.planar_2r.forward(
            math.radians(state.shoulder_deg),
            math.radians(state.elbow_deg),
        )
        self.assertAlmostEqual(local.local_x_mm, planar.x)
        self.assertAlmostEqual(local.local_y_mm, planar.y)
        self.assertAlmostEqual(local.z_axis_mm, state.z_mm)
        self.assertAlmostEqual(model.slide_local_y_per_mm(), 1.0)

    def test_model_does_not_read_startup_position(self) -> None:
        model = FiveAxisKinematics(geometry())
        self.assertFalse(hasattr(model.geometry, "startup_position"))


class FiveAxisGeometryConfigTests(unittest.TestCase):
    def test_loads_confirmed_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(json.dumps(self._document()), encoding="utf-8")
            loaded = load_five_axis_geometry(path)
            self.assertEqual(loaded.link1_length_mm, 100)
            self.assertEqual(loaded.slide_direction_xyz, (0, 1, 0))

    def test_example_cannot_be_used_without_confirmation(self) -> None:
        with self.assertRaisesRegex(FiveAxisGeometryError, "geometry_confirmed"):
            load_five_axis_geometry(
                HOST_ROOT / "config" / "five_axis_geometry.example.json"
            )

    def test_rejects_non_unit_direction(self) -> None:
        document = self._document()
        document["slide_direction_xyz"] = [0, 2, 0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(FiveAxisGeometryError, "unit vector"):
                load_five_axis_geometry(path)

    def test_rejects_missing_measured_link_length(self) -> None:
        document = self._document()
        document["link_lengths_mm"] = [None, 50]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(FiveAxisGeometryError, "must be a number"):
                load_five_axis_geometry(path)

    @staticmethod
    def _document() -> dict[str, object]:
        return {
            "schema_version": 1,
            "geometry_confirmed": True,
            "link_lengths_mm": [100, 50],
            "slide_direction_xyz": [0, 1, 0],
            "z_direction_xyz": [0, 0, 1],
            "slide_zero_T_planar_origin_at_zero": {
                "translation_mm": [1, 2, 3],
                "rotation_rpy_deg": [0, 0, 0],
            },
            "rotation_output_T_tool": {
                "translation_mm": [4, 5, 6],
                "rotation_rpy_deg": [0, 0, 7],
            },
        }


if __name__ == "__main__":
    unittest.main()

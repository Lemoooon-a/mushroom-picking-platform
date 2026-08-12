from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from kinematics.five_axis import (
    FiveAxisGeometry,
    FiveAxisGeometryError,
    FiveAxisKinematics,
    load_five_axis_geometry,
)
from kinematics.frame_chain import RobotAxisState


HOST_ROOT = Path(__file__).resolve().parents[3]


def geometry(*, tcp_height_at_z_zero_mm: float = 200.0) -> FiveAxisGeometry:
    return FiveAxisGeometry(
        link1_length_mm=100.0,
        link2_length_mm=50.0,
        tcp_height_at_z_zero_mm=tcp_height_at_z_zero_mm,
    )


class FiveAxisKinematicsTests(unittest.TestCase):
    def test_all_zero_places_output_at_sum_of_links(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (150, 0, 200), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 0)

    def test_slide_and_z_follow_fixed_mechanical_directions(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(25, 40, 0, 0, 0)
        )
        np.testing.assert_allclose(
            result.translation_mm,
            (150, 25, 240),
            atol=1e-12,
        )

    def test_positive_shoulder_rotates_both_links(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 90, 0, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (0, 150, 200), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 90)

    def test_positive_elbow_is_relative_to_link1(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 90, 0)
        )
        np.testing.assert_allclose(result.translation_mm, (100, 50, 200), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 90)

    def test_rotation_changes_orientation_not_planar_endpoint(self) -> None:
        result = FiveAxisKinematics(geometry()).forward_kinematics(
            RobotAxisState(0, 0, 0, 0, 30)
        )
        np.testing.assert_allclose(result.translation_mm, (150, 0, 200), atol=1e-12)
        self.assertAlmostEqual(result.yaw_deg, 30)

    def test_rotation_never_changes_tcp_xyz(self) -> None:
        model = FiveAxisKinematics(geometry())
        states = (
            RobotAxisState(40, -30, 20, 60, rotation)
            for rotation in (-180, -30, 0, 75, 180)
        )
        translations = [model.forward_kinematics(state).translation_mm for state in states]
        for translation in translations[1:]:
            np.testing.assert_allclose(translation, translations[0], atol=1e-12)

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
            self.assertEqual(loaded.tcp_height_at_z_zero_mm, 200)

    def test_example_cannot_be_used_without_confirmation(self) -> None:
        with self.assertRaisesRegex(FiveAxisGeometryError, "geometry_confirmed"):
            load_five_axis_geometry(
                HOST_ROOT / "config" / "examples" / "five_axis_geometry.json"
            )

    def test_rejects_missing_tcp_height(self) -> None:
        document = self._document()
        document["tcp_height_at_z_zero_mm"] = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(FiveAxisGeometryError, "tcp_height_at_z_zero_mm"):
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
            "tcp_height_at_z_zero_mm": 200,
        }


if __name__ == "__main__":
    unittest.main()

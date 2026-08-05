from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from config.frame_transforms import (
    FixedFrameTransforms,
    FrameTransformConfigError,
    load_frame_transforms,
    load_frame_transforms_document,
    save_frame_transforms,
)
from geometry.rigid_transform import RigidTransform


HOST_ROOT = Path(__file__).resolve().parents[3]


class FrameTransformConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base = RigidTransform.from_xyz_yaw_deg(
            x_mm=1, y_mm=2, z_mm=3, yaw_deg=4
        )
        self.tool = RigidTransform.from_xyz_rpy_deg(
            x_mm=5,
            y_mm=6,
            z_mm=7,
            roll_deg=8,
            pitch_deg=9,
            yaw_deg=10,
        )

    def test_example_json_loads(self) -> None:
        transforms = load_frame_transforms(
            HOST_ROOT / "config" / "examples" / "frame_transforms.json"
        )
        np.testing.assert_allclose(transforms.base_T_slide_zero.matrix, np.eye(4))
        self.assertIsNone(transforms.tool_T_camera)

    def test_save_load_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            save_frame_transforms(
                path,
                FixedFrameTransforms(self.base, self.tool),
                metadata={"validated": False},
            )
            loaded = load_frame_transforms_document(path)
            np.testing.assert_allclose(
                loaded.transforms.base_T_slide_zero.matrix,
                self.base.matrix,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                loaded.transforms.tool_T_camera.matrix,
                self.tool.matrix,
                atol=1e-12,
            )
            self.assertEqual(loaded.metadata["validated"], False)

    def test_default_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            transforms = FixedFrameTransforms(self.base, None)
            save_frame_transforms(path, transforms)
            with self.assertRaises(FileExistsError):
                save_frame_transforms(path, transforms)

    def test_explicit_overwrite_replaces(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            save_frame_transforms(path, FixedFrameTransforms(self.base, None))
            save_frame_transforms(
                path,
                FixedFrameTransforms(RigidTransform.identity(), self.tool),
                overwrite=True,
            )
            loaded = load_frame_transforms(path)
            np.testing.assert_allclose(loaded.base_T_slide_zero.matrix, np.eye(4))
            self.assertIsNotNone(loaded.tool_T_camera)

    def test_saved_file_is_complete_json_and_no_temp_remains(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            save_frame_transforms(path, FixedFrameTransforms(self.base, self.tool))
            json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(tuple(Path(directory).glob("*.tmp")), ())

    def test_rejects_wrong_schema(self) -> None:
        self._assert_document_error({"schema_version": 2}, "schema_version")

    def test_rejects_missing_base(self) -> None:
        self._assert_document_error({"schema_version": 1}, "base_T_slide_zero")

    def test_rejects_invalid_translation(self) -> None:
        self._assert_document_error(
            {
                "schema_version": 1,
                "base_T_slide_zero": {
                    "translation_mm": [0, 0],
                    "rotation_rpy_deg": [0, 0, 0],
                },
            },
            "translation_mm",
        )

    def test_rejects_invalid_rpy(self) -> None:
        self._assert_document_error(
            {
                "schema_version": 1,
                "base_T_slide_zero": {
                    "translation_mm": [0, 0, 0],
                    "rotation_rpy_deg": [0, "bad", 0],
                },
            },
            "rotation_rpy_deg",
        )

    def test_rejects_corrupt_json(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            path.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(FrameTransformConfigError, "invalid JSON"):
                load_frame_transforms(path)

    def test_rejects_non_json_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(FrameTransformConfigError, "JSON serializable"):
                save_frame_transforms(
                    Path(directory) / "frames.json",
                    FixedFrameTransforms(self.base, None),
                    metadata={"bad": object()},
                )

    def _assert_document_error(self, document: object, pattern: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(FrameTransformConfigError, pattern):
                load_frame_transforms(path)


if __name__ == "__main__":
    unittest.main()

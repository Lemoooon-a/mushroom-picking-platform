from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from config.hardware import HardwareConfig, load_robot_hardware_config
from config.motion_runtime import MotionRuntimeConfig, load_robot_motion_config
from config.robot_runtime import (
    DEFAULT_ROBOT_RUNTIME_PATH,
    RobotRuntimeConfigError,
    load_robot_runtime_config,
    update_robot_runtime_frame_transforms,
)
from geometry.rigid_transform import RigidTransform
from kinematics.five_axis import FiveAxisKinematics, load_robot_five_axis_kinematics
from config.frame_transforms import FixedFrameTransforms


class RobotRuntimeConfigTests(unittest.TestCase):
    def test_default_config_loads_all_required_sections(self) -> None:
        config = load_robot_runtime_config()

        self.assertEqual(config.source_path, DEFAULT_ROBOT_RUNTIME_PATH)
        self.assertTrue(config.vision_runtime.validated)
        self.assertEqual(config.vision_runtime.host, "172.20.10.10")
        self.assertEqual(len(config.scan_pick.scan_poses), 8)
        self.assertEqual(
            (
                config.scan_pick.place_pose.x_mm,
                config.scan_pick.place_pose.y_mm,
                config.scan_pick.place_pose.z_mm,
                config.scan_pick.place_pose.yaw_deg,
            ),
            (250.0, 1000.0, 150.0, 0.0),
        )
        self.assertFalse(hasattr(config.scan_pick, "place_approach_height_mm"))
        self.assertTrue(config.recording.enabled)
        self.assertEqual(
            config.recording.jsonl_path,
            DEFAULT_ROBOT_RUNTIME_PATH.parent.parent
            / "runtime/scan-pick-real.jsonl",
        )

    def test_missing_each_required_section_is_rejected(self) -> None:
        original = self._root()
        for section in (
            "frame_transforms",
            "tray_workspace",
            "vision_runtime",
            "grasp_profile",
            "scan_pick",
            "recording",
        ):
            with self.subTest(section=section), tempfile.TemporaryDirectory() as directory:
                root = dict(original)
                root.pop(section)
                path = self._write(Path(directory), root)
                with self.assertRaisesRegex(RobotRuntimeConfigError, section):
                    load_robot_runtime_config(path)

    def test_invalid_nested_section_is_rejected_with_section_name(self) -> None:
        root = self._root()
        root["grasp_profile"] = {
            **root["grasp_profile"],
            "validated": False,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = self._write(Path(directory), root)
            with self.assertRaisesRegex(
                RobotRuntimeConfigError,
                "grasp_profile.*not validated",
            ):
                load_robot_runtime_config(path)

    def test_recording_path_must_stay_inside_runtime_cache(self) -> None:
        root = self._root()
        root["recording"] = {
            "enabled": True,
            "jsonl_path": "../outside.jsonl",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = self._write(Path(directory), root)
            with self.assertRaisesRegex(RobotRuntimeConfigError, "host/runtime"):
                load_robot_runtime_config(path)

    def test_recording_path_rejects_machine_absolute_path(self) -> None:
        root = self._root()
        root["recording"] = {
            "enabled": True,
            "jsonl_path": "/tmp/scan-pick-real.jsonl",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = self._write(Path(directory), root)
            with self.assertRaisesRegex(RobotRuntimeConfigError, "relative"):
                load_robot_runtime_config(path)

    def test_frame_update_preserves_every_other_section(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self._write(Path(directory), self._root())
            before = json.loads(path.read_text(encoding="utf-8"))
            replacement = FixedFrameTransforms(
                RigidTransform.identity(),
                RigidTransform.from_xyz_yaw_deg(
                    x_mm=1,
                    y_mm=2,
                    z_mm=3,
                    yaw_deg=4,
                ),
            )

            update_robot_runtime_frame_transforms(
                path,
                replacement,
                metadata={"validated": False, "test": True},
            )

            after = json.loads(path.read_text(encoding="utf-8"))
            for section in (
                "tray_workspace",
                "vision_runtime",
                "grasp_profile",
                "scan_pick",
                "recording",
            ):
                self.assertEqual(after[section], before[section])
            loaded = load_robot_runtime_config(path)
            self.assertEqual(loaded.frame_transforms.metadata["test"], True)

    def test_flat_robot_hardware_motion_and_geometry_configs_load(self) -> None:
        self.assertIsInstance(load_robot_hardware_config(), HardwareConfig)
        self.assertIsInstance(load_robot_motion_config(), MotionRuntimeConfig)
        self.assertIsInstance(load_robot_five_axis_kinematics(), FiveAxisKinematics)

    @staticmethod
    def _root() -> dict[str, object]:
        return json.loads(DEFAULT_ROBOT_RUNTIME_PATH.read_text(encoding="utf-8"))

    @staticmethod
    def _write(directory: Path, root: dict[str, object]) -> Path:
        path = directory / "robot_runtime.json"
        path.write_text(
            json.dumps(root, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return path


if __name__ == "__main__":
    unittest.main()

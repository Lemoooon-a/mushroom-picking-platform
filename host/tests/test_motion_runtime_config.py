"""集中运动 Runtime 配置的纯数据验证测试。"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from config.motion_runtime import (
    ArrivalConfig,
    AxisMotionProfile,
    MotionRuntimeConfig,
    MotionRuntimeConfigLoadError,
    load_local_motion_config,
)


def arrival() -> ArrivalConfig:
    return ArrivalConfig(0.2, 0.1, 0.05, 3.0)


def motion_config() -> MotionRuntimeConfig:
    profile = AxisMotionProfile(1.0, 2.0, arrival())
    return MotionRuntimeConfig(profile, profile, profile, profile, profile)


class ArrivalConfigTests(unittest.TestCase):
    def test_non_positive_required_values_are_rejected(self) -> None:
        for field_index in (0, 2, 3):
            for value in (0.0, -1.0):
                values = [0.2, 0.1, 0.05, 3.0]
                values[field_index] = value
                with self.subTest(field_index=field_index, value=value):
                    with self.assertRaises(ValueError):
                        ArrivalConfig(*values)

    def test_negative_stable_time_is_rejected_and_zero_is_legal(self) -> None:
        with self.assertRaises(ValueError):
            ArrivalConfig(0.2, -0.1, 0.05, 3.0)
        self.assertEqual(ArrivalConfig(0.2, 0.0, 0.05, 3.0).stable_time_s, 0.0)

    def test_all_fields_reject_non_finite_values(self) -> None:
        for field_index in range(4):
            for value in (math.inf, -math.inf, math.nan):
                values = [0.2, 0.1, 0.05, 3.0]
                values[field_index] = value
                with self.subTest(field_index=field_index, value=value):
                    with self.assertRaises(ValueError):
                        ArrivalConfig(*values)


class AxisMotionProfileTests(unittest.TestCase):
    def test_velocity_and_acceleration_must_be_positive_and_finite(self) -> None:
        for field_name in ("default_velocity", "default_acceleration"):
            for value in (0.0, -1.0, math.inf, -math.inf, math.nan):
                values = {
                    "default_velocity": 1.0,
                    "default_acceleration": 2.0,
                    field_name: value,
                }
                with self.subTest(field_name=field_name, value=value):
                    with self.assertRaises(ValueError):
                        AxisMotionProfile(arrival=arrival(), **values)

    def test_none_velocity_and_acceleration_are_legal(self) -> None:
        profile = AxisMotionProfile(None, None, arrival())
        self.assertIsNone(profile.default_velocity)
        self.assertIsNone(profile.default_acceleration)


class MotionConfigLoadingTests(unittest.TestCase):
    @patch("drivers.feetech_protocol.FeetechBus.open")
    @patch("drivers.can_bus.CanMotorBus.open")
    @patch("drivers.stm32_motion.STM32SerialTransport.open")
    @patch("drivers.device_discovery.GsUsb.scan")
    @patch("drivers.device_discovery.list_ports.comports")
    def test_example_import_has_no_hardware_side_effects(
        self,
        comports: Mock,
        scan: Mock,
        stm32_open: Mock,
        can_open: Mock,
        feetech_open: Mock,
    ) -> None:
        path = Path(__file__).resolve().parents[1] / "config/motion_local.example.py"
        spec = importlib.util.spec_from_file_location(
            "config.motion_local_example",
            path,
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        self.assertIsInstance(module.MOTION, MotionRuntimeConfig)
        comports.assert_not_called()
        scan.assert_not_called()
        stm32_open.assert_not_called()
        can_open.assert_not_called()
        feetech_open.assert_not_called()

    @patch("config.motion_runtime.import_module")
    def test_missing_local_config_has_copy_instruction(self, importer: Mock) -> None:
        importer.side_effect = ModuleNotFoundError(
            "missing",
            name="config.motion_local",
        )
        with self.assertRaisesRegex(
            MotionRuntimeConfigLoadError,
            "motion_local.example.py.*motion_local.py",
        ):
            load_local_motion_config()

    @patch("config.motion_runtime.import_module")
    def test_local_config_requires_expected_type(self, importer: Mock) -> None:
        importer.return_value = SimpleNamespace(MOTION=object())
        with self.assertRaisesRegex(MotionRuntimeConfigLoadError, "MotionRuntimeConfig"):
            load_local_motion_config()

    @patch("config.motion_runtime.import_module")
    def test_valid_local_config_is_returned(self, importer: Mock) -> None:
        expected = motion_config()
        importer.return_value = SimpleNamespace(MOTION=expected)
        self.assertIs(load_local_motion_config(), expected)


if __name__ == "__main__":
    unittest.main()

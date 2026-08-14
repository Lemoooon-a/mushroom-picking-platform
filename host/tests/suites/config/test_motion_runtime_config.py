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
    LinearAxisMotionLimits,
    LinearAxisPositionLimits,
    MotionRuntimeConfig,
    MotionRuntimeConfigLoadError,
    load_robot_motion_config,
)
from motion.unified_protocol import AxisName


def arrival() -> ArrivalConfig:
    return ArrivalConfig(0.2, 0.1, 0.05, 3.0)


def motion_config() -> MotionRuntimeConfig:
    profile = AxisMotionProfile(1.0, 2.0, arrival())
    return MotionRuntimeConfig(
        profile,
        profile,
        profile,
        profile,
        profile,
        LinearAxisPositionLimits(0.0, 800.0),
        LinearAxisPositionLimits(0.0, 190.0),
        LinearAxisMotionLimits(72.0, 180.0),
        LinearAxisMotionLimits(10.0, 25.0),
    )


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


class LinearAxisPositionLimitsTests(unittest.TestCase):
    def test_limits_must_be_finite_real_numbers_in_ascending_order(self) -> None:
        for minimum, maximum, error in (
            (0.0, 0.0, ValueError),
            (1.0, 0.0, ValueError),
            (math.nan, 1.0, ValueError),
            (0.0, math.inf, ValueError),
            (False, 1.0, TypeError),
            (0.0, "1", TypeError),
        ):
            with self.subTest(minimum=minimum, maximum=maximum):
                with self.assertRaises(error):
                    LinearAxisPositionLimits(minimum, maximum)  # type: ignore[arg-type]

    def test_motion_config_exposes_only_slide_and_z_limits(self) -> None:
        config = motion_config()
        self.assertEqual(
            config.linear_position_limits(),
            {
                AxisName.SLIDE: (0.0, 800.0),
                AxisName.Z: (0.0, 190.0),
            },
        )


class LinearAxisMotionLimitsTests(unittest.TestCase):
    def test_limits_must_be_positive_finite_real_numbers(self) -> None:
        for velocity, acceleration, error in (
            (0.0, 1.0, ValueError),
            (-1.0, 1.0, ValueError),
            (1.0, 0.0, ValueError),
            (1.0, math.inf, ValueError),
            (math.nan, 1.0, ValueError),
            (False, 1.0, TypeError),
            (None, 1.0, TypeError),
            (1.0, "2", TypeError),
        ):
            with self.subTest(velocity=velocity, acceleration=acceleration):
                with self.assertRaises(error):
                    LinearAxisMotionLimits(  # type: ignore[arg-type]
                        velocity,
                        acceleration,
                    )

    def test_motion_config_exposes_linear_motion_limits(self) -> None:
        self.assertEqual(
            motion_config().linear_motion_limits(),
            {
                AxisName.SLIDE: (72.0, 180.0),
                AxisName.Z: (10.0, 25.0),
            },
        )

    def test_linear_defaults_may_equal_but_not_exceed_limits(self) -> None:
        profile = AxisMotionProfile(10.0, 25.0, arrival())
        valid = MotionRuntimeConfig(
            profile,
            profile,
            profile,
            profile,
            profile,
            LinearAxisPositionLimits(0.0, 800.0),
            LinearAxisPositionLimits(0.0, 190.0),
            LinearAxisMotionLimits(10.0, 25.0),
            LinearAxisMotionLimits(10.0, 25.0),
        )
        self.assertEqual(valid.slide.default_velocity, 10.0)

        for field_name, limits in (
            ("default_velocity", LinearAxisMotionLimits(9.0, 25.0)),
            ("default_acceleration", LinearAxisMotionLimits(10.0, 24.0)),
        ):
            with self.subTest(field_name=field_name):
                with self.assertRaisesRegex(ValueError, field_name):
                    MotionRuntimeConfig(
                        profile,
                        profile,
                        profile,
                        profile,
                        profile,
                        LinearAxisPositionLimits(0.0, 800.0),
                        LinearAxisPositionLimits(0.0, 190.0),
                        limits,
                        LinearAxisMotionLimits(10.0, 25.0),
                    )


class MotionConfigLoadingTests(unittest.TestCase):
    @patch("drivers.feetech_protocol.FeetechBus.open")
    @patch("drivers.can_bus.CanMotorBus.open")
    @patch("drivers.stm32_motion.STM32SerialTransport.open")
    @patch("drivers.device_discovery.GsUsb.scan")
    @patch("drivers.device_discovery.list_ports.comports")
    def test_robot_config_import_has_no_hardware_side_effects(
        self,
        comports: Mock,
        scan: Mock,
        stm32_open: Mock,
        can_open: Mock,
        feetech_open: Mock,
    ) -> None:
        path = Path(__file__).resolve().parents[3] / "config/robot_motion.py"
        spec = importlib.util.spec_from_file_location(
            "config.robot_motion_test",
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

    @patch("config.motion_runtime._load_robot_module")
    def test_missing_robot_config_reports_fixed_path(self, importer: Mock) -> None:
        importer.side_effect = ModuleNotFoundError(
            "missing",
            name="config.robot_motion",
        )
        with self.assertRaisesRegex(
            MotionRuntimeConfigLoadError,
            "config/robot_motion.py",
        ):
            load_robot_motion_config()

    @patch("config.motion_runtime._load_robot_module")
    def test_robot_config_requires_expected_type(self, importer: Mock) -> None:
        importer.return_value = SimpleNamespace(MOTION=object())
        with self.assertRaisesRegex(MotionRuntimeConfigLoadError, "MotionRuntimeConfig"):
            load_robot_motion_config()

    @patch("config.motion_runtime._load_robot_module")
    def test_valid_robot_config_is_returned(self, importer: Mock) -> None:
        expected = motion_config()
        importer.return_value = SimpleNamespace(MOTION=expected)
        self.assertIs(load_robot_motion_config(), expected)


if __name__ == "__main__":
    unittest.main()

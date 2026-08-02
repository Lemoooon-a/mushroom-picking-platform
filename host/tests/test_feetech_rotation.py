"""Feetech 旋转轴换算、限位、反馈和 dry-run 测试。"""

from __future__ import annotations

from contextlib import redirect_stdout
import io
import math
import unittest
from unittest.mock import patch

from robot.feetech_rotation import (
    FeetechRotationAxis,
    FeetechRotationConfig,
    FeetechRotationLimitError,
    FeetechRotationPositionError,
    position_rad_to_raw,
    resolve_raw_position,
)
from scripts import test_feetech_rotation as cli


def make_config(**overrides: object) -> FeetechRotationConfig:
    values: dict[str, object] = {
        "name": "rotation",
        "servo_id": 1,
        "counts_per_turn": 4096,
        "zero_raw": 4000,
        "direction_sign": 1,
        "min_position_rad": -1.0,
        "max_position_rad": 1.0,
        "max_speed_raw": 1000,
    }
    values.update(overrides)
    return FeetechRotationConfig(**values)  # type: ignore[arg-type]


class FakeBus:
    def __init__(self, read_data: bytes = b"") -> None:
        self.read_data = read_data
        self.writes: list[tuple[int, int, bytes, bool]] = []

    def read_registers(self, servo_id: int, address: int, length: int) -> bytes:
        self.last_read = (servo_id, address, length)
        return self.read_data

    def write_registers(
        self,
        servo_id: int,
        address: int,
        data: bytes,
        *,
        expect_status: bool = True,
    ) -> None:
        self.writes.append((servo_id, address, data, expect_status))


class ConversionTests(unittest.TestCase):
    def test_zero_direction_and_encoder_wrap(self) -> None:
        config = make_config()
        self.assertEqual(position_rad_to_raw(0.0, config), 4000)
        expected = 96 * math.tau / 4096
        self.assertAlmostEqual(resolve_raw_position(0, config), expected)

    def test_negative_direction(self) -> None:
        config = make_config(direction_sign=-1, zero_raw=100)
        self.assertAlmostEqual(
            resolve_raw_position(50, config), 50 * math.tau / 4096
        )

    def test_limit_and_unmappable_position(self) -> None:
        config = make_config()
        with self.assertRaises(FeetechRotationLimitError):
            position_rad_to_raw(1.1, config)
        with self.assertRaises(FeetechRotationPositionError):
            resolve_raw_position(2000, config)


class AxisTests(unittest.TestCase):
    def test_position_command_writes_official_six_byte_layout(self) -> None:
        bus = FakeBus()
        axis = FeetechRotationAxis(bus, make_config())  # type: ignore[arg-type]
        target = axis.command_position(0.0, 100, move_time_raw=2)
        self.assertEqual(target, 4000)
        self.assertEqual(
            bus.writes,
            [(1, 0x2A, bytes.fromhex("a0 0f 02 00 64 00"), True)],
        )

    def test_all_parameters_are_checked_before_first_write(self) -> None:
        bus = FakeBus()
        axis = FeetechRotationAxis(bus, make_config())  # type: ignore[arg-type]
        with self.assertRaises(FeetechRotationLimitError):
            axis.command_position(0.0, 1001, acceleration_raw=2)
        self.assertEqual(bus.writes, [])

    def test_torque_enable_and_disable_are_explicit(self) -> None:
        bus = FakeBus()
        axis = FeetechRotationAxis(bus, make_config())  # type: ignore[arg-type]
        axis.enable_torque()
        axis.disable_torque()
        self.assertEqual(
            bus.writes,
            [(1, 0x28, b"\x01", True), (1, 0x28, b"\x00", True)],
        )

    def test_feedback_decoding(self) -> None:
        data = bytes.fromhex("a0 0f 34 12 78 56 78 1e 01 00")
        bus = FakeBus(data)
        feedback = FeetechRotationAxis(bus, make_config()).read_feedback()  # type: ignore[arg-type]
        self.assertEqual(feedback.position_raw, 4000)
        self.assertAlmostEqual(feedback.position_rad, 0.0)
        self.assertEqual(feedback.speed_raw, 0x1234)
        self.assertEqual(feedback.load_raw, 0x5678)
        self.assertTrue(feedback.moving)


class CLITests(unittest.TestCase):
    def test_default_mode_is_dry_run_and_never_constructs_bus(self) -> None:
        argv = [
            "--position-rad", "0", "--speed-raw", "100",
            "--servo-id", "1", "--counts-per-turn", "4096",
            "--zero-raw", "4000", "--direction-sign", "1",
            "--min-position-rad", "-1", "--max-position-rad", "1",
            "--max-speed-raw", "1000",
        ]
        output = io.StringIO()
        with patch.object(cli, "FeetechBus", side_effect=AssertionError("hardware opened")):
            with redirect_stdout(output):
                self.assertEqual(cli.main(argv), 0)
        self.assertIn('"mode": "dry-run"', output.getvalue())
        self.assertIn('"write_payload_hex": "a00f00006400"', output.getvalue())


if __name__ == "__main__":
    unittest.main()

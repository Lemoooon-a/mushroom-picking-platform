"""MG4010 协议层的离线字节级测试。"""

from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest


HOST_ROOT = Path(__file__).resolve().parents[3]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.mg4010_protocol import (  # noqa: E402
    CommandMismatchError,
    InvalidDlcError,
    build_position_command_2,
    build_motor_off_request,
    build_motor_run_request,
    build_read_fault_request,
    build_read_multi_turn_request,
    build_read_single_turn_request,
    build_read_status_request,
    build_request_id,
    build_response_id,
    build_stop_request,
    parse_fault_response,
    parse_multi_turn_response,
    parse_position_command_2_response,
    parse_single_turn_response,
    parse_status_response,
)


class ArbitrationIdTests(unittest.TestCase):
    def test_motor_id_boundaries(self) -> None:
        self.assertEqual(build_request_id(1), 0x141)
        self.assertEqual(build_response_id(1), 0x181)
        self.assertEqual(build_request_id(32), 0x160)
        self.assertEqual(build_response_id(32), 0x1A0)

    def test_motor_id_outside_range_is_rejected(self) -> None:
        for motor_id in (0, 33):
            with self.subTest(motor_id=motor_id):
                with self.assertRaises(ValueError):
                    build_request_id(motor_id)
                with self.assertRaises(ValueError):
                    build_response_id(motor_id)


class RequestBuilderTests(unittest.TestCase):
    def test_read_requests_are_eight_bytes(self) -> None:
        self.assertEqual(
            build_read_single_turn_request(),
            bytes.fromhex("94 00 00 00 00 00 00 00"),
        )
        self.assertEqual(
            build_read_multi_turn_request(),
            bytes.fromhex("92 00 00 00 00 00 00 00"),
        )
        self.assertEqual(
            build_read_status_request(),
            bytes.fromhex("9C 00 00 00 00 00 00 00"),
        )
        self.assertEqual(
            build_read_fault_request(),
            bytes.fromhex("9A 00 00 00 00 00 00 00"),
        )

    def test_a4_positive_angle_encoding(self) -> None:
        self.assertEqual(
            build_position_command_2(12.34, 180),
            bytes.fromhex("A4 00 B4 00 D2 04 00 00"),
        )

    def test_a4_negative_angle_uses_twos_complement(self) -> None:
        self.assertEqual(
            build_position_command_2(-1.0, 36),
            bytes.fromhex("A4 00 24 00 9C FF FF FF"),
        )

    def test_a4_max_speed_is_little_endian_uint16(self) -> None:
        payload = build_position_command_2(0, 0x1234)
        self.assertEqual(payload[2:4], bytes.fromhex("34 12"))

    def test_a4_angle_control_int32_boundaries(self) -> None:
        minimum = build_position_command_2(-(2**31) / 100, 1)
        maximum = build_position_command_2(((2**31) - 1) / 100, 1)
        self.assertEqual(minimum[4:8], struct.pack("<i", -(2**31)))
        self.assertEqual(maximum[4:8], struct.pack("<i", (2**31) - 1))

        with self.assertRaises(ValueError):
            build_position_command_2((2**31) / 100, 1)
        with self.assertRaises(ValueError):
            build_position_command_2((-(2**31) - 1) / 100, 1)

    def test_a4_max_speed_outside_uint16_is_rejected(self) -> None:
        for speed in (0, -1, 65536):
            with self.subTest(speed=speed):
                with self.assertRaises(ValueError):
                    build_position_command_2(0, speed)

    def test_stop_request_uses_0x81(self) -> None:
        self.assertEqual(
            build_stop_request(), bytes.fromhex("81 00 00 00 00 00 00 00")
        )
        self.assertNotIn(0x80, build_stop_request())

    def test_enable_and_disable_are_distinct_protocol_commands(self) -> None:
        self.assertEqual(
            build_motor_run_request(),
            bytes.fromhex("88 00 00 00 00 00 00 00"),
        )
        self.assertEqual(
            build_motor_off_request(),
            bytes.fromhex("80 00 00 00 00 00 00 00"),
        )


class ResponseParserTests(unittest.TestCase):
    def test_parse_single_turn_response(self) -> None:
        raw = 324_000
        payload = bytes([0x94, 0, 0, 0]) + struct.pack("<I", raw)
        parsed = parse_single_turn_response(payload)
        self.assertEqual(parsed.circle_angle_raw, raw)
        self.assertAlmostEqual(parsed.motor_cycle_deg, 3240.0)

    def test_parse_multi_turn_positive_response(self) -> None:
        raw = 1_234_567
        payload = bytes([0x92]) + raw.to_bytes(7, "little", signed=True)
        parsed = parse_multi_turn_response(payload)
        self.assertEqual(parsed.raw, raw)
        self.assertAlmostEqual(parsed.motor_deg, 12_345.67)

    def test_parse_multi_turn_negative_response(self) -> None:
        raw = -36_000
        payload = bytes([0x92]) + raw.to_bytes(7, "little", signed=True)
        parsed = parse_multi_turn_response(payload)
        self.assertEqual(parsed.raw, raw)
        self.assertAlmostEqual(parsed.motor_deg, -360.0)

    def test_parse_status_response(self) -> None:
        payload = bytes([0x9C, 25]) + struct.pack("<hhH", 1024, -360, 0x1234)
        parsed = parse_status_response(payload)
        self.assertEqual(parsed.temperature_c, 25)
        self.assertEqual(parsed.torque_current_raw, 1024)
        self.assertAlmostEqual(parsed.torque_current_a, 16.5)
        self.assertEqual(parsed.motor_speed_deg_s, -360)
        self.assertEqual(parsed.encoder_raw, 0x1234)

    def test_parse_fault_response(self) -> None:
        payload = bytes([0x9A, 42]) + struct.pack("<hhBB", 2400, -123, 0x10, 0x42)
        parsed = parse_fault_response(payload)
        self.assertEqual(parsed.temperature_c, 42)
        self.assertAlmostEqual(parsed.bus_voltage_v, 24.0)
        self.assertAlmostEqual(parsed.bus_current_a, -1.23)
        self.assertEqual(parsed.motor_state, 0x10)
        self.assertEqual(parsed.error_state, 0x42)

    def test_parse_position_command_response(self) -> None:
        payload = bytes([0xA4, 34]) + struct.pack("<hhH", 100, -360, 0x1234)
        parsed = parse_position_command_2_response(payload)
        self.assertEqual(parsed.temperature_c, 34)
        self.assertEqual(parsed.torque_current_raw, 100)
        self.assertEqual(parsed.motor_speed_deg_s, -360)
        self.assertEqual(parsed.encoder_raw, 0x1234)

    def test_wrong_dlc_is_rejected(self) -> None:
        with self.assertRaises(InvalidDlcError):
            parse_single_turn_response(bytes.fromhex("94 00 00 00 00 00 00"))

    def test_wrong_command_is_rejected(self) -> None:
        with self.assertRaises(CommandMismatchError):
            parse_single_turn_response(bytes.fromhex("92 00 00 00 00 00 00 00"))


if __name__ == "__main__":
    unittest.main()

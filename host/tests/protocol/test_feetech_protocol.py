"""Feetech 帧、校验、超时和串口生命周期的纯离线测试。"""

from __future__ import annotations

import unittest

from drivers.feetech_protocol import (
    FeetechBus,
    FeetechDeviceError,
    FeetechNotOpenError,
    FeetechProtocolError,
    FeetechSerialConfig,
    FeetechTimeoutError,
    build_instruction_packet,
    calculate_checksum,
    parse_status_packet,
)


def status_packet(servo_id: int, error: int = 0, parameters: bytes = b"") -> bytes:
    body = bytes((servo_id, len(parameters) + 2, error)) + parameters
    return b"\xff\xff" + body + bytes((calculate_checksum(body),))


class FakeSerial:
    def __init__(self, response: bytes = b"") -> None:
        self.response = bytearray(response)
        self.writes: list[bytes] = []
        self.is_open = True
        self.closed = False
        self.reset_calls = 0

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        result = bytes(self.response[:size])
        del self.response[:size]
        return result

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.closed = True
        self.is_open = False


class PacketTests(unittest.TestCase):
    def test_official_six_byte_position_example(self) -> None:
        packet = build_instruction_packet(
            1,
            3,
            bytes.fromhex("2a 00 08 00 00 e8 03"),
        )
        self.assertEqual(
            packet,
            bytes.fromhex("ff ff 01 09 03 2a 00 08 00 00 e8 03 d5"),
        )

    def test_parse_status_and_parameters(self) -> None:
        parsed = parse_status_packet(status_packet(2, parameters=b"\x34\x12"))
        self.assertEqual(parsed.servo_id, 2)
        self.assertEqual(parsed.parameters, b"\x34\x12")

    def test_rejects_checksum_id_length_and_device_error(self) -> None:
        packet = bytearray(status_packet(1))
        packet[-1] ^= 1
        with self.assertRaisesRegex(FeetechProtocolError, "checksum"):
            parse_status_packet(bytes(packet))
        with self.assertRaisesRegex(FeetechProtocolError, "expected ID"):
            parse_status_packet(status_packet(1), expected_id=2)
        with self.assertRaisesRegex(FeetechProtocolError, "length"):
            parse_status_packet(status_packet(1) + b"\x00")
        with self.assertRaises(FeetechDeviceError):
            parse_status_packet(status_packet(1, error=4))


class BusTests(unittest.TestCase):
    def make_bus(self, serial_port: FakeSerial) -> FeetechBus:
        return FeetechBus(
            FeetechSerialConfig("fake", 115200, timeout=0.001),
            serial_port=serial_port,
        )

    def test_read_register_transaction(self) -> None:
        serial_port = FakeSerial(status_packet(1, parameters=b"\x34\x12"))
        bus = self.make_bus(serial_port)
        self.assertEqual(bus.read_registers(1, 0x38, 2), b"\x34\x12")
        self.assertEqual(
            serial_port.writes,
            [build_instruction_packet(1, 2, b"\x38\x02")],
        )
        self.assertEqual(serial_port.reset_calls, 1)

    def test_write_can_explicitly_skip_status(self) -> None:
        serial_port = FakeSerial()
        bus = self.make_bus(serial_port)
        bus.write_registers(1, 0x28, b"\x00", expect_status=False)
        self.assertEqual(len(serial_port.writes), 1)

    def test_timeout_and_not_open_are_explicit(self) -> None:
        with self.assertRaises(FeetechTimeoutError):
            self.make_bus(FakeSerial()).ping(1)
        closed = FakeSerial()
        closed.is_open = False
        with self.assertRaises(FeetechNotOpenError):
            self.make_bus(closed).ping(1)

    def test_close_is_idempotent_and_releases_port(self) -> None:
        serial_port = FakeSerial()
        bus = self.make_bus(serial_port)
        bus.close()
        bus.close()
        self.assertTrue(serial_port.closed)
        self.assertFalse(bus.is_open)

    def test_context_manager_closes_on_exception(self) -> None:
        serial_port = FakeSerial()
        bus = self.make_bus(serial_port)
        with self.assertRaisesRegex(RuntimeError, "test"):
            with bus:
                raise RuntimeError("test")
        self.assertTrue(serial_port.closed)

    def test_short_write_is_transport_error(self) -> None:
        class ShortWriteSerial(FakeSerial):
            def write(self, data: bytes) -> int:
                return len(data) - 1

        with self.assertRaisesRegex(Exception, "short serial write"):
            self.make_bus(ShortWriteSerial()).write_registers(
                1, 0x28, b"\x00", expect_status=False
            )


if __name__ == "__main__":
    unittest.main()

"""Offline tests for the motor-side MG4010 driver facade."""

from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest

import can


HOST_ROOT = Path(__file__).resolve().parents[2]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.can_bus import CanRequestNotSentError, MotorCommunicationError  # noqa: E402
from drivers.mg4010_driver import (  # noqa: E402
    MG4010Driver,
    MotorCommandResultUnknownError,
)


class FakeCanMotorBus:
    """Scriptable transport fake; it never opens or accesses CAN hardware."""

    def __init__(
        self,
        outcomes: list[can.Message | Exception],
        *,
        send_only_error: MotorCommunicationError | None = None,
    ) -> None:
        self.outcomes = list(outcomes)
        self.calls: list[dict[str, object]] = []
        self.send_only_calls: list[tuple[int, bytes]] = []
        self.send_only_error = send_only_error

    def transact(
        self,
        arbitration_id: int,
        data: bytes,
        expected_response_id: int,
        expected_command: int,
        expected_param_id: int | None = None,
    ) -> can.Message:
        self.calls.append(
            {
                "arbitration_id": arbitration_id,
                "data": data,
                "expected_response_id": expected_response_id,
                "expected_command": expected_command,
                "expected_param_id": expected_param_id,
            }
        )
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    def send_only(self, arbitration_id: int, data: bytes) -> None:
        self.send_only_calls.append((arbitration_id, data))
        if self.send_only_error is not None:
            raise self.send_only_error


def response_message(payload: bytes, motor_id: int = 1) -> can.Message:
    return can.Message(
        arbitration_id=0x180 + motor_id,
        data=payload,
        is_extended_id=False,
        is_rx=True,
    )


class MG4010DriverReadTests(unittest.TestCase):
    def test_read_single_turn_position(self) -> None:
        raw = 324_000
        bus = FakeCanMotorBus(
            [response_message(bytes([0x94, 0, 0, 0]) + struct.pack("<I", raw))]
        )

        position = MG4010Driver(bus, 1).read_single_turn_position()

        self.assertEqual(position.circle_angle_raw, raw)
        self.assertEqual(position.motor_cycle_deg, 3240.0)
        self.assertEqual(bus.calls[0]["expected_command"], 0x94)

    def test_read_multi_turn_position_deg(self) -> None:
        raw = -356_250
        payload = bytes([0x92]) + raw.to_bytes(7, "little", signed=True)
        bus = FakeCanMotorBus([response_message(payload)])

        position_deg = MG4010Driver(bus, 1).read_multi_turn_position_deg()

        self.assertEqual(position_deg, -3562.5)
        self.assertEqual(bus.calls[0]["expected_command"], 0x92)

    def test_read_status(self) -> None:
        payload = bytes([0x9C, 34]) + struct.pack("<hhH", 100, -360, 0x1234)
        bus = FakeCanMotorBus([response_message(payload)])

        status = MG4010Driver(bus, 1).read_status()

        self.assertEqual(status.temperature_c, 34)
        self.assertEqual(status.torque_current_raw, 100)
        self.assertEqual(status.motor_speed_deg_s, -360)
        self.assertEqual(status.encoder_raw, 0x1234)
        self.assertEqual(bus.calls[0]["expected_command"], 0x9C)

    def test_read_fault(self) -> None:
        payload = bytes([0x9A, 35]) + struct.pack("<hhBB", 4800, -125, 0x00, 0x04)
        bus = FakeCanMotorBus([response_message(payload)])

        fault = MG4010Driver(bus, 1).read_fault()

        self.assertEqual(fault.temperature_c, 35)
        self.assertEqual(fault.bus_voltage_v, 48.0)
        self.assertEqual(fault.bus_current_a, -1.25)
        self.assertEqual(fault.motor_state, 0x00)
        self.assertEqual(fault.error_state, 0x04)
        self.assertEqual(bus.calls[0]["expected_command"], 0x9A)


class MG4010DriverCommandTests(unittest.TestCase):
    def test_command_position_only_waits_for_communication_response(self) -> None:
        response = response_message(
            bytes([0xA4, 34]) + struct.pack("<hhH", 0, 0, 0x1234)
        )
        bus = FakeCanMotorBus([response])

        result = MG4010Driver(bus, 1).command_position(12.34, 180.0)

        self.assertIsNone(result)
        self.assertEqual(len(bus.calls), 1)
        self.assertEqual(
            bus.calls[0]["data"],
            bytes.fromhex("A4 00 B4 00 D2 04 00 00"),
        )
        self.assertEqual(bus.calls[0]["expected_command"], 0xA4)

    def test_command_validation_failure_does_not_send_stop(self) -> None:
        bus = FakeCanMotorBus([])

        with self.assertRaises(ValueError):
            MG4010Driver(bus, 1).command_position(float("nan"), 180.0)

        self.assertEqual(bus.calls, [])

    def test_unknown_command_result_attempts_stop_then_raises(self) -> None:
        bus = FakeCanMotorBus(
            [
                MotorCommunicationError("A4 timed out"),
            ]
        )

        with self.assertRaisesRegex(
            MotorCommandResultUnknownError, "final mechanical state is unknown"
        ):
            MG4010Driver(bus, 1).command_position(12.34, 180.0)

        self.assertEqual([call["expected_command"] for call in bus.calls], [0xA4])
        self.assertEqual(
            bus.send_only_calls,
            [(0x141, bytes.fromhex("81 00 00 00 00 00 00 00"))],
        )

    def test_failure_before_send_does_not_stop_or_report_unknown_result(self) -> None:
        bus = FakeCanMotorBus([CanRequestNotSentError("bus is not open")])

        with self.assertRaisesRegex(CanRequestNotSentError, "not open"):
            MG4010Driver(bus, 1).command_position(12.34, 180.0)

        self.assertEqual([call["expected_command"] for call in bus.calls], [0xA4])
        self.assertEqual(bus.send_only_calls, [])

    def test_unknown_result_is_preserved_when_stop_also_fails(self) -> None:
        bus = FakeCanMotorBus(
            [MotorCommunicationError("A4 timed out")],
            send_only_error=MotorCommunicationError("stop send failed"),
        )

        with self.assertRaisesRegex(
            MotorCommandResultUnknownError, "stop attempt also failed"
        ):
            MG4010Driver(bus, 1).command_position(12.34, 180.0)

        self.assertEqual([call["expected_command"] for call in bus.calls], [0xA4])
        self.assertEqual(
            bus.send_only_calls,
            [(0x141, bytes.fromhex("81 00 00 00 00 00 00 00"))],
        )

    def test_stop_uses_0x81_and_protocol_response_id(self) -> None:
        bus = FakeCanMotorBus(
            [response_message(bytes.fromhex("81 00 00 00 00 00 00 00"), motor_id=2)]
        )

        MG4010Driver(bus, 2).stop()

        self.assertEqual(bus.calls[0]["arbitration_id"], 0x142)
        self.assertEqual(bus.calls[0]["expected_response_id"], 0x182)
        self.assertEqual(bus.calls[0]["expected_command"], 0x81)
        self.assertEqual(
            bus.calls[0]["data"], bytes.fromhex("81 00 00 00 00 00 00 00")
        )
        self.assertNotIn(0x80, bytes(bus.calls[0]["data"]))

    def test_enable_and_disable_wait_for_matching_echoes(self) -> None:
        bus = FakeCanMotorBus(
            [
                response_message(bytes.fromhex("88 00 00 00 00 00 00 00")),
                response_message(bytes.fromhex("80 00 00 00 00 00 00 00")),
            ]
        )
        driver = MG4010Driver(bus, 1)

        driver.enable()
        driver.disable()

        self.assertEqual(
            [call["expected_command"] for call in bus.calls],
            [0x88, 0x80],
        )
        self.assertEqual(
            [call["data"][0] for call in bus.calls],
            [0x88, 0x80],
        )


if __name__ == "__main__":
    unittest.main()

"""Offline tests for the shared MG4010 CAN transport."""

from __future__ import annotations

from collections.abc import Iterable
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import can

from drivers.can_bus import (
    CanBusNotOpenError,
    CanFrameValidationError,
    CanMotorBus,
    CanRequestNotSentError,
    CanTransactionTimeoutError,
)


REQUEST_ID = 0x141
RESPONSE_ID = 0x181
COMMAND = 0x9A
REQUEST_DATA = bytes([COMMAND, 0, 0, 0, 0, 0, 0, 0])


def response_message(
    *,
    arbitration_id: int = RESPONSE_ID,
    command: int = COMMAND,
    param_id: int = 0,
    marker: int = 0,
    is_rx: bool = True,
    is_extended_id: bool = False,
    is_remote_frame: bool = False,
    is_error_frame: bool = False,
    is_fd: bool = False,
    dlc: int | None = None,
    data: bytes | None = None,
) -> can.Message:
    payload = (
        bytes([command, param_id, marker, 0, 0, 0, 0, 0])
        if data is None
        else data
    )
    return can.Message(
        arbitration_id=arbitration_id,
        data=payload,
        dlc=dlc,
        is_rx=is_rx,
        is_extended_id=is_extended_id,
        is_remote_frame=is_remote_frame,
        is_error_frame=is_error_frame,
        is_fd=is_fd,
    )


class FakeBus:
    """Make responses visible only after send, like a real request/reply bus."""

    def __init__(
        self,
        responses_per_send: Iterable[Iterable[can.Message]] = (),
        *,
        stale: Iterable[can.Message] = (),
    ) -> None:
        self.responses_per_send = [list(items) for items in responses_per_send]
        self.stale = list(stale)
        self.pending: list[can.Message] = []
        self.sent: list[can.Message] = []
        self.recv_timeouts: list[float | None] = []
        self.shutdown_calls = 0

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        del timeout
        self.sent.append(msg)
        index = len(self.sent) - 1
        if index < len(self.responses_per_send):
            self.pending.extend(self.responses_per_send[index])

    def recv(self, timeout: float | None = None) -> can.Message | None:
        self.recv_timeouts.append(timeout)
        if timeout == 0.0:
            return self.stale.pop(0) if self.stale else None
        return self.pending.pop(0) if self.pending else None

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def make_motor_bus(
    fake: FakeBus,
    *,
    retries: int = 0,
    allow_same_id_response: bool = False,
) -> CanMotorBus:
    return CanMotorBus(
        bus=fake,
        timeout=0.001,
        retries=retries,
        allow_same_id_response=allow_same_id_response,
    )


def transact(bus: CanMotorBus, **kwargs: int | bytes) -> can.Message:
    arguments: dict[str, int | bytes | None] = {
        "arbitration_id": REQUEST_ID,
        "data": REQUEST_DATA,
        "expected_response_id": RESPONSE_ID,
        "expected_command": COMMAND,
    }
    arguments.update(kwargs)
    return bus.transact(**arguments)  # type: ignore[arg-type]


class MatchingTests(unittest.TestCase):
    def test_accepts_protocol_response_id(self) -> None:
        expected = response_message(marker=7)
        fake = FakeBus([[expected]])

        actual = transact(make_motor_bus(fake))

        self.assertIs(actual, expected)
        self.assertEqual(len(fake.sent), 1)
        request = fake.sent[0]
        self.assertEqual(request.arbitration_id, REQUEST_ID)
        self.assertEqual(bytes(request.data), REQUEST_DATA)
        self.assertFalse(request.is_extended_id)

    def test_ignores_other_motor_then_accepts_match(self) -> None:
        other_motor = response_message(arbitration_id=0x182)
        expected = response_message(marker=8)
        fake = FakeBus([[other_motor, expected]])

        self.assertIs(transact(make_motor_bus(fake)), expected)

    def test_ignores_wrong_command_then_accepts_match(self) -> None:
        wrong_command = response_message(command=0x9C)
        expected = response_message(marker=9)
        fake = FakeBus([[wrong_command, expected]])

        self.assertIs(transact(make_motor_bus(fake)), expected)

    def test_param_id_is_matched_when_requested(self) -> None:
        wrong_parameter = response_message(command=0xC0, param_id=0x20)
        expected = response_message(command=0xC0, param_id=0x0A)
        fake = FakeBus([[wrong_parameter, expected]])
        bus = make_motor_bus(fake)
        data = bytes([0xC0, 0x0A, 0, 0, 0, 0, 0, 0])

        actual = transact(
            bus,
            data=data,
            expected_command=0xC0,
            expected_param_id=0x0A,
        )

        self.assertIs(actual, expected)

    def test_default_rejects_same_id_response(self) -> None:
        fake = FakeBus([[response_message(arbitration_id=REQUEST_ID)]])

        with self.assertRaises(CanTransactionTimeoutError):
            transact(make_motor_bus(fake))

    def test_same_id_compatibility_ignores_echo(self) -> None:
        echo = response_message(
            arbitration_id=REQUEST_ID,
            data=REQUEST_DATA,
            is_rx=False,
        )
        expected = response_message(arbitration_id=REQUEST_ID, marker=10)
        fake = FakeBus([[echo, expected]])

        actual = transact(
            make_motor_bus(fake, allow_same_id_response=True)
        )

        self.assertIs(actual, expected)

    def test_same_expected_id_requires_explicit_compatibility(self) -> None:
        fake = FakeBus()

        with self.assertRaisesRegex(ValueError, "allow_same_id_response"):
            transact(
                make_motor_bus(fake),
                expected_response_id=REQUEST_ID,
            )


class InvalidFrameTests(unittest.TestCase):
    def assert_rejected(self, message: can.Message) -> None:
        fake = FakeBus([[message]])
        with self.assertRaises(CanFrameValidationError):
            transact(make_motor_bus(fake))

    def test_rejects_extended_frame(self) -> None:
        self.assert_rejected(response_message(is_extended_id=True))

    def test_rejects_remote_frame(self) -> None:
        self.assert_rejected(response_message(is_remote_frame=True))

    def test_rejects_error_frame(self) -> None:
        self.assert_rejected(response_message(is_error_frame=True))

    def test_rejects_can_fd_frame(self) -> None:
        self.assert_rejected(response_message(is_fd=True))

    def test_rejects_wrong_dlc(self) -> None:
        self.assert_rejected(response_message(dlc=7))

    def test_reports_wrong_command_if_no_match_follows(self) -> None:
        fake = FakeBus([[response_message(command=0x9C)]])

        with self.assertRaisesRegex(CanFrameValidationError, "0x9A.*0x9C"):
            transact(make_motor_bus(fake))

    def test_reports_wrong_parameter_if_no_match_follows(self) -> None:
        fake = FakeBus([[response_message(command=0xC0, param_id=0x20)]])
        bus = make_motor_bus(fake)

        with self.assertRaisesRegex(CanFrameValidationError, "ParamID"):
            transact(
                bus,
                data=bytes([0xC0, 0x0A, 0, 0, 0, 0, 0, 0]),
                expected_command=0xC0,
                expected_param_id=0x0A,
            )


class TransactionTests(unittest.TestCase):
    def test_initial_queue_drain_failure_reports_request_not_sent(self) -> None:
        fake = FakeBus()
        fake.recv = Mock(side_effect=RuntimeError("receive failed"))  # type: ignore[method-assign]

        with self.assertRaisesRegex(CanRequestNotSentError, "was not sent"):
            transact(make_motor_bus(fake))

        self.assertEqual(fake.sent, [])

    def test_timeout_retries_requested_number_of_times(self) -> None:
        fake = FakeBus([[], [], []])

        with self.assertRaises(CanTransactionTimeoutError):
            transact(make_motor_bus(fake, retries=2))

        self.assertEqual(len(fake.sent), 3)

    def test_uses_positive_monotonic_remaining_timeout(self) -> None:
        fake = FakeBus([[response_message()]])

        transact(make_motor_bus(fake))

        blocking_timeouts = [value for value in fake.recv_timeouts if value != 0.0]
        self.assertEqual(len(blocking_timeouts), 1)
        self.assertGreater(blocking_timeouts[0] or 0.0, 0.0)
        self.assertLessEqual(blocking_timeouts[0] or 0.0, 0.001)

    def test_drains_stale_frames_before_sending(self) -> None:
        stale = response_message(marker=1)
        fresh = response_message(marker=2)
        fake = FakeBus([[fresh]], stale=[stale])

        actual = transact(make_motor_bus(fake))

        self.assertIs(actual, fresh)
        self.assertFalse(fake.stale)

    def test_send_only_uses_standard_eight_byte_frame(self) -> None:
        fake = FakeBus()
        bus = make_motor_bus(fake)

        bus.send_only(REQUEST_ID, REQUEST_DATA)

        self.assertEqual(len(fake.sent), 1)
        self.assertEqual(bytes(fake.sent[0].data), REQUEST_DATA)
        self.assertFalse(fake.sent[0].is_extended_id)

    def test_operations_require_open_bus(self) -> None:
        bus = CanMotorBus(interface="socketcan", channel="can0")

        with self.assertRaises(CanBusNotOpenError):
            transact(bus)
        with self.assertRaises(CanBusNotOpenError):
            bus.send_only(REQUEST_ID, REQUEST_DATA)


class CoordinatedBus:
    """Hold motor 1 inside recv to prove motor 2 cannot enter the transaction."""

    def __init__(self) -> None:
        self.sent: list[can.Message] = []
        self.first_sent = threading.Event()
        self.release_first = threading.Event()
        self.local = threading.local()

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        del timeout
        self.sent.append(msg)
        self.local.request_id = msg.arbitration_id
        if msg.arbitration_id == 0x141:
            self.first_sent.set()

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if timeout == 0.0:
            return None
        request_id = self.local.request_id
        if request_id == 0x141:
            self.release_first.wait(timeout=1.0)
            return response_message(arbitration_id=0x181)
        return response_message(arbitration_id=0x182)

    def shutdown(self) -> None:
        pass


class SharedBusLockTests(unittest.TestCase):
    def test_two_motor_transactions_are_serialized(self) -> None:
        fake = CoordinatedBus()
        bus = CanMotorBus(bus=fake, timeout=1.0, retries=0)
        errors: list[BaseException] = []

        def run_motor(
            request_id: int, response_id: int, command: int
        ) -> None:
            try:
                bus.transact(
                    request_id,
                    bytes([command, 0, 0, 0, 0, 0, 0, 0]),
                    response_id,
                    command,
                )
            except BaseException as exc:  # captured for assertion in main thread
                errors.append(exc)

        first = threading.Thread(target=run_motor, args=(0x141, 0x181, 0x9A))
        second = threading.Thread(target=run_motor, args=(0x142, 0x182, 0x9A))
        first.start()
        self.assertTrue(fake.first_sent.wait(timeout=1.0))
        second.start()
        time.sleep(0.02)
        self.assertEqual([msg.arbitration_id for msg in fake.sent], [0x141])

        fake.release_first.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertFalse(errors)
        self.assertEqual(
            [msg.arbitration_id for msg in fake.sent],
            [0x141, 0x142],
        )


class LifecycleTests(unittest.TestCase):
    def test_close_is_idempotent(self) -> None:
        fake = FakeBus()
        bus = make_motor_bus(fake)

        bus.close()
        bus.close()

        self.assertEqual(fake.shutdown_calls, 1)
        self.assertFalse(bus.is_open)

    def test_context_manager_opens_and_closes(self) -> None:
        backend = Mock()
        with patch("drivers.can_bus.can.Bus", return_value=backend) as constructor:
            with CanMotorBus(interface="socketcan", channel="can7") as bus:
                self.assertTrue(bus.is_open)
            self.assertFalse(bus.is_open)

        constructor.assert_called_once_with(
            interface="socketcan", channel="can7", ignore_config=True
        )
        backend.shutdown.assert_called_once_with()

    @patch("gs_usb.gs_usb.GsUsb.scan", return_value=[])
    def test_gs_usb_preflight_does_not_construct_bus(self, _scan: Mock) -> None:
        with patch("drivers.can_bus.can.Bus") as constructor:
            bus = CanMotorBus(interface="gs_usb", channel=0, bitrate=1_000_000)
            with self.assertRaisesRegex(can.CanInitializationError, "Devices found: 0"):
                bus.open()

        constructor.assert_not_called()
        self.assertFalse(bus.is_open)

    def test_resolved_gs_usb_device_uses_verified_bus_address(self) -> None:
        resolved_device = SimpleNamespace(bus=3, address=7)
        backend = Mock()
        with patch("gs_usb.gs_usb.GsUsb.scan") as scan:
            with patch("drivers.can_bus.can.Bus", return_value=backend) as constructor:
                bus = CanMotorBus(
                    interface="gs_usb",
                    bitrate=1_000_000,
                    gs_usb_device=resolved_device,
                )
                bus.open()

        scan.assert_not_called()
        constructor.assert_called_once_with(
            interface="gs_usb",
            channel=0,
            ignore_config=True,
            bus=3,
            address=7,
            bitrate=1_000_000,
        )

    def test_resolved_gs_usb_device_requires_bus_address(self) -> None:
        bus = CanMotorBus(
            interface="gs_usb",
            bitrate=1_000_000,
            gs_usb_device=SimpleNamespace(bus=None, address=None),
        )
        with self.assertRaisesRegex(can.CanInitializationError, "bus and address"):
            bus.open()

    def test_resolved_gs_usb_device_rejects_socketcan(self) -> None:
        bus = CanMotorBus(
            interface="socketcan",
            channel="can0",
            gs_usb_device=SimpleNamespace(bus=1, address=2),
        )
        with self.assertRaisesRegex(ValueError, "socketcan"):
            bus.open()


if __name__ == "__main__":
    unittest.main()

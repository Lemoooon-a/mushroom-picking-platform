"""STM32 machine protocol v1 的离线客户端测试。"""

from __future__ import annotations

from pathlib import Path
import unittest

from drivers.stm32_motion import (
    PROTOCOL_VERSION,
    STM32AxisFault,
    STM32CommandError,
    STM32CommandEventError,
    STM32MotionClient,
    STM32MotionProtocolError,
    STM32MotionTimeoutError,
    parse_machine_line,
)


class FakeTransport:
    def __init__(self, lines: list[str] | None = None) -> None:
        self.lines = list(lines or [])
        self.writes: list[str] = []
        self.closed = False

    def write_line(self, line: str) -> None:
        self.writes.append(line)

    def read_line(self) -> str | None:
        return self.lines.pop(0) if self.lines else None

    def close(self) -> None:
        self.closed = True


class ParserTests(unittest.TestCase):
    def test_axis_fault_values_match_machine_protocol_v1(self) -> None:
        self.assertEqual(
            tuple(int(value) for value in STM32AxisFault),
            (0, 1, 2, 3, 4),
        )

    def test_logs_are_ignored_and_machine_lines_parsed(self) -> None:
        self.assertIsNone(parse_machine_line("I: debug"))
        message = parse_machine_line("!12 DONE Z -35000")
        self.assertEqual(message.sequence, 12)
        self.assertEqual(message.arguments, ("Z", "-35000"))

    def test_malformed_sequence_is_rejected(self) -> None:
        with self.assertRaises(STM32MotionProtocolError):
            parse_machine_line("=abc OK")


class ClientTests(unittest.TestCase):
    def test_query_axis_typed_status(self) -> None:
        transport = FakeTransport(["debug", "=0 ST Z 1 0 0 1 1 -35000 0"])
        status = STM32MotionClient(transport).query_axis("z")
        self.assertEqual(transport.writes, ["@0 QS Z"])
        self.assertTrue(status.position_valid)
        self.assertEqual(status.position_um, -35000)

    def test_nonblocking_command_preserves_early_event(self) -> None:
        transport = FakeTransport(["!0 DONE S 10000", "=0 OK"])
        event = STM32MotionClient(transport).move_relative(
            "slide", 10000, 5000, 10000
        )
        self.assertEqual(event.kind, "DONE")
        self.assertEqual(transport.writes, ["@0 MR S 10000 5000 10000"])

    def test_submit_returns_after_ok_without_waiting_for_done(self) -> None:
        transport = FakeTransport(["=0 OK"])
        client = STM32MotionClient(transport)
        submission = client.submit_move_absolute("slide", 12345, 2000, 3000)
        self.assertEqual(submission.sequence, 0)
        self.assertEqual(submission.axis, "S")
        self.assertEqual(submission.command, "MA")
        self.assertEqual(transport.writes, ["@0 MA S 12345 2000 3000"])
        self.assertIsNone(client.poll_command(submission))

    def test_submit_preserves_early_event_for_later_poll(self) -> None:
        transport = FakeTransport(["!0 DONE Z 1000", "=0 OK"])
        client = STM32MotionClient(transport)
        submission = client.submit_move_absolute("z", 1000, 2000, 3000)
        event = client.poll_command(submission)
        self.assertIsNotNone(event)
        self.assertEqual(event.kind, "DONE")

    def test_poll_returns_abort_and_fault_without_collapsing_them(self) -> None:
        for kind in ("ABORT", "FAULT"):
            transport = FakeTransport(["=0 OK", f"!0 {kind} Z 9"])
            client = STM32MotionClient(transport)
            submission = client.submit_home("z")
            event = client.poll_command(submission)
            self.assertIsNotNone(event)
            self.assertEqual(event.kind, kind)

    def test_wait_for_command_uses_submission_sequence(self) -> None:
        transport = FakeTransport(["=7 OK", "!7 DONE S 12345"])
        client = STM32MotionClient(transport, first_sequence=7)
        submission = client.submit_move_relative("slide", 12345, 2000, 3000)
        event = client.wait_for_command(submission, timeout=0.01)
        self.assertEqual(event.sequence, submission.sequence)
        self.assertEqual(event.kind, "DONE")

    def test_sequence_wraps_at_uint16(self) -> None:
        transport = FakeTransport(["=65535 OK", "=0 OK"])
        client = STM32MotionClient(transport, first_sequence=65535)
        client.stop("z")
        client.stop("z")
        self.assertEqual(transport.writes, ["@65535 ST Z", "@0 ST Z"])

    def test_synchronous_error_and_fault_event_are_distinct(self) -> None:
        with self.assertRaises(STM32CommandError) as sync_error:
            STM32MotionClient(FakeTransport(["=0 ERR 8"])).home("z")
        self.assertEqual(sync_error.exception.error_code, 8)
        with self.assertRaises(STM32CommandEventError) as event_error:
            STM32MotionClient(
                FakeTransport(["=0 OK", "!0 FAULT Z 9 0"])
            ).home("z")
        self.assertEqual(event_error.exception.event.kind, "FAULT")
        with self.assertRaises(STM32CommandEventError) as abort_error:
            STM32MotionClient(
                FakeTransport(["=0 OK", "!0 ABORT Z 0"])
            ).home("z")
        self.assertEqual(abort_error.exception.event.kind, "ABORT")

    def test_suction_query_and_done(self) -> None:
        transport = FakeTransport(["=0 SS 1 1 0 1 0", "=1 OK", "!1 DONE V 1"])
        client = STM32MotionClient(transport)
        status = client.query_suction()
        self.assertTrue(status.pump_on)
        self.assertEqual(client.suction_start().arguments, ("V", "1"))

    def test_version_stop_and_close(self) -> None:
        transport = FakeTransport(["=0 VR 1 fw123", "=1 OK"])
        client = STM32MotionClient(transport)
        self.assertEqual(client.version().protocol_version, "1")
        client.stop_all()
        client.close()
        self.assertTrue(transport.closed)

    def test_timeout_is_explicit(self) -> None:
        with self.assertRaises(STM32MotionTimeoutError):
            STM32MotionClient(FakeTransport()).query_axis("z", timeout=0.001)


class FirmwareContractTests(unittest.TestCase):
    def test_host_constants_match_submodule_protocol_header(self) -> None:
        root = Path(__file__).resolve().parents[2]
        header = (
            root
            / "firmware/stm32_motion_controller/App/Inc/app_protocol.h"
        ).read_text(encoding="utf-8")
        self.assertIn(f'#define APP_PROTOCOL_VERSION         "{PROTOCOL_VERSION}"', header)
        self.assertIn("#define APP_PROTOCOL_MAX_LINE_LENGTH (96U)", header)

    def test_supported_commands_remain_documented_by_firmware(self) -> None:
        root = Path(__file__).resolve().parents[2]
        readme = (
            root / "firmware/stm32_motion_controller/App/README.md"
        ).read_text(encoding="utf-8")
        for command in (
            "QS", "MR", "MA", "HM", "ST", "DI", "EN", "SA",
            "CF", "QH", "SQ", "SU", "SR", "SX", "VR",
        ):
            self.assertIn(f"`{command}`", readme)


if __name__ == "__main__":
    unittest.main()

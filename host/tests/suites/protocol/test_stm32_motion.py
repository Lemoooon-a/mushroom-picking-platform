"""STM32 machine protocol v2 的纯软件客户端测试。"""

from __future__ import annotations

from pathlib import Path
import threading
import time
import unittest
import warnings

from drivers.stm32_motion import (
    PROTOCOL_VERSION,
    Axis,
    AxisFault,
    MotionAbortedError,
    MotionFaultError,
    ProtocolCommandError,
    ProtocolDisconnectedError,
    ProtocolFrameError,
    ProtocolParser,
    ProtocolTimeoutError,
    STM32MotionClient,
    STM32MotionConfigurationError,
    parse_machine_line,
)


class FakeTransport:
    def __init__(self, lines: list[str] | None = None) -> None:
        self.lines = list(lines or [])
        self.writes: list[str] = []
        self.closed = False
        self.read_error: Exception | None = None
        self.write_error: Exception | None = None

    def open(self) -> None:
        self.closed = False

    def write_line(self, line: str) -> None:
        if self.write_error is not None:
            raise self.write_error
        self.writes.append(line)

    def read_line(self) -> str | None:
        if self.read_error is not None:
            raise self.read_error
        return self.lines.pop(0) if self.lines else None

    def close(self) -> None:
        self.closed = True


class ConcurrentTransport:
    """让两个 Host 线程争用读取，并以乱序 sequence 返回完整机器行。"""

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.writes: list[str] = []
        self.first_read_entered = threading.Event()
        self.release_first_read = threading.Event()
        self._guard = threading.Lock()
        self._first_read = True
        self.active_reads = 0
        self.maximum_active_reads = 0
        self.close_calls = 0

    def write_line(self, line: str) -> None:
        sequence = line.split(maxsplit=1)[0][1:]
        command = line.split(maxsplit=2)[1]
        with self._guard:
            self.writes.append(line)
            if command == "QS":
                self.lines.append(
                    f"={sequence} ST S 1 1 0 1 1 9990 0"
                )
            elif command == "MR":
                # 后发 ACK 放在先发 QS 响应之前，验证按 sequence 缓冲。
                self.lines.insert(0, f"={sequence} OK")
                self.lines.append(f"!{sequence} DONE S 19990")
            else:
                raise AssertionError(f"unexpected command: {line}")

    def read_line(self) -> str | None:
        with self._guard:
            self.active_reads += 1
            self.maximum_active_reads = max(
                self.maximum_active_reads,
                self.active_reads,
            )
            block = self._first_read
            self._first_read = False
        try:
            if block:
                self.first_read_entered.set()
                self.release_first_read.wait(timeout=1.0)
            with self._guard:
                return self.lines.pop(0) if self.lines else None
        finally:
            with self._guard:
                self.active_reads -= 1

    def close(self) -> None:
        self.close_calls += 1


class BlockingPollTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__(["=0 OK"])
        self.block_next_read = False
        self.read_entered = threading.Event()
        self.release_read = threading.Event()

    def write_line(self, line: str) -> None:
        super().write_line(line)
        if line.endswith("ST S"):
            sequence = line.split(maxsplit=1)[0][1:]
            self.lines.append(f"={sequence} OK")

    def read_line(self) -> str | None:
        if self.block_next_read:
            self.block_next_read = False
            self.read_entered.set()
            self.release_read.wait(timeout=1.0)
            return None
        return super().read_line()


class CountingErrorTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self.read_error = OSError("disconnected")
        self.close_calls = 0

    def close(self) -> None:
        self.close_calls += 1
        super().close()


class ParserTests(unittest.TestCase):
    def test_axis_fault_values_match_machine_protocol_v2(self) -> None:
        self.assertEqual(tuple(int(value) for value in AxisFault), (0, 1, 2, 3, 4))
        self.assertEqual(AxisFault.LIMIT, 1)
        self.assertEqual(AxisFault.HARDWARE_OR_CONFIG, 3)

    def test_logs_are_forwarded_and_machine_lines_parsed(self) -> None:
        logs: list[str] = []
        parser = ProtocolParser(logs.append)
        self.assertIsNone(parser.parse("I: debug"))
        message = parser.parse("!12 DONE Z -35000")
        self.assertEqual(logs, ["I: debug"])
        self.assertEqual(message.sequence, 12)
        self.assertEqual(message.arguments, ("Z", "-35000"))

    def test_v2_vacuum_fault_has_no_value_field(self) -> None:
        message = parse_machine_line("!56 FAULT V 14")
        self.assertEqual(message.arguments, ("V", "14"))

    def test_strict_parser_rejects_bad_frames(self) -> None:
        for line in (
            "=abc OK",
            "=1 ST Z 1 0",
            "=2 OK EXTRA",
            "!3 DONE X 0",
            "!4 FAULT Z nine 0",
            "!5 UNKNOWN Z 0",
            "=6 UNKNOWN",
        ):
            with self.subTest(line=line), self.assertRaises(ProtocolFrameError):
                parse_machine_line(line)


class ClientTests(unittest.TestCase):
    def test_concurrent_query_and_motion_reads_are_serialized_and_routed(self) -> None:
        transport = ConcurrentTransport()
        client = STM32MotionClient(transport)
        query_results = []
        submissions = []
        errors: list[BaseException] = []

        def query() -> None:
            try:
                query_results.append(client.query_axis("slide"))
            except BaseException as exc:
                errors.append(exc)

        def submit() -> None:
            try:
                submissions.append(
                    client.submit_move_relative("slide", 10000, 5000, 10000)
                )
            except BaseException as exc:
                errors.append(exc)

        query_thread = threading.Thread(target=query)
        submit_thread = threading.Thread(target=submit)
        query_thread.start()
        self.assertTrue(transport.first_read_entered.wait(timeout=1.0))
        submit_thread.start()
        time.sleep(0.02)
        self.assertEqual(transport.maximum_active_reads, 1)
        transport.release_first_read.set()
        query_thread.join(timeout=1.0)
        submit_thread.join(timeout=1.0)

        self.assertFalse(query_thread.is_alive())
        self.assertFalse(submit_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(query_results[0].position_um, 9990)
        self.assertEqual(client.poll_command(submissions[0]).kind, "DONE")
        self.assertEqual(transport.maximum_active_reads, 1)

    def test_stop_waits_only_for_one_active_read_not_motion_completion(self) -> None:
        transport = BlockingPollTransport()
        client = STM32MotionClient(transport)
        submission = client.submit_move_relative("slide", 10000, 5000, 10000)
        transport.block_next_read = True
        poll_results = []
        stop_completed = threading.Event()

        poll_thread = threading.Thread(
            target=lambda: poll_results.append(client.poll_command(submission))
        )
        stop_thread = threading.Thread(
            target=lambda: (client.stop("slide"), stop_completed.set())
        )
        poll_thread.start()
        self.assertTrue(transport.read_entered.wait(timeout=1.0))
        stop_thread.start()
        time.sleep(0.02)
        self.assertFalse(stop_completed.is_set())
        transport.release_read.set()
        stop_thread.join(timeout=1.0)
        poll_thread.join(timeout=1.0)

        self.assertTrue(stop_completed.is_set())
        self.assertEqual(poll_results, [None])
        transport.lines.append("!0 ABORT S 9990")
        self.assertEqual(client.poll_command(submission).kind, "ABORT")

    def test_concurrent_transport_error_disconnects_and_closes_once(self) -> None:
        transport = CountingErrorTransport()
        client = STM32MotionClient(transport)
        barrier = threading.Barrier(3)
        errors: list[BaseException] = []

        def query() -> None:
            barrier.wait()
            try:
                client.query_axis("slide")
            except BaseException as exc:
                errors.append(exc)

        threads = [threading.Thread(target=query) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=1.0)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(len(errors), 2)
        self.assertTrue(all(isinstance(exc, ProtocolDisconnectedError) for exc in errors))
        self.assertEqual(transport.close_calls, 1)
        self.assertFalse(client.pending_sync_sequences)
        self.assertFalse(client.pending_async_sequences)
        with self.assertRaises(ProtocolDisconnectedError):
            client.query_axis("slide")

    def test_version_handshake_and_resynchronization_order(self) -> None:
        transport = FakeTransport(
            [
                "=0 VR 2 fw123",
                "=1 ST Z 1 0 0 0 0 0 2",
                "=2 ST S 1 0 0 1 1 0 0",
                "=3 HS 0 S 6 0",
                "=4 SS 0 0 0 0 0",
            ]
        )
        snapshot = STM32MotionClient(transport).resynchronize()
        self.assertEqual(snapshot.version.protocol_version, PROTOCOL_VERSION)
        self.assertEqual(snapshot.z.axis, Axis.Z)
        self.assertEqual(snapshot.slide.axis, Axis.SLIDE)
        self.assertEqual(
            transport.writes,
            ["@0 VR", "@1 QS Z", "@2 QS S", "@3 QH", "@4 SQ"],
        )

    def test_reconnect_discards_pending_and_rebuilds_read_only_state(self) -> None:
        transport = FakeTransport(
            [
                "=1 VR 2 fw123",
                "=2 ST Z 1 0 0 0 0 0 2",
                "=3 ST S 1 0 0 0 0 0 2",
                "=4 HS 0 Z 0 0",
                "=5 SS 0 0 0 0 0",
            ]
        )
        client = STM32MotionClient(transport, first_sequence=1)
        snapshot = client.reconnect()
        self.assertEqual(snapshot.version.protocol_version, "2")
        self.assertFalse(transport.closed)
        self.assertFalse(client.pending_sync_sequences)
        self.assertFalse(client.pending_async_sequences)

    def test_query_axis_uses_hardware_ready_and_preserves_unknown_fault(self) -> None:
        transport = FakeTransport(["debug", "=0 ST Z 1 0 0 1 1 -35000 99"])
        status = STM32MotionClient(transport).query_axis("z")
        self.assertTrue(status.hardware_ready)
        self.assertEqual(status.position_um, -35000)
        self.assertEqual(status.fault, 99)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            self.assertTrue(status.driver_configured)
        self.assertEqual(len(caught), 1)

    def test_home_z_and_slide_wait_for_done(self) -> None:
        for axis, code in (("z", "Z"), ("slide", "S")):
            transport = FakeTransport(["=0 OK", f"!0 DONE {code} 0"])
            event = STM32MotionClient(transport).home(axis)
            self.assertEqual(event.kind, "DONE")
            self.assertEqual(transport.writes, [f"@0 HM {code}"])

    def test_relative_and_absolute_moves_complete(self) -> None:
        relative = FakeTransport(["=0 OK", "!0 DONE S 10000"])
        self.assertEqual(
            STM32MotionClient(relative)
            .move_relative("slide", 10000, 5000, 10000)
            .kind,
            "DONE",
        )
        absolute = FakeTransport(["=0 OK", "!0 DONE Z 35000"])
        self.assertEqual(
            STM32MotionClient(absolute)
            .move_absolute("z", 35000, 5000, 10000)
            .arguments,
            ("Z", "35000"),
        )

    def test_stop_ok_and_original_abort_are_separate_sequences(self) -> None:
        transport = FakeTransport(["=0 OK", "!0 ABORT S 1234", "=1 OK"])
        client = STM32MotionClient(transport)
        submission = client.submit_move_relative("slide", 10000, 5000, 10000)
        client.stop("slide")
        event = client.wait_for_command(submission)
        self.assertEqual(event.kind, "ABORT")
        self.assertEqual(transport.writes, ["@0 MR S 10000 5000 10000", "@1 ST S"])

    def test_sync_err_fault_and_abort_have_distinct_exceptions(self) -> None:
        with self.assertRaises(ProtocolCommandError) as rejected:
            STM32MotionClient(FakeTransport(["=0 ERR 7"])).home("z")
        self.assertEqual(rejected.exception.error_code, 7)

        with self.assertRaises(MotionFaultError) as faulted:
            STM32MotionClient(
                FakeTransport(["=0 OK", "!0 FAULT Z 9 0"])
            ).home("z")
        self.assertEqual(faulted.exception.error_code, 9)
        self.assertEqual(faulted.exception.value, 0)
        with self.assertRaises(MotionAbortedError) as aborted:
            STM32MotionClient(
                FakeTransport(["=0 OK", "!0 ABORT Z 0"])
            ).home("z")
        self.assertEqual(aborted.exception.target, "Z")

    def test_early_event_is_buffered_until_ack_and_poll(self) -> None:
        transport = FakeTransport(["!0 DONE S 10000", "=0 OK"])
        client = STM32MotionClient(transport)
        submission = client.submit_move_relative("slide", 10000, 5000, 10000)
        self.assertEqual(client.poll_command(submission).kind, "DONE")

    def test_sequence_wrap_skips_still_pending_sequence(self) -> None:
        transport = FakeTransport(["=65535 OK", "=0 OK"])
        client = STM32MotionClient(transport, first_sequence=65535)
        client.submit_home("z")
        client._sequence = 65535
        client.stop("slide")
        self.assertEqual(transport.writes, ["@65535 HM Z", "@0 ST S"])
        self.assertEqual(client.pending_async_sequences, frozenset({65535}))

    def test_duplicate_submission_object_cannot_complete_twice(self) -> None:
        transport = FakeTransport(["=0 OK", "!0 DONE Z 0"])
        client = STM32MotionClient(transport)
        submission = client.submit_home("z")
        client.wait_for_command(submission)
        with self.assertRaises(ProtocolFrameError):
            client.poll_command(submission)

    def test_disconnect_clears_pending_and_future_commands_fail_explicitly(self) -> None:
        transport = FakeTransport(["=0 OK"])
        client = STM32MotionClient(transport)
        client.submit_home("z")
        client.disconnect("cable removed")
        self.assertFalse(client.pending_sync_sequences)
        self.assertFalse(client.pending_async_sequences)
        with self.assertRaises(ProtocolDisconnectedError):
            client.stop("z")

    def test_transport_error_is_mapped_to_disconnected(self) -> None:
        transport = FakeTransport()
        transport.write_error = OSError("gone")
        with self.assertRaises(ProtocolDisconnectedError):
            STM32MotionClient(transport).stop("z")

    def test_suction_status_operations_and_stop_abort(self) -> None:
        transport = FakeTransport(
            [
                "=0 SS 1 1 0 0 0",
                "=1 OK",
                "!1 DONE V 1",
                "=2 OK",
                "!2 DONE V 0",
                "=3 OK",
                "!3 ABORT V 0",
                "=4 OK",
            ]
        )
        client = STM32MotionClient(transport)
        self.assertTrue(client.get_suction_status().pump_on)
        self.assertEqual(client.suction().kind, "DONE")
        self.assertEqual(client.release().kind, "DONE")
        pending_release = client._submit_nonblocking("SR", "V", 2.0)
        client.stop_suction()
        self.assertEqual(client.wait_for_command(pending_release).kind, "ABORT")

    def test_semantic_suction_aliases_use_su_sr_and_sx(self) -> None:
        transport = FakeTransport(
            [
                "=0 OK",
                "!0 DONE V 1",
                "=1 OK",
                "!1 DONE V 0",
                "=2 OK",
            ]
        )
        client = STM32MotionClient(transport)
        client.suction_grip()
        client.suction_release()
        client.suction_idle()
        self.assertEqual(transport.writes, ["@0 SU", "@1 SR", "@2 SX"])

    def test_mm_conversion_rounds_signed_values_and_checks_ranges(self) -> None:
        transport = FakeTransport(["=0 ERR 8"])
        with self.assertRaises(ProtocolCommandError):
            STM32MotionClient(transport).move_absolute_mm("z", -35.0, 5.0, 10.0)
        self.assertEqual(transport.writes, ["@0 MA Z -35000 5000 10000"])

        transport = FakeTransport(["=0 OK", "!0 DONE S 1"])
        STM32MotionClient(transport).move_relative_mm("slide", 0.0014, 0.001, 0.001)
        self.assertEqual(transport.writes, ["@0 MR S 1 1 1"])

        with self.assertRaises(STM32MotionConfigurationError):
            STM32MotionClient(FakeTransport()).move_absolute_mm(
                "z", float("inf"), 1.0, 1.0
            )
        with self.assertRaises(STM32MotionConfigurationError):
            STM32MotionClient(FakeTransport()).move_relative(
                "z", 2**31, 1, 1
            )

    def test_timeout_is_explicit_and_releases_sync_sequence(self) -> None:
        client = STM32MotionClient(FakeTransport())
        with self.assertRaises(ProtocolTimeoutError):
            client.query_axis("z", timeout=0.001)
        self.assertFalse(client.pending_sync_sequences)


class FirmwareContractTests(unittest.TestCase):
    def test_host_constants_match_submodule_protocol_header(self) -> None:
        root = Path(__file__).resolve().parents[4]
        header = (
            root / "firmware/stm32_motion_controller/App/Inc/app_protocol.h"
        ).read_text(encoding="utf-8")
        self.assertIn(f'#define APP_PROTOCOL_VERSION         "{PROTOCOL_VERSION}"', header)
        self.assertIn("#define APP_PROTOCOL_MAX_LINE_LENGTH (96U)", header)

    def test_frozen_document_contains_complete_command_set(self) -> None:
        root = Path(__file__).resolve().parents[4]
        document = (
            root
            / "firmware/stm32_motion_controller/docs/stm32_motion_protocol_v2.md"
        ).read_text(encoding="utf-8")
        for command in (
            "QS", "MR", "MA", "HM", "ST", "DI", "EN", "SA",
            "CF", "QH", "VR", "SQ", "SU", "SR", "SX",
        ):
            self.assertIn(f"`{command}`", document)

    def test_firmware_dispatch_and_wire_formats_match_v2_contract(self) -> None:
        root = Path(__file__).resolve().parents[4]
        firmware = root / "firmware/stm32_motion_controller"
        source = (firmware / "App/Src/app_protocol.c").read_text(encoding="utf-8")
        document = (firmware / "docs/stm32_motion_protocol_v2.md").read_text(
            encoding="utf-8"
        )
        vectors = (
            firmware / "docs/stm32_motion_protocol_v2_test_vectors.md"
        ).read_text(encoding="utf-8")

        for command in (
            "QS", "MR", "MA", "HM", "ST", "DI", "EN", "SA",
            "CF", "QH", "VR", "SQ", "SU", "SR", "SX",
        ):
            self.assertIn(f'"{command}"', source)
        for wire_format in (
            "ST <axis> <hardware_ready>",
            "FAULT <Z|S> <protocol_error> <position_um>",
            "FAULT V <protocol_error>",
        ):
            self.assertIn(wire_format, document)
        self.assertIn("@4 MA Z -35000 5000 10000", vectors)
        self.assertIn("@45 MA Z 35000 5000 10000", vectors)


if __name__ == "__main__":
    unittest.main()

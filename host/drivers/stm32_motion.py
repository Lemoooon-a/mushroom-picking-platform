"""STM32 motion protocol v2 的正式主机侧客户端。

协议真值位于 ``firmware/stm32_motion_controller/App/Inc/app_protocol.h``
和同目录 ``README.md``。本模块只在显式 ``open()`` 后打开串口。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import time
from collections.abc import Callable
from typing import Protocol
import warnings


PROTOCOL_VERSION = "2"
MAX_LINE_LENGTH = 96
AXIS_CODES = {"z": "Z", "slide": "S", "Z": "Z", "S": "S"}
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1
UINT32_MAX = 2**32 - 1


class Axis(str, Enum):
    """Stable machine-protocol axis codes."""

    Z = "Z"
    SLIDE = "S"


class AxisFault(IntEnum):
    """Stable ``QS`` axis-status fault values from machine protocol v2."""

    NONE = 0
    LIMIT = 1
    POSITION_INVALID = 2
    HARDWARE_OR_CONFIG = 3
    HOMING = 4


# Compatibility name retained for existing upper-layer imports.
STM32AxisFault = AxisFault


class STM32MotionError(Exception):
    """STM32 motion 客户端基础异常。"""


class STM32MotionConfigurationError(STM32MotionError):
    """串口或命令参数无效。"""


class STM32MotionTimeoutError(STM32MotionError):
    """等待同步响应或异步事件超时。"""


class STM32MotionProtocolError(STM32MotionError):
    """收到不符合协议 v2 的机器行。"""


class ProtocolFrameError(STM32MotionProtocolError):
    """机器协议帧结构、字段类型或字段数量非法。"""


class ProtocolTimeoutError(STM32MotionTimeoutError):
    """等待同步响应或异步终态超时，设备动作状态未知。"""


class ProtocolDisconnectedError(STM32MotionError):
    """串口已断开；本地 pending 已失效但设备状态未知。"""


class STM32CommandError(STM32MotionError):
    """同步响应明确拒绝命令。"""

    def __init__(self, sequence: int, error_code: int) -> None:
        super().__init__(f"STM32 command {sequence} failed with error {error_code}")
        self.sequence = sequence
        self.error_code = error_code


class ProtocolCommandError(STM32CommandError):
    """协议以 ``ERR`` 同步拒绝命令。"""


class STM32CommandEventError(STM32MotionError):
    """已接受的非阻塞命令最终 ABORT 或 FAULT。"""

    def __init__(self, event: "STM32Message") -> None:
        super().__init__(
            f"STM32 command {event.sequence} ended with "
            f"{event.kind} {' '.join(event.arguments)}"
        )
        self.event = event


class MotionAbortedError(STM32CommandEventError):
    """已接受的运动、归零或吸盘命令收到 ``ABORT``。"""

    def __init__(self, event: "STM32Message") -> None:
        super().__init__(event)
        self.target = event.arguments[0]
        self.value = int(event.arguments[1], 10)


class MotionFaultError(STM32CommandEventError):
    """已接受的运动、归零或吸盘命令收到 ``FAULT``。"""

    def __init__(self, event: "STM32Message") -> None:
        super().__init__(event)
        self.target = event.arguments[0]
        self.error_code = int(event.arguments[1], 10)
        self.value = int(event.arguments[2], 10) if len(event.arguments) == 3 else None


class LineTransport(Protocol):
    def write_line(self, line: str) -> None: ...

    def read_line(self) -> str | None: ...

    def close(self) -> None: ...


class ByteSerialPort(Protocol):
    is_open: bool

    def write(self, data: bytes) -> int: ...

    def readline(self) -> bytes: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class STM32SerialConfig:
    port: str
    baudrate: int
    timeout: float = 0.1
    write_timeout: float = 0.1

    def __post_init__(self) -> None:
        if not self.port.strip():
            raise STM32MotionConfigurationError("serial port must not be empty")
        if self.baudrate <= 0:
            raise STM32MotionConfigurationError("baudrate must be positive")
        if self.timeout <= 0 or self.write_timeout <= 0:
            raise STM32MotionConfigurationError("timeouts must be positive")


class STM32SerialTransport:
    """可注入 fake 的 pyserial 行传输；构造时不访问硬件。"""

    def __init__(
        self,
        config: STM32SerialConfig,
        *,
        serial_port: ByteSerialPort | None = None,
    ) -> None:
        self.config = config
        self._serial = serial_port

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        try:
            import serial

            self._serial = serial.Serial(
                self.config.port,
                self.config.baudrate,
                timeout=self.config.timeout,
                write_timeout=self.config.write_timeout,
            )
        except Exception as exc:
            self._serial = None
            raise STM32MotionError(
                f"failed to open STM32 serial port {self.config.port!r}: {exc}"
            ) from exc

    def close(self) -> None:
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            serial_port.close()

    def __enter__(self) -> "STM32SerialTransport":
        self.open()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def _require_serial(self) -> ByteSerialPort:
        if not self.is_open or self._serial is None:
            raise STM32MotionError("STM32 serial port is not open")
        return self._serial

    def write_line(self, line: str) -> None:
        encoded = (line + "\n").encode("ascii")
        serial_port = self._require_serial()
        count = serial_port.write(encoded)
        if count != len(encoded):
            raise STM32MotionError(f"short serial write: {count}/{len(encoded)} bytes")
        serial_port.flush()

    def read_line(self) -> str | None:
        raw = self._require_serial().readline()
        if not raw:
            return None
        try:
            return raw.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise STM32MotionProtocolError("machine line is not ASCII") from exc


@dataclass(frozen=True)
class STM32Message:
    channel: str
    sequence: int
    kind: str
    arguments: tuple[str, ...]


@dataclass(frozen=True)
class STM32CommandSubmission:
    """An accepted non-blocking command tracked by protocol sequence."""

    sequence: int
    axis: str
    command: str


@dataclass(frozen=True)
class AxisStatus:
    axis: Axis
    hardware_ready: bool
    enabled: bool
    busy: bool
    homed: bool
    position_valid: bool
    position_um: int
    fault: AxisFault | int

    @property
    def configured(self) -> bool:
        """兼容旧调用者；新代码应使用 ``hardware_ready``。"""

        warnings.warn(
            "AxisStatus.configured is deprecated; use hardware_ready",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.hardware_ready

    @property
    def driver_configured(self) -> bool:
        """兼容早期驱动器命名；不表示外置驱动器状态。"""

        warnings.warn(
            "AxisStatus.driver_configured is deprecated; use hardware_ready",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.hardware_ready


@dataclass(frozen=True)
class HomingStatus:
    active: bool
    axis: Axis | None
    state: int
    error: int


HomeStatus = HomingStatus


@dataclass(frozen=True)
class MotionResult:
    axis: Axis
    position_um: int


@dataclass(frozen=True)
class SuctionStatus:
    state: int
    pump_on: bool
    release_open: bool
    busy: bool
    fault: int


@dataclass(frozen=True)
class VersionInfo:
    protocol_version: str
    firmware_version: str


@dataclass(frozen=True)
class ConnectionSnapshot:
    version: VersionInfo
    z: AxisStatus
    slide: AxisStatus
    homing: HomingStatus
    suction: SuctionStatus


class ProtocolParser:
    """严格解析 v2 机器行，并把其他行转发为日志。"""

    def __init__(self, on_log_line: Callable[[str], None] | None = None) -> None:
        self.on_log_line = on_log_line

    def parse(self, line: str) -> STM32Message | None:
        stripped = line.strip()
        if not stripped or stripped[0] not in "=!":
            if stripped and self.on_log_line is not None:
                self.on_log_line(stripped)
            return None

        fields = stripped.split()
        if len(fields) < 2 or len(fields[0]) < 2:
            raise ProtocolFrameError(f"malformed machine line: {line!r}")
        channel = fields[0][0]
        sequence_text = fields[0][1:]
        if not sequence_text.isascii() or not sequence_text.isdecimal():
            raise ProtocolFrameError(f"invalid sequence in line: {line!r}")
        sequence = int(sequence_text, 10)
        if not 0 <= sequence <= 0xFFFF:
            raise ProtocolFrameError(f"sequence out of range: {sequence}")

        message = STM32Message(channel, sequence, fields[1], tuple(fields[2:]))
        self._validate(message)
        return message

    @staticmethod
    def _require_count(message: STM32Message, count: int) -> None:
        if len(message.arguments) != count:
            raise ProtocolFrameError(
                f"{message.channel}{message.sequence} {message.kind} must contain "
                f"{count} fields, got {len(message.arguments)}"
            )

    @staticmethod
    def _require_int(value: str, name: str) -> int:
        try:
            return int(value, 10)
        except ValueError as exc:
            raise ProtocolFrameError(f"{name} must be an integer, got {value!r}") from exc

    @staticmethod
    def _require_bool(value: str, name: str) -> None:
        if value not in ("0", "1"):
            raise ProtocolFrameError(f"{name} must be 0 or 1, got {value!r}")

    @staticmethod
    def _require_axis(value: str) -> None:
        if value not in (Axis.Z.value, Axis.SLIDE.value):
            raise ProtocolFrameError(f"axis must be Z or S, got {value!r}")

    def _validate(self, message: STM32Message) -> None:
        args = message.arguments
        if message.channel == "=":
            if message.kind == "OK":
                self._require_count(message, 0)
            elif message.kind == "ERR":
                self._require_count(message, 1)
                self._require_int(args[0], "protocol_error")
            elif message.kind == "VR":
                self._require_count(message, 2)
            elif message.kind == "ST":
                self._require_count(message, 8)
                self._require_axis(args[0])
                for index, name in enumerate(
                    ("hardware_ready", "enabled", "busy", "homed", "position_valid"),
                    start=1,
                ):
                    self._require_bool(args[index], name)
                self._require_int(args[6], "position_um")
                self._require_int(args[7], "fault")
            elif message.kind == "HS":
                self._require_count(message, 4)
                self._require_bool(args[0], "active")
                self._require_axis(args[1])
                self._require_int(args[2], "state")
                self._require_int(args[3], "error")
            elif message.kind == "SS":
                self._require_count(message, 5)
                self._require_int(args[0], "state")
                self._require_bool(args[1], "pump_on")
                self._require_bool(args[2], "release_open")
                self._require_bool(args[3], "busy")
                self._require_int(args[4], "fault")
            else:
                raise ProtocolFrameError(f"unknown synchronous response {message.kind!r}")
            return

        if message.channel != "!":
            raise ProtocolFrameError(f"unknown protocol channel {message.channel!r}")
        if message.kind in ("DONE", "ABORT"):
            self._require_count(message, 2)
            if args[0] not in (Axis.Z.value, Axis.SLIDE.value, "V"):
                raise ProtocolFrameError(f"invalid event target {args[0]!r}")
            self._require_int(args[1], "event_value")
            return
        if message.kind == "FAULT":
            if not args:
                self._require_count(message, 1)
            target = args[0]
            if target in (Axis.Z.value, Axis.SLIDE.value):
                self._require_count(message, 3)
                self._require_int(args[1], "protocol_error")
                self._require_int(args[2], "position_um")
            elif target == "V":
                self._require_count(message, 2)
                self._require_int(args[1], "protocol_error")
            else:
                raise ProtocolFrameError(f"invalid event target {target!r}")
            return
        raise ProtocolFrameError(f"unknown asynchronous event {message.kind!r}")


def parse_machine_line(line: str) -> STM32Message | None:
    """解析机器行；普通调试日志返回 ``None``。"""

    return ProtocolParser().parse(line)


def _parse_bool(value: str, field_name: str) -> bool:
    if value not in ("0", "1"):
        raise STM32MotionProtocolError(f"{field_name} must be 0 or 1, got {value!r}")
    return value == "1"


def _parse_int(value: str, field_name: str) -> int:
    try:
        return int(value, 10)
    except ValueError as exc:
        raise STM32MotionProtocolError(
            f"{field_name} must be an integer, got {value!r}"
        ) from exc


def _axis_code(axis: str | Axis) -> str:
    if isinstance(axis, Axis):
        return axis.value
    try:
        return AXIS_CODES[axis]
    except KeyError as exc:
        raise STM32MotionConfigurationError("axis must be z/Z or slide/S") from exc


def _integer(
    value: int,
    name: str,
    *,
    minimum: int = INT32_MIN,
    maximum: int = INT32_MAX,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise STM32MotionConfigurationError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise STM32MotionConfigurationError(
            f"{name} must be in {minimum}..{maximum}, got {value}"
        )
    return value


def _positive_uint32(value: int, name: str) -> int:
    return _integer(value, name, minimum=1, maximum=UINT32_MAX)


def _axis_fault(value: str) -> AxisFault | int:
    parsed = _parse_int(value, "fault")
    try:
        return AxisFault(parsed)
    except ValueError:
        return parsed


def _mm_to_um(value: float, name: str, *, unsigned: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise STM32MotionConfigurationError(f"{name} must be a real number")
    if not math.isfinite(value):
        raise STM32MotionConfigurationError(f"{name} must be finite")
    converted = round(value * 1000.0)
    minimum, maximum = (1, UINT32_MAX) if unsigned else (INT32_MIN, INT32_MAX)
    if not minimum <= converted <= maximum:
        raise STM32MotionConfigurationError(
            f"{name} converts to {converted} outside {minimum}..{maximum}"
        )
    return converted


class STM32MotionClient:
    """sequence 感知的同步响应/异步事件客户端。"""

    def __init__(
        self,
        transport: LineTransport,
        *,
        first_sequence: int = 0,
        on_log_line: Callable[[str], None] | None = None,
    ) -> None:
        if not 0 <= first_sequence <= 0xFFFF:
            raise STM32MotionConfigurationError("first_sequence must be in 0..65535")
        self.transport = transport
        self._sequence = first_sequence
        self._parser = ProtocolParser(on_log_line)
        self._pending_messages: list[STM32Message] = []
        self._pending_sync: dict[int, str] = {}
        self._pending_async: dict[int, STM32CommandSubmission] = {}
        self._disconnected_reason: str | None = None

    def close(self) -> None:
        self.disconnect("client closed")

    def disconnect(self, reason: str = "serial transport disconnected") -> None:
        """Invalidate all local pending state without assuming device motion stopped."""

        self._disconnected_reason = reason
        self._pending_sync.clear()
        self._pending_async.clear()
        self._pending_messages.clear()
        self.transport.close()

    @property
    def pending_sync_sequences(self) -> frozenset[int]:
        return frozenset(self._pending_sync)

    @property
    def pending_async_sequences(self) -> frozenset[int]:
        return frozenset(self._pending_async)

    def _require_connected(self) -> None:
        if self._disconnected_reason is not None:
            raise ProtocolDisconnectedError(self._disconnected_reason)

    def _mark_disconnected(self, exc: Exception) -> ProtocolDisconnectedError:
        reason = f"STM32 serial transport disconnected: {exc}"
        try:
            self.disconnect(reason)
        except Exception:
            self._disconnected_reason = reason
            self._pending_sync.clear()
            self._pending_async.clear()
            self._pending_messages.clear()
        return ProtocolDisconnectedError(reason)

    def _next_sequence(self) -> int:
        buffered_sequences = {message.sequence for message in self._pending_messages}
        for _ in range(0x10000):
            result = self._sequence
            self._sequence = (self._sequence + 1) & 0xFFFF
            if (
                result not in self._pending_sync
                and result not in self._pending_async
                and result not in buffered_sequences
            ):
                return result
        raise STM32MotionConfigurationError("all protocol sequences are pending")

    def _send(self, command: str) -> int:
        self._require_connected()
        sequence = self._next_sequence()
        frame = f"@{sequence} {command}"
        if len(frame.encode("ascii")) > MAX_LINE_LENGTH:
            raise STM32MotionConfigurationError("command exceeds protocol line limit")
        self._pending_sync[sequence] = command
        try:
            self.transport.write_line(frame)
        except Exception as exc:
            self._pending_sync.pop(sequence, None)
            raise self._mark_disconnected(exc) from exc
        return sequence

    def _take_buffered(self, sequence: int, channel: str) -> STM32Message | None:
        for index, message in enumerate(self._pending_messages):
            if message.sequence == sequence and message.channel == channel:
                return self._pending_messages.pop(index)
        return None

    def _read_message(self) -> STM32Message | None:
        self._require_connected()
        try:
            line = self.transport.read_line()
        except Exception as exc:
            raise self._mark_disconnected(exc) from exc
        return None if line is None else self._parser.parse(line)

    def _wait(self, sequence: int, channel: str, timeout: float) -> STM32Message:
        if not math.isfinite(timeout) or timeout <= 0:
            raise STM32MotionConfigurationError("timeout must be finite and positive")
        buffered = self._take_buffered(sequence, channel)
        if buffered is not None:
            return buffered
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._read_message()
            if message is None:
                continue
            if message.sequence == sequence and message.channel == channel:
                return message
            self._pending_messages.append(message)
        raise ProtocolTimeoutError(
            f"timed out waiting for {channel}{sequence} after {timeout:.3f}s"
        )

    def _sync(self, command: str, timeout: float = 2.0) -> STM32Message:
        sequence = self._send(command)
        response = self._wait(sequence, "=", timeout)
        self._pending_sync.pop(sequence, None)
        if response.kind == "ERR":
            raise ProtocolCommandError(
                sequence, _parse_int(response.arguments[0], "protocol_error")
            )
        return response

    def _nonblocking(
        self,
        command: str,
        target: str,
        *,
        sync_timeout: float,
        event_timeout: float,
    ) -> STM32Message:
        submission = self._submit_nonblocking(command, target, sync_timeout)
        event = self.wait_for_command(submission, timeout=event_timeout)
        if event.kind == "ABORT":
            raise MotionAbortedError(event)
        if event.kind == "FAULT":
            raise MotionFaultError(event)
        return event

    def _submit_nonblocking(
        self,
        command: str,
        axis: str,
        sync_timeout: float,
    ) -> STM32CommandSubmission:
        sequence = self._send(command)
        response = self._wait(sequence, "=", sync_timeout)
        self._pending_sync.pop(sequence, None)
        if response.kind == "ERR":
            raise ProtocolCommandError(
                sequence, _parse_int(response.arguments[0], "protocol_error")
            )
        if response.kind != "OK" or response.arguments:
            raise ProtocolFrameError("motion acceptance must be '=seq OK'")
        submission = STM32CommandSubmission(
            sequence, axis, command.split(maxsplit=1)[0]
        )
        self._pending_async[sequence] = submission
        return submission

    def _complete_async(
        self,
        submission: STM32CommandSubmission,
        event: STM32Message,
    ) -> STM32Message:
        tracked = self._pending_async.get(submission.sequence)
        if tracked != submission:
            raise ProtocolFrameError(
                f"sequence {submission.sequence} is not pending for this submission"
            )
        if event.arguments[0] != submission.axis:
            raise ProtocolFrameError(
                f"event target {event.arguments[0]!r} does not match "
                f"submission target {submission.axis!r}"
            )
        self._pending_async.pop(submission.sequence, None)
        return event

    def poll_command(
        self,
        submission: STM32CommandSubmission,
    ) -> STM32Message | None:
        """Return the command event if already available, otherwise poll once."""

        if self._pending_async.get(submission.sequence) != submission:
            raise ProtocolFrameError(
                f"sequence {submission.sequence} is not pending for this submission"
            )
        message = self._take_buffered(submission.sequence, "!")
        if message is not None:
            return self._complete_async(submission, message)
        message = self._read_message()
        if message is None:
            return None
        if message.sequence == submission.sequence and message.channel == "!":
            return self._complete_async(submission, message)
        self._pending_messages.append(message)
        return None

    def wait_for_command(
        self,
        submission: STM32CommandSubmission,
        *,
        timeout: float = 120.0,
    ) -> STM32Message:
        """Wait for and return DONE, ABORT, or FAULT for a submission."""

        if self._pending_async.get(submission.sequence) != submission:
            raise ProtocolFrameError(
                f"sequence {submission.sequence} is not pending for this submission"
            )
        event = self._wait(submission.sequence, "!", timeout)
        return self._complete_async(submission, event)

    def query_axis(self, axis: str | Axis, timeout: float = 2.0) -> AxisStatus:
        response = self._sync(f"QS {_axis_code(axis)}", timeout)
        if response.kind != "ST":
            raise ProtocolFrameError("QS response must be ST")
        values = response.arguments
        return AxisStatus(
            axis=Axis(values[0]),
            hardware_ready=_parse_bool(values[1], "hardware_ready"),
            enabled=_parse_bool(values[2], "enabled"),
            busy=_parse_bool(values[3], "busy"),
            homed=_parse_bool(values[4], "homed"),
            position_valid=_parse_bool(values[5], "position_valid"),
            position_um=_parse_int(values[6], "position_um"),
            fault=_axis_fault(values[7]),
        )

    def get_status(self, axis: str | Axis, timeout: float = 2.0) -> AxisStatus:
        return self.query_axis(axis, timeout)

    def query_home(self, timeout: float = 2.0) -> HomingStatus:
        response = self._sync("QH", timeout)
        if response.kind != "HS":
            raise ProtocolFrameError("QH response must be HS")
        values = response.arguments
        return HomingStatus(
            active=_parse_bool(values[0], "active"),
            axis=Axis(values[1]),
            state=_parse_int(values[2], "state"),
            error=_parse_int(values[3], "error"),
        )

    def query_suction(self, timeout: float = 2.0) -> SuctionStatus:
        response = self._sync("SQ", timeout)
        if response.kind != "SS":
            raise ProtocolFrameError("SQ response must be SS")
        values = response.arguments
        return SuctionStatus(
            state=_parse_int(values[0], "state"),
            pump_on=_parse_bool(values[1], "pump_on"),
            release_open=_parse_bool(values[2], "release_open"),
            busy=_parse_bool(values[3], "busy"),
            fault=_parse_int(values[4], "fault"),
        )

    def get_suction_status(self, timeout: float = 2.0) -> SuctionStatus:
        return self.query_suction(timeout)

    def version(self, timeout: float = 2.0) -> VersionInfo:
        response = self._sync("VR", timeout)
        if response.kind != "VR":
            raise ProtocolFrameError("VR response must be VR")
        return VersionInfo(*response.arguments)

    def resynchronize(self, timeout: float = 2.0) -> ConnectionSnapshot:
        """Rebuild Host state after connect/reconnect using only read-only commands."""

        version = self.version(timeout)
        if version.protocol_version != PROTOCOL_VERSION:
            raise STM32MotionProtocolError(
                f"expected protocol v{PROTOCOL_VERSION}, got v{version.protocol_version}"
            )
        return ConnectionSnapshot(
            version=version,
            z=self.query_axis(Axis.Z, timeout),
            slide=self.query_axis(Axis.SLIDE, timeout),
            homing=self.query_home(timeout),
            suction=self.query_suction(timeout),
        )

    def reconnect(self, timeout: float = 2.0) -> ConnectionSnapshot:
        """Reopen a transport and rebuild state without using stale position caches."""

        self._pending_sync.clear()
        self._pending_async.clear()
        self._pending_messages.clear()
        try:
            self.transport.close()
            opener = getattr(self.transport, "open", None)
            if not callable(opener):
                raise STM32MotionConfigurationError("transport does not support reconnect")
            opener()
        except Exception as exc:
            raise self._mark_disconnected(exc) from exc
        self._disconnected_reason = None
        return self.resynchronize(timeout)

    def move_relative(
        self,
        axis: str,
        distance_um: int,
        speed_um_s: int,
        acceleration_um_s2: int,
        *,
        sync_timeout: float = 2.0,
        event_timeout: float = 120.0,
    ) -> STM32Message:
        command = (
            f"MR {_axis_code(axis)} {_integer(distance_um, 'distance_um')} "
            f"{_positive_uint32(speed_um_s, 'speed_um_s')} "
            f"{_positive_uint32(acceleration_um_s2, 'acceleration_um_s2')}"
        )
        return self._nonblocking(
            command,
            _axis_code(axis),
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

    def move_relative_mm(
        self,
        axis: str | Axis,
        distance_mm: float,
        speed_mm_s: float,
        acceleration_mm_s2: float,
        *,
        sync_timeout: float = 2.0,
        event_timeout: float = 120.0,
    ) -> STM32Message:
        return self.move_relative(
            axis,
            _mm_to_um(distance_mm, "distance_mm"),
            _mm_to_um(speed_mm_s, "speed_mm_s", unsigned=True),
            _mm_to_um(acceleration_mm_s2, "acceleration_mm_s2", unsigned=True),
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

    def move_absolute(
        self,
        axis: str,
        position_um: int,
        speed_um_s: int,
        acceleration_um_s2: int,
        *,
        sync_timeout: float = 2.0,
        event_timeout: float = 120.0,
    ) -> STM32Message:
        command = (
            f"MA {_axis_code(axis)} {_integer(position_um, 'position_um')} "
            f"{_positive_uint32(speed_um_s, 'speed_um_s')} "
            f"{_positive_uint32(acceleration_um_s2, 'acceleration_um_s2')}"
        )
        return self._nonblocking(
            command,
            _axis_code(axis),
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

    def move_absolute_mm(
        self,
        axis: str | Axis,
        position_mm: float,
        speed_mm_s: float,
        acceleration_mm_s2: float,
        *,
        sync_timeout: float = 2.0,
        event_timeout: float = 120.0,
    ) -> STM32Message:
        return self.move_absolute(
            axis,
            _mm_to_um(position_mm, "position_mm"),
            _mm_to_um(speed_mm_s, "speed_mm_s", unsigned=True),
            _mm_to_um(acceleration_mm_s2, "acceleration_mm_s2", unsigned=True),
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

    def submit_move_absolute(
        self,
        axis: str,
        position_um: int,
        speed_um_s: int,
        acceleration_um_s2: int,
        *,
        sync_timeout: float = 2.0,
    ) -> STM32CommandSubmission:
        """Submit an absolute move and return after the synchronous OK."""

        axis_code = _axis_code(axis)
        command = (
            f"MA {axis_code} {_integer(position_um, 'position_um')} "
            f"{_positive_uint32(speed_um_s, 'speed_um_s')} "
            f"{_positive_uint32(acceleration_um_s2, 'acceleration_um_s2')}"
        )
        return self._submit_nonblocking(command, axis_code, sync_timeout)

    def submit_move_relative(
        self,
        axis: str,
        distance_um: int,
        speed_um_s: int,
        acceleration_um_s2: int,
        *,
        sync_timeout: float = 2.0,
    ) -> STM32CommandSubmission:
        """Submit a relative move and return after the synchronous OK."""

        axis_code = _axis_code(axis)
        command = (
            f"MR {axis_code} {_integer(distance_um, 'distance_um')} "
            f"{_positive_uint32(speed_um_s, 'speed_um_s')} "
            f"{_positive_uint32(acceleration_um_s2, 'acceleration_um_s2')}"
        )
        return self._submit_nonblocking(command, axis_code, sync_timeout)

    def home(
        self, axis: str, *, sync_timeout: float = 2.0, event_timeout: float = 60.0
    ) -> STM32Message:
        axis_code = _axis_code(axis)
        return self._nonblocking(
            f"HM {axis_code}",
            axis_code,
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

    def submit_home(
        self,
        axis: str,
        *,
        sync_timeout: float = 2.0,
    ) -> STM32CommandSubmission:
        """Submit homing and return after the synchronous OK."""

        axis_code = _axis_code(axis)
        return self._submit_nonblocking(f"HM {axis_code}", axis_code, sync_timeout)

    def stop(self, axis: str, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync(f"ST {_axis_code(axis)}", timeout))

    def disable(self, axis: str, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync(f"DI {_axis_code(axis)}", timeout))

    def enable(self, axis: str, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync(f"EN {_axis_code(axis)}", timeout))

    def stop_all(self, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync("SA", timeout))

    def clear_fault(self, axis: str, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync(f"CF {_axis_code(axis)}", timeout))

    def suction_start(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        return self._nonblocking(
            "SU", "V", sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def suction_release(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        return self._nonblocking(
            "SR", "V", sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def suction_stop(self, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync("SX", timeout))

    def suction_grip(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        """执行 SU：泵开启、释放阀关闭，并等待现有 DONE/FAULT。"""

        return self.suction_start(
            sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def suction_idle(self, timeout: float = 2.0) -> None:
        """执行 SX：泵关闭、释放阀关闭，并等待 OK。"""

        self.suction_stop(timeout)

    def suction(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        return self.suction_start(
            sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def release(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        return self.suction_release(
            sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def stop_suction(self, timeout: float = 2.0) -> None:
        self.suction_stop(timeout)

    @staticmethod
    def _expect_ok(response: STM32Message) -> None:
        if response.kind != "OK" or response.arguments:
            raise ProtocolFrameError("command response must be '=seq OK'")

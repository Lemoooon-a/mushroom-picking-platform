"""STM32 motion protocol v1 的正式主机侧客户端。

协议真值位于 ``firmware/stm32_motion_controller/App/Inc/app_protocol.h``
和同目录 ``README.md``。本模块只在显式 ``open()`` 后打开串口。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol


PROTOCOL_VERSION = "1"
MAX_LINE_LENGTH = 96
AXIS_CODES = {"z": "Z", "slide": "S", "Z": "Z", "S": "S"}


class STM32MotionError(Exception):
    """STM32 motion 客户端基础异常。"""


class STM32MotionConfigurationError(STM32MotionError):
    """串口或命令参数无效。"""


class STM32MotionTimeoutError(STM32MotionError):
    """等待同步响应或异步事件超时。"""


class STM32MotionProtocolError(STM32MotionError):
    """收到不符合协议 v1 的机器行。"""


class STM32CommandError(STM32MotionError):
    """同步响应明确拒绝命令。"""

    def __init__(self, sequence: int, error_code: int) -> None:
        super().__init__(f"STM32 command {sequence} failed with error {error_code}")
        self.sequence = sequence
        self.error_code = error_code


class STM32CommandEventError(STM32MotionError):
    """已接受的非阻塞命令最终 ABORT 或 FAULT。"""

    def __init__(self, event: "STM32Message") -> None:
        super().__init__(
            f"STM32 command {event.sequence} ended with "
            f"{event.kind} {' '.join(event.arguments)}"
        )
        self.event = event


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
class AxisStatus:
    axis: str
    configured: bool
    enabled: bool
    busy: bool
    homed: bool
    position_valid: bool
    position_um: int
    fault: int


@dataclass(frozen=True)
class HomeStatus:
    active: bool
    axis: str
    state: int
    error: int


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


def parse_machine_line(line: str) -> STM32Message | None:
    """解析机器行；普通调试日志返回 ``None``。"""

    stripped = line.strip()
    if not stripped or stripped[0] not in "=!":
        return None
    fields = stripped.split()
    if len(fields) < 2 or len(fields[0]) < 2:
        raise STM32MotionProtocolError(f"malformed machine line: {line!r}")
    channel = fields[0][0]
    try:
        sequence = int(fields[0][1:], 10)
    except ValueError as exc:
        raise STM32MotionProtocolError(f"invalid sequence in line: {line!r}") from exc
    if not 0 <= sequence <= 0xFFFF:
        raise STM32MotionProtocolError(f"sequence out of range: {sequence}")
    return STM32Message(channel, sequence, fields[1], tuple(fields[2:]))


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


def _axis_code(axis: str) -> str:
    try:
        return AXIS_CODES[axis]
    except KeyError as exc:
        raise STM32MotionConfigurationError("axis must be z/Z or slide/S") from exc


def _integer(value: int, name: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise STM32MotionConfigurationError(f"{name} must be an integer")
    if positive and value <= 0:
        raise STM32MotionConfigurationError(f"{name} must be positive")
    return value


class STM32MotionClient:
    """sequence 感知的同步响应/异步事件客户端。"""

    def __init__(self, transport: LineTransport, *, first_sequence: int = 0) -> None:
        if not 0 <= first_sequence <= 0xFFFF:
            raise STM32MotionConfigurationError("first_sequence must be in 0..65535")
        self.transport = transport
        self._sequence = first_sequence
        self._pending: list[STM32Message] = []

    def close(self) -> None:
        self.transport.close()

    def _next_sequence(self) -> int:
        result = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        return result

    def _send(self, command: str) -> int:
        sequence = self._next_sequence()
        frame = f"@{sequence} {command}"
        if len(frame.encode("ascii")) > MAX_LINE_LENGTH:
            raise STM32MotionConfigurationError("command exceeds protocol line limit")
        self.transport.write_line(frame)
        return sequence

    def _wait(self, sequence: int, channel: str, timeout: float) -> STM32Message:
        if not math.isfinite(timeout) or timeout <= 0:
            raise STM32MotionConfigurationError("timeout must be finite and positive")
        for index, message in enumerate(self._pending):
            if message.sequence == sequence and message.channel == channel:
                return self._pending.pop(index)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.transport.read_line()
            if line is None:
                continue
            message = parse_machine_line(line)
            if message is None:
                continue
            if message.sequence == sequence and message.channel == channel:
                return message
            self._pending.append(message)
        raise STM32MotionTimeoutError(
            f"timed out waiting for {channel}{sequence} after {timeout:.3f}s"
        )

    def _sync(self, command: str, timeout: float = 2.0) -> STM32Message:
        sequence = self._send(command)
        response = self._wait(sequence, "=", timeout)
        if response.kind == "ERR":
            if len(response.arguments) != 1:
                raise STM32MotionProtocolError("ERR response must contain one code")
            raise STM32CommandError(sequence, _parse_int(response.arguments[0], "error"))
        return response

    def _nonblocking(
        self, command: str, *, sync_timeout: float, event_timeout: float
    ) -> STM32Message:
        sequence = self._send(command)
        response = self._wait(sequence, "=", sync_timeout)
        if response.kind == "ERR":
            if len(response.arguments) != 1:
                raise STM32MotionProtocolError("ERR response must contain one code")
            raise STM32CommandError(sequence, _parse_int(response.arguments[0], "error"))
        if response.kind != "OK" or response.arguments:
            raise STM32MotionProtocolError("motion acceptance must be '=seq OK'")
        event = self._wait(sequence, "!", event_timeout)
        if event.kind != "DONE":
            raise STM32CommandEventError(event)
        return event

    def query_axis(self, axis: str, timeout: float = 2.0) -> AxisStatus:
        response = self._sync(f"QS {_axis_code(axis)}", timeout)
        if response.kind != "ST" or len(response.arguments) != 8:
            raise STM32MotionProtocolError("QS response must contain ST plus 8 fields")
        values = response.arguments
        return AxisStatus(
            axis=values[0],
            configured=_parse_bool(values[1], "configured"),
            enabled=_parse_bool(values[2], "enabled"),
            busy=_parse_bool(values[3], "busy"),
            homed=_parse_bool(values[4], "homed"),
            position_valid=_parse_bool(values[5], "valid"),
            position_um=_parse_int(values[6], "position_um"),
            fault=_parse_int(values[7], "fault"),
        )

    def query_home(self, timeout: float = 2.0) -> HomeStatus:
        response = self._sync("QH", timeout)
        if response.kind != "HS" or len(response.arguments) != 4:
            raise STM32MotionProtocolError("QH response must contain HS plus 4 fields")
        values = response.arguments
        return HomeStatus(
            active=_parse_bool(values[0], "active"),
            axis=values[1],
            state=_parse_int(values[2], "state"),
            error=_parse_int(values[3], "error"),
        )

    def query_suction(self, timeout: float = 2.0) -> SuctionStatus:
        response = self._sync("SQ", timeout)
        if response.kind != "SS" or len(response.arguments) != 5:
            raise STM32MotionProtocolError("SQ response must contain SS plus 5 fields")
        values = response.arguments
        return SuctionStatus(
            state=_parse_int(values[0], "state"),
            pump_on=_parse_bool(values[1], "pump_on"),
            release_open=_parse_bool(values[2], "release_open"),
            busy=_parse_bool(values[3], "busy"),
            fault=_parse_int(values[4], "fault"),
        )

    def version(self, timeout: float = 2.0) -> VersionInfo:
        response = self._sync("VR", timeout)
        if response.kind != "VR" or len(response.arguments) != 2:
            raise STM32MotionProtocolError("VR response must contain two versions")
        return VersionInfo(*response.arguments)

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
            f"{_integer(speed_um_s, 'speed_um_s', positive=True)} "
            f"{_integer(acceleration_um_s2, 'acceleration_um_s2', positive=True)}"
        )
        return self._nonblocking(
            command, sync_timeout=sync_timeout, event_timeout=event_timeout
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
            f"{_integer(speed_um_s, 'speed_um_s', positive=True)} "
            f"{_integer(acceleration_um_s2, 'acceleration_um_s2', positive=True)}"
        )
        return self._nonblocking(
            command, sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def home(
        self, axis: str, *, sync_timeout: float = 2.0, event_timeout: float = 60.0
    ) -> STM32Message:
        return self._nonblocking(
            f"HM {_axis_code(axis)}",
            sync_timeout=sync_timeout,
            event_timeout=event_timeout,
        )

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
            "SU", sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def suction_release(
        self, *, sync_timeout: float = 2.0, event_timeout: float = 2.0
    ) -> STM32Message:
        return self._nonblocking(
            "SR", sync_timeout=sync_timeout, event_timeout=event_timeout
        )

    def suction_stop(self, timeout: float = 2.0) -> None:
        self._expect_ok(self._sync("SX", timeout))

    @staticmethod
    def _expect_ok(response: STM32Message) -> None:
        if response.kind != "OK" or response.arguments:
            raise STM32MotionProtocolError("command response must be '=seq OK'")

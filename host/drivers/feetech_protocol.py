"""Feetech 半双工串行总线的最小安全协议实现。

模块只处理协议帧与串口生命周期，不包含具体机构的零点、方向或限位。
串口在显式调用 :meth:`FeetechBus.open` 前不会打开。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Protocol


HEADER = b"\xff\xff"
INSTRUCTION_PING = 0x01
INSTRUCTION_READ = 0x02
INSTRUCTION_WRITE = 0x03
BROADCAST_ID = 0xFE


class FeetechError(Exception):
    """Feetech 驱动基础异常。"""


class FeetechConfigurationError(FeetechError):
    """通信配置或协议参数无效。"""


class FeetechNotOpenError(FeetechError):
    """尚未打开串口。"""


class FeetechTimeoutError(FeetechError):
    """等待状态包超时。"""


class FeetechProtocolError(FeetechError):
    """收到格式、校验和或来源不正确的数据包。"""


class FeetechDeviceError(FeetechError):
    """舵机状态包返回非零错误码。"""

    def __init__(self, servo_id: int, error: int) -> None:
        super().__init__(f"servo ID {servo_id} returned error 0x{error:02X}")
        self.servo_id = servo_id
        self.error = error


class SerialPort(Protocol):
    is_open: bool

    def write(self, data: bytes) -> int: ...

    def read(self, size: int = 1) -> bytes: ...

    def flush(self) -> None: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class FeetechSerialConfig:
    port: str
    baudrate: int
    timeout: float = 0.1
    write_timeout: float = 0.1

    def __post_init__(self) -> None:
        if not self.port.strip():
            raise FeetechConfigurationError("serial port must not be empty")
        if self.baudrate <= 0:
            raise FeetechConfigurationError("baudrate must be greater than zero")
        if self.timeout <= 0 or self.write_timeout <= 0:
            raise FeetechConfigurationError("serial timeouts must be greater than zero")


@dataclass(frozen=True)
class FeetechStatusPacket:
    servo_id: int
    error: int
    parameters: bytes


def _validate_servo_id(servo_id: int, *, allow_broadcast: bool) -> None:
    maximum = BROADCAST_ID if allow_broadcast else BROADCAST_ID - 1
    if not 0 <= servo_id <= maximum:
        raise FeetechConfigurationError(
            f"servo_id must be in 0..{maximum}, got {servo_id}"
        )


def calculate_checksum(payload: bytes) -> int:
    """计算从 ID 到最后一个参数字节的反码校验和。"""

    return (~sum(payload)) & 0xFF


def build_instruction_packet(
    servo_id: int, instruction: int, parameters: bytes = b""
) -> bytes:
    """构建 ``FF FF ID LENGTH INSTRUCTION PARAMS CHECKSUM``。"""

    _validate_servo_id(servo_id, allow_broadcast=True)
    if not 0 <= instruction <= 0xFF:
        raise FeetechConfigurationError("instruction must fit uint8")
    if len(parameters) > 0xFD:
        raise FeetechConfigurationError("instruction parameters are too long")
    body = bytes((servo_id, len(parameters) + 2, instruction)) + parameters
    return HEADER + body + bytes((calculate_checksum(body),))


def parse_status_packet(packet: bytes, *, expected_id: int | None = None) -> FeetechStatusPacket:
    """严格解析完整状态包并验证长度、ID、校验和和设备错误。"""

    if len(packet) < 6 or not packet.startswith(HEADER):
        raise FeetechProtocolError("invalid status packet header or minimum length")
    servo_id = packet[2]
    length = packet[3]
    if length < 2 or len(packet) != length + 4:
        raise FeetechProtocolError(
            f"invalid status packet length: field={length}, bytes={len(packet)}"
        )
    if calculate_checksum(packet[2:-1]) != packet[-1]:
        raise FeetechProtocolError("status packet checksum mismatch")
    if expected_id is not None and servo_id != expected_id:
        raise FeetechProtocolError(
            f"status packet ID {servo_id} does not match expected ID {expected_id}"
        )
    result = FeetechStatusPacket(servo_id, packet[4], packet[5:-1])
    if result.error:
        raise FeetechDeviceError(result.servo_id, result.error)
    return result


class FeetechBus:
    """串行化请求/应答事务并确保串口可显式关闭。"""

    def __init__(
        self,
        config: FeetechSerialConfig,
        *,
        serial_port: SerialPort | None = None,
    ) -> None:
        self.config = config
        self._serial = serial_port
        self._lock = threading.RLock()

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
            raise FeetechError(
                f"failed to open Feetech serial port {self.config.port!r}: {exc}"
            ) from exc

    def close(self) -> None:
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            serial_port.close()

    def __enter__(self) -> "FeetechBus":
        self.open()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def _require_serial(self) -> SerialPort:
        if not self.is_open or self._serial is None:
            raise FeetechNotOpenError("Feetech serial port is not open")
        return self._serial

    def _read_exact(self, size: int, deadline: float) -> bytes:
        serial_port = self._require_serial()
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline:
            chunk = serial_port.read(size - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != size:
            raise FeetechTimeoutError(
                f"status packet timed out after {len(data)}/{size} bytes"
            )
        return bytes(data)

    def _read_status(self, *, expected_id: int) -> FeetechStatusPacket:
        deadline = time.monotonic() + self.config.timeout
        previous = b""
        while time.monotonic() < deadline:
            current = self._read_exact(1, deadline)
            if previous == b"\xff" and current == b"\xff":
                break
            previous = current
        else:
            raise FeetechTimeoutError("status packet header timed out")
        prefix = self._read_exact(2, deadline)
        length = prefix[1]
        if length < 2:
            raise FeetechProtocolError(f"invalid status length field {length}")
        remainder = self._read_exact(length, deadline)
        return parse_status_packet(
            HEADER + prefix + remainder,
            expected_id=expected_id,
        )

    def request(
        self,
        servo_id: int,
        instruction: int,
        parameters: bytes = b"",
        *,
        expect_status: bool = True,
    ) -> FeetechStatusPacket | None:
        if servo_id == BROADCAST_ID and expect_status:
            raise FeetechConfigurationError("broadcast request cannot expect a status")
        packet = build_instruction_packet(servo_id, instruction, parameters)
        with self._lock:
            serial_port = self._require_serial()
            serial_port.reset_input_buffer()
            written = serial_port.write(packet)
            if written != len(packet):
                raise FeetechError(f"short serial write: {written}/{len(packet)} bytes")
            serial_port.flush()
            if not expect_status:
                return None
            return self._read_status(expected_id=servo_id)

    def ping(self, servo_id: int) -> None:
        self.request(servo_id, INSTRUCTION_PING)

    def read_registers(self, servo_id: int, address: int, length: int) -> bytes:
        if not 0 <= address <= 0xFF or not 1 <= length <= 0xFF:
            raise FeetechConfigurationError("register address/length must fit uint8")
        status = self.request(
            servo_id,
            INSTRUCTION_READ,
            bytes((address, length)),
        )
        assert status is not None
        if len(status.parameters) != length:
            raise FeetechProtocolError(
                f"read returned {len(status.parameters)} bytes, expected {length}"
            )
        return status.parameters

    def write_registers(
        self,
        servo_id: int,
        address: int,
        data: bytes,
        *,
        expect_status: bool = True,
    ) -> None:
        if not 0 <= address <= 0xFF or not data:
            raise FeetechConfigurationError("register address must fit uint8 and data is required")
        self.request(
            servo_id,
            INSTRUCTION_WRITE,
            bytes((address,)) + data,
            expect_status=expect_status,
        )

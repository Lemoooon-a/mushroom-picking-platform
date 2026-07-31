"""瓴控 MG4010E-i36 CAN V2.36 协议编码与解析。

本模块只处理协议常量、CAN 标识符、8 字节数据域和协议原生单位。CAN
设备访问、请求事务和机械侧坐标换算由其他模块负责。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct


REQUEST_ID_BASE = 0x140
RESPONSE_ID_BASE = 0x180
MIN_MOTOR_ID = 1
MAX_MOTOR_ID = 32
FRAME_DLC = 8

# V2.36 对 MG 系列转矩电流规定的分辨率。
MG_CURRENT_AMPS_PER_LSB = 66.0 / 4096.0


class Command(IntEnum):
    """首版正式驱动使用的 V2.36 命令字。"""

    STOP = 0x81
    READ_MULTI_TURN_POSITION = 0x92
    READ_SINGLE_TURN_POSITION = 0x94
    READ_FAULT = 0x9A
    READ_STATUS = 0x9C
    POSITION_COMMAND_2 = 0xA4


class MotorError(Exception):
    """MG4010 控制库异常基类。"""


class MotorProtocolError(MotorError):
    """数据域不符合 MG4010 协议。"""


class InvalidDlcError(MotorProtocolError):
    """数据域长度不是协议要求的 8 字节。"""


class CommandMismatchError(MotorProtocolError):
    """应答命令字与所解析的命令不符。"""


@dataclass(frozen=True)
class MotorSingleTurnPosition:
    """0x94 返回的单圈位置，使用电机侧协议原生单位。"""

    circle_angle_raw: int
    motor_cycle_deg: float


@dataclass(frozen=True)
class MotorMultiTurnPosition:
    """0x92 返回的当前上电周期多圈位置。"""

    raw: int
    motor_deg: float


@dataclass(frozen=True)
class MotorStatus:
    """0x9C 返回的温度、转矩电流、速度和编码器计数。"""

    temperature_c: int
    torque_current_raw: int
    torque_current_a: float
    motor_speed_deg_s: int
    encoder_raw: int


@dataclass(frozen=True)
class MotorFault:
    """0x9A 返回的电气量、运行状态和错误状态。"""

    temperature_c: int
    bus_voltage_v: float
    bus_current_a: float
    motor_state: int
    error_state: int


@dataclass(frozen=True)
class PositionCommandResponse:
    """0xA4 通信应答；数据布局与 0x9C 相同。"""

    temperature_c: int
    torque_current_raw: int
    torque_current_a: float
    motor_speed_deg_s: int
    encoder_raw: int


def _validate_motor_id(motor_id: int) -> None:
    if (
        isinstance(motor_id, bool)
        or not isinstance(motor_id, int)
        or not MIN_MOTOR_ID <= motor_id <= MAX_MOTOR_ID
    ):
        raise ValueError(
            f"motor_id must be an integer in {MIN_MOTOR_ID}..{MAX_MOTOR_ID}, "
            f"got {motor_id!r}"
        )


def build_request_id(motor_id: int) -> int:
    """返回命令报文标识符 ``0x140 + motor_id``。"""

    _validate_motor_id(motor_id)
    return REQUEST_ID_BASE + motor_id


def build_response_id(motor_id: int) -> int:
    """返回协议应答标识符 ``0x180 + motor_id``。"""

    _validate_motor_id(motor_id)
    return RESPONSE_ID_BASE + motor_id


def _build_command_request(command: Command) -> bytes:
    data = bytearray(FRAME_DLC)
    data[0] = int(command)
    return bytes(data)


def build_read_single_turn_request() -> bytes:
    """构造 0x94 单圈绝对位置读取数据域。"""

    return _build_command_request(Command.READ_SINGLE_TURN_POSITION)


def build_read_multi_turn_request() -> bytes:
    """构造 0x92 当前多圈位置读取数据域。"""

    return _build_command_request(Command.READ_MULTI_TURN_POSITION)


def build_read_status_request() -> bytes:
    """构造 0x9C 状态 2 读取数据域。"""

    return _build_command_request(Command.READ_STATUS)


def build_read_fault_request() -> bytes:
    """构造 0x9A 状态 1/故障读取数据域。"""

    return _build_command_request(Command.READ_FAULT)


def build_stop_request() -> bytes:
    """构造 0x81 电机停止数据域。"""

    return _build_command_request(Command.STOP)


def build_position_command_2(
    target_motor_deg: float, max_motor_speed_deg_s: float
) -> bytes:
    """构造 0xA4 多圈绝对位置闭环控制命令 2 数据域。

    ``maxSpeed`` 是小端 uint16_t，单位为 1 degree/s；``angleControl``
    是小端 int32_t，单位为 0.01 degree。
    """

    if not math.isfinite(target_motor_deg):
        raise ValueError("target_motor_deg must be finite")
    if not math.isfinite(max_motor_speed_deg_s):
        raise ValueError("max_motor_speed_deg_s must be finite")

    angle_control_raw = round(target_motor_deg * 100)
    max_speed_raw = round(max_motor_speed_deg_s)

    if not -(2**31) <= angle_control_raw <= 2**31 - 1:
        raise ValueError("target_motor_deg is outside the protocol int32_t range")
    if not 1 <= max_speed_raw <= 0xFFFF:
        raise ValueError(
            "max_motor_speed_deg_s is outside the protocol uint16_t range"
        )

    data = bytearray(FRAME_DLC)
    data[0] = int(Command.POSITION_COMMAND_2)
    data[1] = 0
    data[2:4] = struct.pack("<H", max_speed_raw)
    data[4:8] = struct.pack("<i", angle_control_raw)
    return bytes(data)


def _require_response(data: bytes, expected_command: Command) -> None:
    if len(data) != FRAME_DLC:
        raise InvalidDlcError(
            f"expected {FRAME_DLC} data bytes, got {len(data)}"
        )
    if data[0] != int(expected_command):
        raise CommandMismatchError(
            f"expected command 0x{int(expected_command):02X}, "
            f"got 0x{data[0]:02X}"
        )


def parse_single_turn_response(data: bytes) -> MotorSingleTurnPosition:
    """解析 0x94；DATA[4:8] 是 0.01 degree/LSB 的小端 uint32_t。"""

    _require_response(data, Command.READ_SINGLE_TURN_POSITION)
    circle_angle_raw = struct.unpack_from("<I", data, 4)[0]
    return MotorSingleTurnPosition(
        circle_angle_raw=circle_angle_raw,
        motor_cycle_deg=circle_angle_raw * 0.01,
    )


def parse_multi_turn_response(data: bytes) -> MotorMultiTurnPosition:
    """解析 0x92 DATA[1:8] 为有符号 56 位小端二进制补码。

    厂商协议文字将该值声明为 int64_t，但一个 CAN 数据域在命令字之后
    仅剩七个字节。这里保留项目现有、已验证的七字节补码解析，以处理这项
    资料矛盾；并不声称厂商正式定义了 int56 类型。
    """

    _require_response(data, Command.READ_MULTI_TURN_POSITION)
    raw = int.from_bytes(data[1:8], byteorder="little", signed=True)
    return MotorMultiTurnPosition(raw=raw, motor_deg=raw * 0.01)


def _parse_status_layout(
    data: bytes, expected_command: Command
) -> tuple[int, int, float, int, int]:
    _require_response(data, expected_command)
    temperature_c = struct.unpack_from("<b", data, 1)[0]
    torque_current_raw = struct.unpack_from("<h", data, 2)[0]
    motor_speed_deg_s = struct.unpack_from("<h", data, 4)[0]
    encoder_raw = struct.unpack_from("<H", data, 6)[0]
    return (
        temperature_c,
        torque_current_raw,
        torque_current_raw * MG_CURRENT_AMPS_PER_LSB,
        motor_speed_deg_s,
        encoder_raw,
    )


def parse_status_response(data: bytes) -> MotorStatus:
    """解析 0x9C 状态 2 应答。"""

    return MotorStatus(*_parse_status_layout(data, Command.READ_STATUS))


def parse_fault_response(data: bytes) -> MotorFault:
    """解析 0x9A 状态 1/故障应答。"""

    _require_response(data, Command.READ_FAULT)
    temperature_c = struct.unpack_from("<b", data, 1)[0]
    bus_voltage_raw = struct.unpack_from("<h", data, 2)[0]
    bus_current_raw = struct.unpack_from("<h", data, 4)[0]
    return MotorFault(
        temperature_c=temperature_c,
        bus_voltage_v=bus_voltage_raw * 0.01,
        bus_current_a=bus_current_raw * 0.01,
        motor_state=data[6],
        error_state=data[7],
    )


def parse_position_command_2_response(data: bytes) -> PositionCommandResponse:
    """解析 0xA4 通信应答。"""

    return PositionCommandResponse(
        *_parse_status_layout(data, Command.POSITION_COMMAND_2)
    )


__all__ = [
    "Command",
    "CommandMismatchError",
    "FRAME_DLC",
    "InvalidDlcError",
    "MAX_MOTOR_ID",
    "MG_CURRENT_AMPS_PER_LSB",
    "MIN_MOTOR_ID",
    "MotorError",
    "MotorFault",
    "MotorMultiTurnPosition",
    "MotorProtocolError",
    "MotorSingleTurnPosition",
    "MotorStatus",
    "PositionCommandResponse",
    "REQUEST_ID_BASE",
    "RESPONSE_ID_BASE",
    "build_position_command_2",
    "build_read_fault_request",
    "build_read_multi_turn_request",
    "build_read_single_turn_request",
    "build_read_status_request",
    "build_request_id",
    "build_response_id",
    "build_stop_request",
    "parse_fault_response",
    "parse_multi_turn_response",
    "parse_position_command_2_response",
    "parse_single_turn_response",
    "parse_status_response",
]

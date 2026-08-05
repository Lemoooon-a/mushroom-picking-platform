"""基于 Feetech 磁编码协议的有限行程旋转轴。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from drivers.feetech_protocol import FeetechBus, FeetechConfigurationError


class FeetechRotationError(Exception):
    """旋转轴基础异常。"""


class FeetechRotationLimitError(FeetechRotationError):
    """位置或速度超过配置限制。"""


class FeetechRotationPositionError(FeetechRotationError):
    """原始位置无法唯一映射到有限行程。"""


@dataclass(frozen=True)
class FeetechMagneticRegisterMap:
    """Feetech 磁编码协议寄存器；具体型号仍需按数据手册复核。"""

    torque_enable: int = 0x28
    acceleration: int = 0x29
    goal_position: int = 0x2A
    goal_time: int = 0x2C
    goal_speed: int = 0x2E
    present_position: int = 0x38
    feedback_length: int = 10


@dataclass(frozen=True)
class FeetechRotationConfig:
    name: str
    servo_id: int
    counts_per_turn: int
    zero_raw: int
    direction_sign: int
    min_position_rad: float
    max_position_rad: float
    max_speed_raw: int
    expect_write_status: bool = True
    registers: FeetechMagneticRegisterMap = field(
        default_factory=FeetechMagneticRegisterMap
    )

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise FeetechConfigurationError("rotation name must not be empty")
        if not 0 <= self.servo_id <= 0xFD:
            raise FeetechConfigurationError("servo_id must be in 0..253")
        if not 2 <= self.counts_per_turn <= 0x10000:
            raise FeetechConfigurationError("counts_per_turn must be in 2..65536")
        if not 0 <= self.zero_raw < self.counts_per_turn:
            raise FeetechConfigurationError("zero_raw must be within one turn")
        if self.direction_sign not in (-1, 1):
            raise FeetechConfigurationError("direction_sign must be -1 or +1")
        if not all(
            math.isfinite(value)
            for value in (self.min_position_rad, self.max_position_rad)
        ):
            raise FeetechConfigurationError("rotation limits must be finite")
        if self.min_position_rad >= self.max_position_rad:
            raise FeetechConfigurationError("minimum limit must be below maximum")
        if self.max_position_rad - self.min_position_rad >= math.tau:
            raise FeetechConfigurationError("finite travel width must be less than 2*pi")
        if not 1 <= self.max_speed_raw <= 0xFFFF:
            raise FeetechConfigurationError("max_speed_raw must be in 1..65535")


@dataclass(frozen=True)
class FeetechRotationFeedback:
    position_raw: int
    position_rad: float
    speed_raw: int
    load_raw: int
    voltage_raw: int
    temperature_c: int
    moving: bool
    error_raw: int


def _uint16_le(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise FeetechRotationLimitError(f"value {value} does not fit uint16")
    return value.to_bytes(2, "little")


def resolve_raw_position(raw: int, config: FeetechRotationConfig) -> float:
    if not 0 <= raw < config.counts_per_turn:
        raise FeetechRotationPositionError(
            f"raw position {raw} is outside 0..{config.counts_per_turn - 1}"
        )
    candidates: list[float] = []
    for turns in range(-2, 3):
        delta = raw - config.zero_raw + turns * config.counts_per_turn
        value = config.direction_sign * delta * math.tau / config.counts_per_turn
        if config.min_position_rad - 1e-12 <= value <= config.max_position_rad + 1e-12:
            candidates.append(value)
    if len(candidates) != 1:
        raise FeetechRotationPositionError(
            f"rotation {config.name}: raw position {raw} has {len(candidates)} "
            "legal finite-travel mappings"
        )
    return min(max(candidates[0], config.min_position_rad), config.max_position_rad)


def position_rad_to_raw(position_rad: float, config: FeetechRotationConfig) -> int:
    if not math.isfinite(position_rad):
        raise FeetechRotationLimitError("position_rad must be finite")
    if not config.min_position_rad <= position_rad <= config.max_position_rad:
        raise FeetechRotationLimitError(
            f"target {position_rad:.6f} rad is outside "
            f"[{config.min_position_rad:.6f}, {config.max_position_rad:.6f}]"
        )
    raw = round(
        config.zero_raw
        + config.direction_sign * position_rad * config.counts_per_turn / math.tau
    )
    return raw % config.counts_per_turn


def build_position_payload(
    position_rad: float,
    speed_raw: int,
    config: FeetechRotationConfig,
    *,
    move_time_raw: int = 0,
    acceleration_raw: int = 0,
) -> tuple[int, bytes]:
    """在任何 I/O 前验证命令并生成官方 position/time/speed 六字节载荷。"""

    target_raw = position_rad_to_raw(position_rad, config)
    if not 1 <= speed_raw <= config.max_speed_raw:
        raise FeetechRotationLimitError(
            f"speed_raw must be in 1..{config.max_speed_raw}"
        )
    if not 0 <= move_time_raw <= 0xFFFF:
        raise FeetechRotationLimitError("move_time_raw must fit uint16")
    if not 0 <= acceleration_raw <= 0xFF:
        raise FeetechRotationLimitError("acceleration_raw must fit uint8")
    payload = (
        _uint16_le(target_raw)
        + _uint16_le(move_time_raw)
        + _uint16_le(speed_raw)
    )
    return target_raw, payload


class FeetechRotationAxis:
    """需要完整标定配置、且绝不自动使能转矩的旋转轴。"""

    def __init__(self, bus: FeetechBus, config: FeetechRotationConfig) -> None:
        self.bus = bus
        self.config = config

    def enable_torque(self) -> None:
        self.bus.write_registers(
            self.config.servo_id,
            self.config.registers.torque_enable,
            b"\x01",
            expect_status=self.config.expect_write_status,
        )

    def disable_torque(self) -> None:
        self.bus.write_registers(
            self.config.servo_id,
            self.config.registers.torque_enable,
            b"\x00",
            expect_status=self.config.expect_write_status,
        )

    def torque_enabled(self) -> bool:
        """读取 Torque Enable 寄存器并拒绝含糊的非 0/1 状态。"""

        data = self.bus.read_registers(
            self.config.servo_id,
            self.config.registers.torque_enable,
            1,
        )
        if len(data) != 1 or data[0] not in (0, 1):
            rendered = data.hex(" ") if data else "<empty>"
            raise FeetechRotationError(
                f"rotation {self.config.name}: invalid torque-enable register "
                f"value {rendered}; expected 00 or 01"
            )
        return data[0] == 1

    def read_position(self) -> float:
        data = self.bus.read_registers(
            self.config.servo_id,
            self.config.registers.present_position,
            2,
        )
        return resolve_raw_position(int.from_bytes(data, "little"), self.config)

    def read_feedback(self) -> FeetechRotationFeedback:
        data = self.bus.read_registers(
            self.config.servo_id,
            self.config.registers.present_position,
            self.config.registers.feedback_length,
        )
        raw = int.from_bytes(data[0:2], "little")
        return FeetechRotationFeedback(
            position_raw=raw,
            position_rad=resolve_raw_position(raw, self.config),
            speed_raw=int.from_bytes(data[2:4], "little"),
            load_raw=int.from_bytes(data[4:6], "little"),
            voltage_raw=data[6],
            temperature_c=data[7],
            moving=bool(data[8]),
            error_raw=data[9],
        )

    def command_position(
        self,
        position_rad: float,
        speed_raw: int,
        *,
        move_time_raw: int = 0,
        acceleration_raw: int = 0,
    ) -> int:
        target_raw, payload = build_position_payload(
            position_rad,
            speed_raw,
            self.config,
            move_time_raw=move_time_raw,
            acceleration_raw=acceleration_raw,
        )

        if acceleration_raw:
            self.bus.write_registers(
                self.config.servo_id,
                self.config.registers.acceleration,
                bytes((acceleration_raw,)),
                expect_status=self.config.expect_write_status,
            )
        # 官方磁编码协议从 0x2A 连续写入 position/time/speed，共 6 字节。
        self.bus.write_registers(
            self.config.servo_id,
            self.config.registers.goal_position,
            payload,
            expect_status=self.config.expect_write_status,
        )
        return target_raw

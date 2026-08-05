"""统一吸盘动作语义及 STM32 协议适配。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol, runtime_checkable


class SuctionMode(str, Enum):
    """STM32 当前吸盘状态机暴露的高级动作语义。"""

    IDLE = "idle"
    GRIP = "grip"
    RELEASE = "release"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class SuctionStatus:
    """已确认的输出命令状态；不代表物理真空已经建立。"""

    mode: SuctionMode
    command_acknowledged: bool
    physically_verified: bool
    vacuum_detected: bool | None
    pump_on: bool
    release_open: bool
    busy: bool
    fault: int
    raw_state: int


@runtime_checkable
class SuctionControl(Protocol):
    """离散吸盘能力，不属于连续轴运动目标。"""

    def grip(self) -> SuctionStatus: ...

    def release(self) -> SuctionStatus: ...

    def idle(self) -> SuctionStatus: ...

    def get_status(self) -> SuctionStatus: ...


class STM32SuctionControl:
    """复用一个既有 STM32MotionClient 的吸盘高级语义适配器。"""

    def __init__(self, client: object) -> None:
        self._client = client

    def grip(self) -> SuctionStatus:
        self._client.suction_grip()
        return self.get_status()

    def release(self) -> SuctionStatus:
        self._client.suction_release()
        return self.get_status()

    def idle(self) -> SuctionStatus:
        self._client.suction_idle()
        return self.get_status()

    def get_status(self) -> SuctionStatus:
        raw = self._client.query_suction()
        return SuctionStatus(
            mode=_mode_from_raw_state(raw.state),
            command_acknowledged=True,
            physically_verified=False,
            vacuum_detected=None,
            pump_on=raw.pump_on,
            release_open=raw.release_open,
            busy=raw.busy,
            fault=raw.fault,
            raw_state=raw.state,
        )


def _mode_from_raw_state(state: int) -> SuctionMode:
    # Firmware app_suction_get_state(): 0=IDLE, 1=SUCTION, 3=RELEASE.
    return {
        0: SuctionMode.IDLE,
        1: SuctionMode.GRIP,
        3: SuctionMode.RELEASE,
    }.get(state, SuctionMode.UNKNOWN)


__all__ = [
    "STM32SuctionControl",
    "SuctionControl",
    "SuctionMode",
    "SuctionStatus",
]

"""本机硬件身份和通信参数的纯数据配置。"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module


class HardwareConfigLoadError(RuntimeError):
    """本地硬件配置缺失或内容无效。"""


def _validate_usb_id(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be in 0x0000..0xFFFF")


def _validate_positive_integer(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"{name} must be an integer")
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")


@dataclass(frozen=True)
class UsbVidPid:
    """一对 16 位 USB 厂商/产品标识符。"""

    vid: int
    pid: int

    def __post_init__(self) -> None:
        _validate_usb_id(self.vid, "vid")
        _validate_usb_id(self.pid, "pid")


@dataclass(frozen=True)
class UsbSerialDeviceConfig:
    """USB 串口设备身份及串口通信参数。"""

    identity: UsbVidPid
    baudrate: int
    port_override: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.identity, UsbVidPid):
            raise TypeError("identity must be UsbVidPid")
        _validate_positive_integer(self.baudrate, "baudrate")
        if self.port_override is not None:
            if not isinstance(self.port_override, str):
                raise TypeError("port_override must be a string or None")
            normalized = self.port_override.strip()
            if not normalized:
                raise ValueError("port_override must not be empty")
            object.__setattr__(self, "port_override", normalized)


@dataclass(frozen=True)
class GsUsbDeviceConfig:
    """gs_usb 设备身份及后续启动时使用的 CAN bitrate。"""

    identity: UsbVidPid
    bitrate: int

    def __post_init__(self) -> None:
        if not isinstance(self.identity, UsbVidPid):
            raise TypeError("identity must be UsbVidPid")
        _validate_positive_integer(self.bitrate, "bitrate")


@dataclass(frozen=True)
class HardwareConfig:
    """三类本机硬件的聚合配置。"""

    feetech: UsbSerialDeviceConfig
    stm32_motion: UsbSerialDeviceConfig
    can_adapter: GsUsbDeviceConfig

    def __post_init__(self) -> None:
        if not isinstance(self.feetech, UsbSerialDeviceConfig):
            raise TypeError("feetech must be UsbSerialDeviceConfig")
        if not isinstance(self.stm32_motion, UsbSerialDeviceConfig):
            raise TypeError("stm32_motion must be UsbSerialDeviceConfig")
        if not isinstance(self.can_adapter, GsUsbDeviceConfig):
            raise TypeError("can_adapter must be GsUsbDeviceConfig")


def load_local_hardware_config() -> HardwareConfig:
    """加载同一包内被 Git 忽略的 ``hardware_local.py``。"""

    module_name = f"{__package__}.hardware_local"
    try:
        module = import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name != module_name:
            raise HardwareConfigLoadError(
                f"导入 {module_name} 时缺少依赖模块 {exc.name!r}"
            ) from exc
        raise HardwareConfigLoadError(
            "未找到本地硬件配置。请复制 "
            "host/config/hardware_local.example.py 为 "
            "host/config/hardware_local.py 后填写本机纯数据配置。"
        ) from exc
    except Exception as exc:
        raise HardwareConfigLoadError(
            f"导入本地硬件配置 {module_name} 失败: {exc}"
        ) from exc

    config = getattr(module, "HARDWARE", None)
    if not isinstance(config, HardwareConfig):
        raise HardwareConfigLoadError(
            f"{module_name}.HARDWARE 必须是 HardwareConfig 实例"
        )
    return config


__all__ = [
    "GsUsbDeviceConfig",
    "HardwareConfig",
    "HardwareConfigLoadError",
    "UsbSerialDeviceConfig",
    "UsbVidPid",
    "load_local_hardware_config",
]

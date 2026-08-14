"""按 USB VID/PID 枚举并唯一解析本机硬件；不打开或控制设备。"""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Any

from gs_usb.gs_usb import GsUsb
from serial.tools import list_ports
import usb.util as usb_util
from usb.backend import libusb1
from usb.core import USBError

from config.hardware import (
    GsUsbDeviceConfig,
    HardwareConfig,
    UsbSerialDeviceConfig,
    UsbVidPid,
)


LOGGER = logging.getLogger(__name__)
DIAGNOSTIC_COMMAND = (
    ".venv/bin/python scripts/list_hardware_devices.py --list-all"
)


class DeviceDiscoveryError(RuntimeError):
    pass


class DeviceNotFoundError(DeviceDiscoveryError):
    pass


class AmbiguousDeviceError(DeviceDiscoveryError):
    pass


class DeviceIdentityMismatchError(DeviceDiscoveryError):
    pass


class DeviceMetadataError(DeviceDiscoveryError):
    pass


@dataclass(frozen=True)
class ResolvedSerialDevice:
    port: str
    vid: int | None
    pid: int | None
    serial_number: str | None
    description: str | None
    manufacturer: str | None
    product: str | None
    interface: str | None
    location: str | None


@dataclass(frozen=True)
class ResolvedGsUsbDevice:
    device: object
    vid: int
    pid: int
    serial_number: str | None
    bus: int | None
    address: int | None
    product: str | None


@dataclass(frozen=True)
class ResolvedHardware:
    """一次启动中三类硬件的完整解析结果。"""

    stm32_motion: ResolvedSerialDevice
    can_adapter: ResolvedGsUsbDevice
    feetech: ResolvedSerialDevice


def _optional_text(value: object) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _optional_integer(value: object) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _safe_port_attribute(port: object, name: str) -> object | None:
    try:
        return getattr(port, name, None)
    except Exception as exc:
        LOGGER.warning("读取串口字段 %s 失败: %s", name, exc)
        return None


def list_usb_serial_devices() -> tuple[ResolvedSerialDevice, ...]:
    """列出 pySerial 当前枚举到的端口，不打开任何串口。"""

    try:
        ports = list_ports.comports()
    except Exception as exc:
        raise DeviceMetadataError(f"枚举串口失败: {exc}") from exc

    devices: list[ResolvedSerialDevice] = []
    for port_info in ports:
        port = _optional_text(_safe_port_attribute(port_info, "device"))
        if port is None:
            LOGGER.warning("忽略没有 device 路径的串口枚举项")
            continue
        devices.append(
            ResolvedSerialDevice(
                port=port,
                vid=_optional_integer(_safe_port_attribute(port_info, "vid")),
                pid=_optional_integer(_safe_port_attribute(port_info, "pid")),
                serial_number=_optional_text(
                    _safe_port_attribute(port_info, "serial_number")
                ),
                description=_optional_text(
                    _safe_port_attribute(port_info, "description")
                ),
                manufacturer=_optional_text(
                    _safe_port_attribute(port_info, "manufacturer")
                ),
                product=_optional_text(_safe_port_attribute(port_info, "product")),
                interface=_optional_text(
                    _safe_port_attribute(port_info, "interface")
                ),
                location=_optional_text(
                    _safe_port_attribute(port_info, "location")
                ),
            )
        )
    return tuple(devices)


def _vid_pid_text(vid: int | None, pid: int | None) -> str:
    if vid is None or pid is None:
        return "unknown"
    return f"{vid:04X}:{pid:04X}"


def _serial_candidates_text(devices: tuple[ResolvedSerialDevice, ...]) -> str:
    if not devices:
        return "  (none)"
    return "\n".join(
        "  port={port!r}, VID:PID={identity}, serial={serial!r}, "
        "description={description!r}".format(
            port=device.port,
            identity=_vid_pid_text(device.vid, device.pid),
            serial=device.serial_number,
            description=device.description,
        )
        for device in devices
    )


def _expected_text(identity: UsbVidPid) -> str:
    return _vid_pid_text(identity.vid, identity.pid)


def resolve_usb_serial_port(
    role: str,
    config: UsbSerialDeviceConfig,
) -> ResolvedSerialDevice:
    """按 VID/PID 唯一解析端口；override 也必须通过身份验证。"""

    devices = list_usb_serial_devices()
    expected = config.identity
    candidates = _serial_candidates_text(devices)

    if config.port_override is not None:
        overrides = tuple(
            device for device in devices if device.port == config.port_override
        )
        if not overrides:
            raise DeviceNotFoundError(
                f"{role}: port_override {config.port_override!r} 不存在；"
                f"期望 VID/PID {_expected_text(expected)}。\n"
                f"当前串口:\n{candidates}\n诊断: {DIAGNOSTIC_COMMAND}"
            )
        if len(overrides) > 1:
            raise AmbiguousDeviceError(
                f"{role}: port_override {config.port_override!r} 对应多个枚举项。\n"
                f"候选:\n{_serial_candidates_text(overrides)}\n"
                f"诊断: {DIAGNOSTIC_COMMAND}"
            )
        selected = overrides[0]
        if selected.vid != expected.vid or selected.pid != expected.pid:
            raise DeviceIdentityMismatchError(
                f"{role}: port_override {config.port_override!r} 身份不匹配；"
                f"期望 VID/PID {_expected_text(expected)}，实际 "
                f"{_vid_pid_text(selected.vid, selected.pid)}，"
                f"serial={selected.serial_number!r}, "
                f"description={selected.description!r}。\n"
                f"诊断: {DIAGNOSTIC_COMMAND}"
            )
        return selected

    matches = tuple(
        device
        for device in devices
        if device.vid == expected.vid and device.pid == expected.pid
    )
    if not matches:
        raise DeviceNotFoundError(
            f"{role}: 未找到 VID/PID {_expected_text(expected)}。\n"
            f"当前串口:\n{candidates}\n诊断: {DIAGNOSTIC_COMMAND}"
        )
    if len(matches) > 1:
        raise AmbiguousDeviceError(
            f"{role}: 当前存在多个相同 VID/PID {_expected_text(expected)} 的设备。\n"
            f"候选:\n{_serial_candidates_text(matches)}\n"
            "需要增加 serial number 匹配，或使用经过身份验证的 port_override。\n"
            f"诊断: {DIAGNOSTIC_COMMAND}"
        )
    return matches[0]


# 兼容更偏向“设备”语义的调用名称；两者执行完全相同的严格解析。
resolve_usb_serial_device = resolve_usb_serial_port


_OPTIONAL_DESCRIPTOR_ERRORS = (ValueError, USBError, AttributeError, TypeError)


def _ensure_gs_usb_backend() -> None:
    """Load the project-bundled libusb backend when system discovery fails."""

    if libusb1.get_backend() is not None:
        return
    try:
        import libusb_package
    except ImportError as exc:
        raise RuntimeError(
            "libusb 1.0 backend is unavailable; install the Windows dependencies "
            "from requirements.txt"
        ) from exc
    if libusb1.get_backend(find_library=libusb_package.find_library) is None:
        raise RuntimeError("libusb 1.0 backend could not be loaded")


def _safe_gs_usb_metadata(device: object, name: str, getter: Any) -> object | None:
    try:
        return getter()
    except _OPTIONAL_DESCRIPTOR_ERRORS as exc:
        LOGGER.warning("读取 gs_usb %s 失败: %s", name, exc)
        return None


def _dispose_gs_usb_discovery_handle(usb_device: object) -> None:
    """Release descriptor-read handles before python-can reopens the device."""

    if getattr(usb_device, "_ctx", None) is None:
        return
    usb_util.dispose_resources(usb_device)


def list_gs_usb_devices() -> tuple[ResolvedGsUsbDevice, ...]:
    """列出 gs_usb 设备及安全可读的诊断元数据。"""

    try:
        _ensure_gs_usb_backend()
        scanned = GsUsb.scan()
    except Exception as exc:
        raise DeviceMetadataError(f"枚举 gs_usb 设备失败: {exc}") from exc

    devices: list[ResolvedGsUsbDevice] = []
    for index, device in enumerate(scanned):
        usb_device = getattr(device, "gs_usb", None)
        try:
            vid = int(usb_device.idVendor)
            pid = int(usb_device.idProduct)
        except (AttributeError, TypeError, ValueError, USBError) as exc:
            if usb_device is not None:
                _dispose_gs_usb_discovery_handle(usb_device)
            raise DeviceMetadataError(
                f"gs_usb 候选 {index} 无法读取关键 VID/PID: {exc}"
            ) from exc
        if not 0 <= vid <= 0xFFFF or not 0 <= pid <= 0xFFFF:
            raise DeviceMetadataError(
                f"gs_usb 候选 {index} 的 VID/PID 超出 uint16 范围: {vid}/{pid}"
            )

        serial_number = _optional_text(
            _safe_gs_usb_metadata(
                device, "serial_number", lambda: device.serial_number
            )
        )
        bus = _optional_integer(
            _safe_gs_usb_metadata(device, "bus", lambda: device.bus)
        )
        address = _optional_integer(
            _safe_gs_usb_metadata(device, "address", lambda: device.address)
        )
        product = _optional_text(
            _safe_gs_usb_metadata(
                device, "product", lambda: device.gs_usb.product
            )
        )
        _dispose_gs_usb_discovery_handle(usb_device)
        devices.append(
            ResolvedGsUsbDevice(
                device=device,
                vid=vid,
                pid=pid,
                serial_number=serial_number,
                bus=bus,
                address=address,
                product=product,
            )
        )
    return tuple(devices)


def _gs_usb_candidates_text(devices: tuple[ResolvedGsUsbDevice, ...]) -> str:
    if not devices:
        return "  (none)"
    return "\n".join(
        "  VID:PID={identity}, serial={serial!r}, bus={bus!r}, "
        "address={address!r}, product={product!r}".format(
            identity=_vid_pid_text(device.vid, device.pid),
            serial=device.serial_number,
            bus=device.bus,
            address=device.address,
            product=device.product,
        )
        for device in devices
    )


def resolve_gs_usb_device(
    role: str,
    config: GsUsbDeviceConfig,
) -> ResolvedGsUsbDevice:
    """按 VID/PID 唯一解析 gs_usb；不启动或配置 CAN。"""

    devices = list_gs_usb_devices()
    expected = config.identity
    matches = tuple(
        device
        for device in devices
        if device.vid == expected.vid and device.pid == expected.pid
    )
    if not matches:
        raise DeviceNotFoundError(
            f"{role}: 未找到 gs_usb VID/PID {_expected_text(expected)}。\n"
            f"当前 gs_usb:\n{_gs_usb_candidates_text(devices)}\n"
            f"诊断: {DIAGNOSTIC_COMMAND}"
        )
    if len(matches) > 1:
        raise AmbiguousDeviceError(
            f"{role}: 当前存在多个相同 VID/PID {_expected_text(expected)} 的 "
            "gs_usb 设备。\n"
            f"候选:\n{_gs_usb_candidates_text(matches)}\n"
            "当前版本不使用 serial number 消除歧义；未来可增加第二级身份条件。\n"
            f"诊断: {DIAGNOSTIC_COMMAND}"
        )
    return matches[0]


def resolve_hardware(config: HardwareConfig) -> ResolvedHardware:
    """解析三类硬件但不打开、启动或控制任何设备。"""

    if not isinstance(config, HardwareConfig):
        raise TypeError("config must be HardwareConfig")
    return ResolvedHardware(
        stm32_motion=resolve_usb_serial_port(
            "stm32_motion",
            config.stm32_motion,
        ),
        can_adapter=resolve_gs_usb_device(
            "can_adapter",
            config.can_adapter,
        ),
        feetech=resolve_usb_serial_port(
            "feetech",
            config.feetech,
        ),
    )


__all__ = [
    "AmbiguousDeviceError",
    "DeviceDiscoveryError",
    "DeviceIdentityMismatchError",
    "DeviceMetadataError",
    "DeviceNotFoundError",
    "ResolvedGsUsbDevice",
    "ResolvedHardware",
    "ResolvedSerialDevice",
    "list_gs_usb_devices",
    "list_usb_serial_devices",
    "resolve_gs_usb_device",
    "resolve_hardware",
    "resolve_usb_serial_device",
    "resolve_usb_serial_port",
]

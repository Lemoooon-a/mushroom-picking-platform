#!/usr/bin/env python3
"""只读枚举或解析本机 USB 串口与 gs_usb 设备。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from config.hardware import (  # noqa: E402
    HardwareConfigLoadError,
    load_local_hardware_config,
)
from drivers.device_discovery import (  # noqa: E402
    DeviceDiscoveryError,
    ResolvedGsUsbDevice,
    ResolvedSerialDevice,
    list_gs_usb_devices,
    list_usb_serial_devices,
    resolve_gs_usb_device,
    resolve_usb_serial_port,
)


def _identity(vid: int | None, pid: int | None) -> str:
    if vid is None or pid is None:
        return "unknown"
    return f"{vid:04X}:{pid:04X}"


def _print_serial(device: ResolvedSerialDevice) -> None:
    print(f"port         : {device.port}")
    print(f"VID:PID      : {_identity(device.vid, device.pid)}")
    print(f"serial       : {device.serial_number or '-'}")
    print(f"description  : {device.description or '-'}")
    print(f"manufacturer : {device.manufacturer or '-'}")
    print(f"product      : {device.product or '-'}")
    print(f"interface    : {device.interface or '-'}")
    print(f"location     : {device.location or '-'}")


def _print_gs_usb(device: ResolvedGsUsbDevice) -> None:
    print(f"VID:PID : {_identity(device.vid, device.pid)}")
    print(f"serial  : {device.serial_number or '-'}")
    print(f"product : {device.product or '-'}")
    print(f"bus     : {device.bus if device.bus is not None else '-'}")
    print(f"address : {device.address if device.address is not None else '-'}")


def _list_all() -> int:
    print("USB serial devices")
    serial_devices = list_usb_serial_devices()
    if not serial_devices:
        print("(none)")
    for index, device in enumerate(serial_devices):
        if index:
            print()
        _print_serial(device)

    print("\ngs_usb devices")
    gs_usb_devices = list_gs_usb_devices()
    if not gs_usb_devices:
        print("(none)")
    for index, device in enumerate(gs_usb_devices):
        if index:
            print()
        _print_gs_usb(device)
    return 0


def _resolve() -> int:
    config = load_local_hardware_config()
    failed = False

    serial_roles = (
        ("feetech", config.feetech),
        ("stm32_motion", config.stm32_motion),
    )
    for role, role_config in serial_roles:
        print(f"{role}:")
        print(
            "  expected: "
            f"{_identity(role_config.identity.vid, role_config.identity.pid)}"
        )
        try:
            resolved = resolve_usb_serial_port(role, role_config)
        except DeviceDiscoveryError as exc:
            failed = True
            print(f"  error: {exc}")
        else:
            print(f"  matched port: {resolved.port}")
            print(f"  serial: {resolved.serial_number or '-'}")

    role = "can_adapter"
    role_config = config.can_adapter
    print(f"{role}:")
    print(
        "  expected: "
        f"{_identity(role_config.identity.vid, role_config.identity.pid)}"
    )
    try:
        resolved_can = resolve_gs_usb_device(role, role_config)
    except DeviceDiscoveryError as exc:
        failed = True
        print(f"  error: {exc}")
    else:
        print(f"  serial: {resolved_can.serial_number or '-'}")
        print(f"  bus: {resolved_can.bus if resolved_can.bus is not None else '-'}")
        print(
            "  address: "
            f"{resolved_can.address if resolved_can.address is not None else '-'}"
        )
    return 2 if failed else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="List or resolve hardware without opening or controlling it"
    )
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--list-all", action="store_true")
    action.add_argument("--resolve", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return _list_all() if args.list_all else _resolve()
    except (DeviceDiscoveryError, HardwareConfigLoadError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

"""硬件配置与 VID/PID 设备发现的纯离线测试。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from config.hardware import (
    GsUsbDeviceConfig,
    HardwareConfig,
    HardwareConfigLoadError,
    UsbSerialDeviceConfig,
    UsbVidPid,
    load_local_hardware_config,
)
from drivers.device_discovery import (
    AmbiguousDeviceError,
    DeviceIdentityMismatchError,
    DeviceMetadataError,
    DeviceNotFoundError,
    list_gs_usb_devices,
    list_usb_serial_devices,
    resolve_gs_usb_device,
    resolve_hardware,
    resolve_usb_serial_port,
)


FEETECH_IDENTITY = UsbVidPid(0x1A86, 0x55D3)
STM32_IDENTITY = UsbVidPid(0x0483, 0x374B)
GS_USB_IDENTITY = UsbVidPid(0x1D50, 0x606F)


def serial_config(
    identity: UsbVidPid = STM32_IDENTITY,
    *,
    override: str | None = None,
) -> UsbSerialDeviceConfig:
    return UsbSerialDeviceConfig(identity, 115200, override)


def port(
    device: str,
    vid: int | None,
    pid: int | None,
    *,
    serial: str | None = None,
    description: str = "fake serial port",
) -> SimpleNamespace:
    return SimpleNamespace(
        device=device,
        vid=vid,
        pid=pid,
        serial_number=serial,
        description=description,
        manufacturer="Fake Manufacturer",
        product="Fake Product",
        interface="Interface 0",
        location="1-2.3",
    )


class FakeUsbCore:
    def __init__(
        self,
        vid: int,
        pid: int,
        product: str | None = "Fake gs_usb",
        *,
        product_error: BaseException | None = None,
    ) -> None:
        self.idVendor = vid
        self.idProduct = pid
        self._product = product
        self._product_error = product_error

    @property
    def product(self) -> str | None:
        if self._product_error is not None:
            raise self._product_error
        return self._product


class FakeGsUsb:
    def __init__(
        self,
        vid: int = GS_USB_IDENTITY.vid,
        pid: int = GS_USB_IDENTITY.pid,
        *,
        serial: str | None = "SERIAL",
        serial_error: BaseException | None = None,
        product_error: BaseException | None = None,
        bus: int = 1,
        address: int = 2,
    ) -> None:
        self.gs_usb = FakeUsbCore(
            vid,
            pid,
            product_error=product_error,
        )
        self._serial = serial
        self._serial_error = serial_error
        self._bus = bus
        self._address = address
        self.start = Mock()
        self.set_bitrate = Mock()
        self.send = Mock()
        self.read = Mock()

    @property
    def serial_number(self) -> str | None:
        if self._serial_error is not None:
            raise self._serial_error
        return self._serial

    @property
    def bus(self) -> int:
        return self._bus

    @property
    def address(self) -> int:
        return self._address


def gs_usb_config() -> GsUsbDeviceConfig:
    return GsUsbDeviceConfig(GS_USB_IDENTITY, 1_000_000)


class SerialDiscoveryTests(unittest.TestCase):
    @patch("drivers.device_discovery.list_ports.comports")
    def test_unique_vid_pid_match_preserves_metadata(self, comports: Mock) -> None:
        expected = port(
            "/dev/cu.usbmodem1103",
            STM32_IDENTITY.vid,
            STM32_IDENTITY.pid,
            serial="STM32-SERIAL",
        )
        comports.return_value = [
            port("/dev/cu.other", FEETECH_IDENTITY.vid, FEETECH_IDENTITY.pid),
            expected,
        ]

        resolved = resolve_usb_serial_port("stm32_motion", serial_config())

        self.assertEqual(resolved.port, expected.device)
        self.assertEqual(resolved.serial_number, "STM32-SERIAL")
        self.assertEqual(resolved.location, "1-2.3")

    def _assert_platform_path(self, path: str) -> None:
        with patch(
            "drivers.device_discovery.list_ports.comports",
            return_value=[port(path, STM32_IDENTITY.vid, STM32_IDENTITY.pid)],
        ):
            self.assertEqual(
                resolve_usb_serial_port("stm32_motion", serial_config()).port,
                path,
            )

    def test_macos_path_is_returned_verbatim(self) -> None:
        self._assert_platform_path("/dev/cu.usbmodem1103")

    def test_linux_tty_acm_path_is_returned_verbatim(self) -> None:
        self._assert_platform_path("/dev/ttyACM0")

    def test_linux_tty_usb_path_is_returned_verbatim(self) -> None:
        self._assert_platform_path("/dev/ttyUSB0")

    def test_windows_com_path_is_returned_verbatim(self) -> None:
        self._assert_platform_path("COM7")

    @patch("drivers.device_discovery.list_ports.comports", return_value=[])
    def test_no_matching_device_is_explicit(self, _comports: Mock) -> None:
        with self.assertRaisesRegex(
            DeviceNotFoundError, "(?s)stm32_motion.*0483:374B.*--list-all"
        ):
            resolve_usb_serial_port("stm32_motion", serial_config())

    @patch("drivers.device_discovery.list_ports.comports")
    def test_multiple_same_identity_is_ambiguous(self, comports: Mock) -> None:
        comports.return_value = [
            port("COM7", STM32_IDENTITY.vid, STM32_IDENTITY.pid),
            port("COM8", STM32_IDENTITY.vid, STM32_IDENTITY.pid),
        ]
        with self.assertRaisesRegex(AmbiguousDeviceError, "(?s)COM7.*COM8"):
            resolve_usb_serial_port("stm32_motion", serial_config())

    @patch("drivers.device_discovery.list_ports.comports")
    def test_different_serials_do_not_resolve_ambiguity(self, comports: Mock) -> None:
        comports.return_value = [
            port("COM7", STM32_IDENTITY.vid, STM32_IDENTITY.pid, serial="A"),
            port("COM8", STM32_IDENTITY.vid, STM32_IDENTITY.pid, serial="B"),
        ]
        with self.assertRaisesRegex(AmbiguousDeviceError, "serial number"):
            resolve_usb_serial_port("stm32_motion", serial_config())

    @patch("drivers.device_discovery.list_ports.comports")
    def test_same_description_different_identity_is_not_selected(
        self, comports: Mock
    ) -> None:
        comports.return_value = [
            port("COM1", 0x1111, 0x2222, description="same"),
            port(
                "COM7",
                STM32_IDENTITY.vid,
                STM32_IDENTITY.pid,
                description="same",
            ),
        ]
        self.assertEqual(
            resolve_usb_serial_port("stm32_motion", serial_config()).port,
            "COM7",
        )

    def test_same_vid_wrong_pid_is_not_selected(self) -> None:
        with patch(
            "drivers.device_discovery.list_ports.comports",
            return_value=[port("COM7", STM32_IDENTITY.vid, 0xFFFF)],
        ):
            with self.assertRaises(DeviceNotFoundError):
                resolve_usb_serial_port("stm32_motion", serial_config())

    def test_same_pid_wrong_vid_is_not_selected(self) -> None:
        with patch(
            "drivers.device_discovery.list_ports.comports",
            return_value=[port("COM7", 0xFFFF, STM32_IDENTITY.pid)],
        ):
            with self.assertRaises(DeviceNotFoundError):
                resolve_usb_serial_port("stm32_motion", serial_config())

    @patch("drivers.device_discovery.list_ports.comports")
    def test_port_override_is_verified_and_returned(self, comports: Mock) -> None:
        comports.return_value = [
            port("COM7", STM32_IDENTITY.vid, STM32_IDENTITY.pid),
            port("COM8", STM32_IDENTITY.vid, STM32_IDENTITY.pid),
        ]
        resolved = resolve_usb_serial_port(
            "stm32_motion", serial_config(override="COM8")
        )
        self.assertEqual(resolved.port, "COM8")

    @patch("drivers.device_discovery.list_ports.comports", return_value=[])
    def test_port_override_missing_path_fails(self, _comports: Mock) -> None:
        with self.assertRaisesRegex(DeviceNotFoundError, "COM7.*不存在"):
            resolve_usb_serial_port(
                "stm32_motion", serial_config(override="COM7")
            )

    def test_port_override_identity_mismatch_fails(self) -> None:
        with patch(
            "drivers.device_discovery.list_ports.comports",
            return_value=[port("COM7", 0x1234, 0x5678)],
        ):
            with self.assertRaisesRegex(
                DeviceIdentityMismatchError, "0483:374B.*1234:5678"
            ):
                resolve_usb_serial_port(
                    "stm32_motion", serial_config(override="COM7")
                )

    @patch("serial.Serial")
    @patch("drivers.device_discovery.list_ports.comports")
    def test_resolver_never_opens_serial(
        self, comports: Mock, serial_constructor: Mock
    ) -> None:
        comports.return_value = [
            port("COM7", STM32_IDENTITY.vid, STM32_IDENTITY.pid)
        ]
        resolve_usb_serial_port("stm32_motion", serial_config())
        serial_constructor.assert_not_called()

    @patch("drivers.device_discovery.list_ports.comports")
    def test_path_does_not_infer_identity_or_serial(self, comports: Mock) -> None:
        comports.return_value = [port("/dev/ttyACM0", None, None)]
        listed = list_usb_serial_devices()
        self.assertEqual(len(listed), 1)
        self.assertIsNone(listed[0].vid)
        self.assertIsNone(listed[0].serial_number)
        with self.assertRaises(DeviceNotFoundError):
            resolve_usb_serial_port("stm32_motion", serial_config())

    def test_non_usb_serial_is_not_selected(self) -> None:
        with patch(
            "drivers.device_discovery.list_ports.comports",
            return_value=[port("COM1", None, None)],
        ):
            with self.assertRaises(DeviceNotFoundError):
                resolve_usb_serial_port("stm32_motion", serial_config())


class GsUsbDiscoveryTests(unittest.TestCase):
    @patch("drivers.device_discovery.GsUsb.scan")
    def test_unique_match_returns_device_and_metadata(self, scan: Mock) -> None:
        expected = FakeGsUsb(serial="CAN-SERIAL", bus=3, address=4)
        scan.return_value = [FakeGsUsb(0x1111, 0x2222), expected]

        resolved = resolve_gs_usb_device("can_adapter", gs_usb_config())

        self.assertIs(resolved.device, expected)
        self.assertEqual(resolved.serial_number, "CAN-SERIAL")
        self.assertEqual((resolved.bus, resolved.address), (3, 4))

    @patch("drivers.device_discovery.GsUsb.scan", return_value=[])
    def test_no_match_is_explicit(self, _scan: Mock) -> None:
        with self.assertRaisesRegex(DeviceNotFoundError, "1D50:606F"):
            resolve_gs_usb_device("can_adapter", gs_usb_config())

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_multiple_matches_are_ambiguous(self, scan: Mock) -> None:
        scan.return_value = [FakeGsUsb(serial="A"), FakeGsUsb(serial="B")]
        with self.assertRaisesRegex(AmbiguousDeviceError, "(?s)A.*B"):
            resolve_gs_usb_device("can_adapter", gs_usb_config())

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_other_vid_pid_is_not_selected(self, scan: Mock) -> None:
        scan.return_value = [FakeGsUsb(0x1111, 0x2222)]
        with self.assertRaises(DeviceNotFoundError):
            resolve_gs_usb_device("can_adapter", gs_usb_config())

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_serial_is_diagnostic_only(self, scan: Mock) -> None:
        scan.return_value = [FakeGsUsb(serial="A"), FakeGsUsb(serial="B")]
        with self.assertRaises(AmbiguousDeviceError):
            resolve_gs_usb_device("can_adapter", gs_usb_config())

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_serial_descriptor_error_does_not_block_match(self, scan: Mock) -> None:
        scan.return_value = [FakeGsUsb(serial_error=ValueError("bad descriptor"))]
        with self.assertLogs("drivers.device_discovery", level="WARNING"):
            resolved = resolve_gs_usb_device("can_adapter", gs_usb_config())
        self.assertIsNone(resolved.serial_number)

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_product_descriptor_error_does_not_block_match(self, scan: Mock) -> None:
        scan.return_value = [FakeGsUsb(product_error=ValueError("bad product"))]
        with self.assertLogs("drivers.device_discovery", level="WARNING"):
            resolved = resolve_gs_usb_device("can_adapter", gs_usb_config())
        self.assertIsNone(resolved.product)

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_resolver_performs_no_can_control_io(self, scan: Mock) -> None:
        device = FakeGsUsb()
        scan.return_value = [device]

        resolve_gs_usb_device("can_adapter", gs_usb_config())

        device.start.assert_not_called()
        device.set_bitrate.assert_not_called()
        device.send.assert_not_called()
        device.read.assert_not_called()

    @patch(
        "drivers.device_discovery.GsUsb.scan",
        side_effect=RuntimeError("USB backend unavailable"),
    )
    def test_scan_failure_is_wrapped(self, _scan: Mock) -> None:
        with self.assertRaisesRegex(DeviceMetadataError, "USB backend unavailable"):
            list_gs_usb_devices()

    @patch("drivers.device_discovery.GsUsb.scan")
    def test_missing_critical_identity_is_metadata_error(self, scan: Mock) -> None:
        scan.return_value = [SimpleNamespace(gs_usb=SimpleNamespace())]
        with self.assertRaisesRegex(DeviceMetadataError, "VID/PID"):
            list_gs_usb_devices()


class HardwareConfigTests(unittest.TestCase):
    @patch("drivers.device_discovery.resolve_gs_usb_device")
    @patch("drivers.device_discovery.resolve_usb_serial_port")
    def test_aggregate_resolver_uses_each_config_once_without_control_io(
        self,
        resolve_serial: Mock,
        resolve_can: Mock,
    ) -> None:
        stm32 = SimpleNamespace(port="COM7")
        feetech = SimpleNamespace(port="COM8")
        can_adapter = SimpleNamespace(device=object())
        resolve_serial.side_effect = [stm32, feetech]
        resolve_can.return_value = can_adapter
        config = HardwareConfig(
            feetech=serial_config(FEETECH_IDENTITY),
            stm32_motion=serial_config(STM32_IDENTITY),
            can_adapter=gs_usb_config(),
        )

        resolved = resolve_hardware(config)

        self.assertIs(resolved.stm32_motion, stm32)
        self.assertIs(resolved.can_adapter, can_adapter)
        self.assertIs(resolved.feetech, feetech)
        self.assertEqual(
            resolve_serial.call_args_list,
            [
                call("stm32_motion", config.stm32_motion),
                call("feetech", config.feetech),
            ],
        )
        resolve_can.assert_called_once_with("can_adapter", config.can_adapter)

    def test_vid_below_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbVidPid(-1, 0)

    def test_vid_above_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbVidPid(0x10000, 0)

    def test_pid_below_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbVidPid(0, -1)

    def test_pid_above_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbVidPid(0, 0x10000)

    def test_zero_baudrate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbSerialDeviceConfig(STM32_IDENTITY, 0)

    def test_negative_baudrate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            UsbSerialDeviceConfig(STM32_IDENTITY, -1)

    def test_zero_bitrate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GsUsbDeviceConfig(GS_USB_IDENTITY, 0)

    def test_negative_bitrate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GsUsbDeviceConfig(GS_USB_IDENTITY, -1)

    def test_empty_port_override_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            serial_config(override="")

    def test_whitespace_port_override_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            serial_config(override="   ")

    def test_port_override_is_trimmed_once_in_pure_data_validation(self) -> None:
        self.assertEqual(serial_config(override="  COM7  ").port_override, "COM7")

    @patch("drivers.device_discovery.GsUsb.scan")
    @patch("drivers.device_discovery.list_ports.comports")
    def test_example_config_import_has_no_hardware_side_effects(
        self, comports: Mock, scan: Mock
    ) -> None:
        path = Path(__file__).resolve().parents[2] / "config/examples/hardware.py"
        spec = importlib.util.spec_from_file_location(
            "config.hardware_example", path
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        self.assertIsInstance(module.HARDWARE, HardwareConfig)
        comports.assert_not_called()
        scan.assert_not_called()

    @patch("config.hardware._load_local_module")
    def test_missing_local_config_has_copy_instruction(self, importer: Mock) -> None:
        importer.side_effect = ModuleNotFoundError(
            "missing", name="config.local.hardware"
        )
        with self.assertRaisesRegex(
            HardwareConfigLoadError,
            "config/examples/hardware.py.*config/local/hardware.py",
        ):
            load_local_hardware_config()

    @patch("config.hardware._load_local_module")
    def test_local_config_requires_hardware_config_instance(self, importer: Mock) -> None:
        importer.return_value = SimpleNamespace(HARDWARE=object())
        with self.assertRaisesRegex(HardwareConfigLoadError, "HardwareConfig"):
            load_local_hardware_config()


if __name__ == "__main__":
    unittest.main()

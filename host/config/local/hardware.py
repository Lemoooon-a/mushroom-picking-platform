"""本机硬件配置模板；复制为 hardware_local.py 后按需调整。"""

from .hardware import (
    GsUsbDeviceConfig,
    HardwareConfig,
    UsbSerialDeviceConfig,
    UsbVidPid,
)


FEETECH = UsbSerialDeviceConfig(
    identity=UsbVidPid(vid=0x1A86, pid=0x55D3),
    baudrate=115200,
)

STM32_MOTION = UsbSerialDeviceConfig(
    identity=UsbVidPid(vid=0x0483, pid=0x374B),
    baudrate=115200,
)

CAN_ADAPTER = GsUsbDeviceConfig(
    identity=UsbVidPid(vid=0x1D50, pid=0x606F),
    bitrate=1_000_000,
)

HARDWARE = HardwareConfig(
    feetech=FEETECH,
    stm32_motion=STM32_MOTION,
    can_adapter=CAN_ADAPTER,
)

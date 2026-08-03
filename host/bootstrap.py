"""上层运动控制的统一硬件组装和显式通信生命周期。"""

from __future__ import annotations

from config.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.hardware import HardwareConfig
from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from config.motion_runtime import MotionRuntimeConfig
from drivers.can_bus import CanMotorBus
from drivers.device_discovery import ResolvedHardware, resolve_hardware
from drivers.feetech_protocol import FeetechBus, FeetechSerialConfig
from drivers.mg4010_driver import MG4010Driver
from drivers.stm32_motion import (
    STM32MotionClient,
    STM32SerialConfig,
    STM32SerialTransport,
)
from motion.authorization import MotionAuthorization, RuntimeMode
from motion.client_facades import FrontendMotionFacade, KinematicsMotionFacade
from motion.client_interfaces import (
    FrontendMotionInterface,
    KinematicsMotionInterface,
)
from motion.unified_controller import UnifiedMotionController
from robot.feetech_rotation import FeetechRotationAxis
from robot.joint import CanRotaryJoint


class MotionRuntimeError(RuntimeError):
    """上层运动 Runtime 的组装或生命周期错误。"""


class HardwareOpenError(MotionRuntimeError):
    """打开通信资源失败，已尝试回滚先前打开的资源。"""


class HardwareCloseError(MotionRuntimeError):
    """一个或多个通信资源关闭失败。"""


class UpperMotionRuntime:
    """三类硬件和两个客户端视图共享的单一 Runtime。

    构造不打开硬件。``open()`` 仅按 STM32、CAN、Feetech 的顺序打开
    通信资源，不初始化、使能、归零或运动任何轴。
    """

    def __init__(
        self,
        *,
        resolved_hardware: ResolvedHardware,
        hardware_config: HardwareConfig,
        motion_config: MotionRuntimeConfig,
        authorization: MotionAuthorization,
        stm32_transport: STM32SerialTransport,
        stm32_client: STM32MotionClient,
        can_bus: CanMotorBus,
        shoulder_joint: CanRotaryJoint,
        elbow_joint: CanRotaryJoint,
        feetech_bus: FeetechBus,
        rotation_axis: FeetechRotationAxis,
        controller: UnifiedMotionController,
    ) -> None:
        self.resolved_hardware = resolved_hardware
        self.hardware_config = hardware_config
        self.motion_config = motion_config
        self.authorization = authorization
        self.stm32_transport = stm32_transport
        self.stm32_client = stm32_client
        self.can_bus = can_bus
        self.shoulder_joint = shoulder_joint
        self.elbow_joint = elbow_joint
        self.feetech_bus = feetech_bus
        self.rotation_axis = rotation_axis
        self.controller = controller
        self._frontend_motion: FrontendMotionInterface = FrontendMotionFacade(
            controller
        )
        self._kinematics_motion: KinematicsMotionInterface = (
            KinematicsMotionFacade(controller)
        )
        self._is_open = False
        self._opened_resources: list[tuple[str, object]] = []

    @property
    def frontend_motion(self) -> FrontendMotionInterface:
        return self._frontend_motion

    @property
    def kinematics_motion(self) -> KinematicsMotionInterface:
        return self._kinematics_motion

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        """打开通信资源；部分失败时按相反顺序回滚。"""

        if self._is_open:
            return

        resources = (
            ("STM32 transport", self.stm32_transport),
            ("CAN bus", self.can_bus),
            ("Feetech bus", self.feetech_bus),
        )
        for resource_name, resource in resources:
            try:
                resource.open()
                self._opened_resources.append((resource_name, resource))
            except Exception as exc:
                rollback_errors = self._rollback_opened_resources()
                rollback_summary = (
                    "rollback completed"
                    if not rollback_errors
                    else "rollback attempted with close errors: "
                    + "; ".join(rollback_errors)
                )
                raise HardwareOpenError(
                    f"open stage failed for {resource_name}: {exc}; "
                    f"{rollback_summary}"
                ) from exc
        self._is_open = True

    def close(self) -> None:
        """逆序关闭所有已打开资源，并在尝试全部关闭后报告错误。"""

        if not self._opened_resources:
            self._is_open = False
            return

        close_errors: list[str] = []
        while self._opened_resources:
            resource_name, resource = self._opened_resources.pop()
            try:
                resource.close()
            except Exception as exc:
                close_errors.append(f"{resource_name}: {exc}")
        self._is_open = False
        if close_errors:
            raise HardwareCloseError(
                "failed to close upper-motion hardware resources: "
                + "; ".join(close_errors)
            )

    def _rollback_opened_resources(self) -> list[str]:
        errors: list[str] = []
        while self._opened_resources:
            resource_name, resource = self._opened_resources.pop()
            try:
                resource.close()
            except Exception as exc:
                errors.append(f"{resource_name}: {exc}")
        self._is_open = False
        return errors

    def __enter__(self) -> "UpperMotionRuntime":
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: object,
    ) -> None:
        try:
            self.close()
        except HardwareCloseError as close_error:
            if exc is None:
                raise
            if hasattr(exc, "add_note"):
                exc.add_note(f"runtime close also failed: {close_error}")


def create_upper_motion_runtime(
    hardware_config: HardwareConfig,
    motion_config: MotionRuntimeConfig,
    *,
    mode: RuntimeMode = RuntimeMode.READ_ONLY,
    allow_unverified_rotation_motion: bool = False,
) -> UpperMotionRuntime:
    """解析并组装完整 Runtime，但不打开或控制任何硬件。"""

    if not isinstance(hardware_config, HardwareConfig):
        raise TypeError("hardware_config must be HardwareConfig")
    if not isinstance(motion_config, MotionRuntimeConfig):
        raise TypeError("motion_config must be MotionRuntimeConfig")

    resolved = resolve_hardware(hardware_config)

    stm32_transport = STM32SerialTransport(
        STM32SerialConfig(
            port=resolved.stm32_motion.port,
            baudrate=hardware_config.stm32_motion.baudrate,
        )
    )
    stm32_client = STM32MotionClient(stm32_transport)

    can_bus = CanMotorBus(
        interface="gs_usb",
        channel=0,
        bitrate=hardware_config.can_adapter.bitrate,
        gs_usb_device=resolved.can_adapter.device,
    )
    shoulder_joint = CanRotaryJoint(
        MG4010Driver(can_bus, SHOULDER_JOINT_CONFIG.motor_id),
        SHOULDER_JOINT_CONFIG,
    )
    elbow_joint = CanRotaryJoint(
        MG4010Driver(can_bus, ELBOW_JOINT_CONFIG.motor_id),
        ELBOW_JOINT_CONFIG,
    )

    feetech_bus = FeetechBus(
        FeetechSerialConfig(
            port=resolved.feetech.port,
            baudrate=hardware_config.feetech.baudrate,
        )
    )
    rotation_axis = FeetechRotationAxis(
        feetech_bus,
        END_EFFECTOR_ROTATION_CONFIG,
    )

    authorization = MotionAuthorization(
        mode=mode,
        allow_unverified_rotation_motion=allow_unverified_rotation_motion,
    )
    controller = UnifiedMotionController(
        stm32_client=stm32_client,
        shoulder_joint=shoulder_joint,
        elbow_joint=elbow_joint,
        rotation_axis=rotation_axis,
        arrival_configs=motion_config.arrival_configs(),
        default_motion_parameters=motion_config.default_motion_parameters(),
        authorization=authorization,
    )

    return UpperMotionRuntime(
        resolved_hardware=resolved,
        hardware_config=hardware_config,
        motion_config=motion_config,
        authorization=authorization,
        stm32_transport=stm32_transport,
        stm32_client=stm32_client,
        can_bus=can_bus,
        shoulder_joint=shoulder_joint,
        elbow_joint=elbow_joint,
        feetech_bus=feetech_bus,
        rotation_axis=rotation_axis,
        controller=controller,
    )


__all__ = [
    "HardwareCloseError",
    "HardwareOpenError",
    "MotionRuntimeError",
    "UpperMotionRuntime",
    "create_upper_motion_runtime",
]

"""上位机各执行器后端当前可证明的能力声明。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BackendCapabilities:
    query_state: bool
    command_position: bool
    command_relative: bool
    stop: bool
    disable: bool
    home: bool
    position_valid: bool
    arrival_event: bool
    motion_timeout: bool
    fault_status: bool


STM32_AXIS_CAPABILITIES = BackendCapabilities(
    query_state=True,
    command_position=True,
    command_relative=True,
    stop=True,
    disable=True,
    home=True,
    position_valid=True,
    arrival_event=True,
    motion_timeout=True,
    fault_status=True,
)

MG4010_JOINT_CAPABILITIES = BackendCapabilities(
    query_state=True,
    command_position=True,
    command_relative=False,
    stop=True,
    disable=True,
    home=False,
    position_valid=True,
    arrival_event=False,
    motion_timeout=False,
    fault_status=True,
)

FEETECH_ROTATION_CAPABILITIES = BackendCapabilities(
    query_state=True,
    command_position=True,
    command_relative=False,
    stop=True,
    disable=True,
    home=False,
    position_valid=False,
    arrival_event=False,
    motion_timeout=False,
    fault_status=True,
)

VACUUM_CAPABILITIES = BackendCapabilities(
    query_state=True,
    command_position=False,
    command_relative=False,
    stop=True,
    disable=False,
    home=False,
    position_valid=False,
    arrival_event=True,
    motion_timeout=True,
    fault_status=True,
)


@dataclass(frozen=True)
class UpperMotionBackends:
    """只聚合后端引用与能力，不隐式执行初始化或硬件命令。"""

    slide: object
    z: object
    shoulder: object
    elbow: object
    rotation: object
    vacuum: object

    def capability_matrix(self) -> dict[str, BackendCapabilities]:
        return {
            "slide": STM32_AXIS_CAPABILITIES,
            "z": STM32_AXIS_CAPABILITIES,
            "shoulder": MG4010_JOINT_CAPABILITIES,
            "elbow": MG4010_JOINT_CAPABILITIES,
            "rotation": FEETECH_ROTATION_CAPABILITIES,
            "vacuum": VACUUM_CAPABILITIES,
        }

"""飞特舵机型号固定参数与项目机械标定参数的组合入口。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from robot.feetech_rotation import (
    FeetechMagneticRegisterMap,
    FeetechRotationConfig,
)


@dataclass(frozen=True)
class FeetechModelProfile:
    """同一舵机型号不会随机械安装变化的通信和编码器参数。"""

    key: str
    model: str
    protocol: str
    transport: str
    default_baudrate: int
    counts_per_turn: int
    adapter_auto_direction: bool
    registers: FeetechMagneticRegisterMap = field(
        default_factory=FeetechMagneticRegisterMap
    )

    def make_rotation_config(
        self,
        *,
        name: str,
        servo_id: int,
        zero_raw: int,
        direction_sign: int,
        min_position_rad: float,
        max_position_rad: float,
        max_speed_raw: int,
        expect_write_status: bool = True,
    ) -> FeetechRotationConfig:
        """用型号固定参数和现场标定参数创建一台旋转轴。"""

        return FeetechRotationConfig(
            name=name,
            servo_id=servo_id,
            counts_per_turn=self.counts_per_turn,
            zero_raw=zero_raw,
            direction_sign=direction_sign,
            min_position_rad=min_position_rad,
            max_position_rad=max_position_rad,
            max_speed_raw=max_speed_raw,
            expect_write_status=expect_write_status,
            registers=self.registers,
        )


SM45BL_C001_PROFILE = FeetechModelProfile(
    key="sm45bl-c001",
    model="SM-45BL-C001",
    protocol="Feetech custom serial",
    transport="RS-485 half-duplex",
    default_baudrate=115200,
    counts_per_turn=4096,
    adapter_auto_direction=True,
)


FEETECH_MODEL_PROFILES = {
    SM45BL_C001_PROFILE.key: SM45BL_C001_PROFILE,
}


# 末端旋转轴当前项目配置。角度限位和速度上限用于调试阶段的安全约束，
# 最终值仍需随整机负载和机械干涉范围重新验收。
END_EFFECTOR_ROTATION_CONFIG = SM45BL_C001_PROFILE.make_rotation_config(
    name="end_effector_rotation",
    servo_id=1,
    zero_raw=2130,
    direction_sign=1,
    min_position_rad=math.radians(-150.0),
    max_position_rad=math.radians(150.0),
    max_speed_raw=500,
)
END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW = 500
END_EFFECTOR_ROTATION_POSITIVE_DIRECTION = "+X"
